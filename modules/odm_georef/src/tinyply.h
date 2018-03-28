// This software is in the public domain. Where that dedication is not
// recognized, you are granted a perpetual, irrevocable license to copy,
// distribute, and modify this file as you see fit.
// Authored in 2015 by Dimitri Diakopoulos (http://www.dimitridiakopoulos.com)
// https://github.com/ddiakopoulos/tinyply
// Version 2.0

#ifndef tinyply_h
#define tinyply_h

#include <vector>
#include <string>
#include <stdint.h>
#include <sstream>
#include <memory>
#include <map>

namespace tinyply
{

    enum class Type : uint8_t
    {
        INVALID,
        INT8,
        UINT8,
        INT16,
        UINT16,
        INT32,
        UINT32,
        FLOAT32,
        FLOAT64
    };

    struct PropertyInfo
    {
        int stride;
        std::string str;
    };

    static std::map<Type, PropertyInfo> PropertyTable
    {
        { Type::INT8,{ 1, "char" } },
        { Type::UINT8,{ 1, "uchar" } },
        { Type::INT16,{ 2, "short" } },
        { Type::UINT16,{ 2, "ushort" } },
        { Type::INT32,{ 4, "int" } },
        { Type::UINT32,{ 4, "uint" } },
        { Type::FLOAT32,{ 4, "float" } },
        { Type::FLOAT64,{ 8, "double" } },
        { Type::INVALID,{ 0, "INVALID" } }
    };

    class Buffer
    {
        uint8_t * alias{ nullptr };
        struct delete_array { void operator()(uint8_t * p) { delete[] p; } };
        std::unique_ptr<uint8_t, decltype(Buffer::delete_array())> data;
        size_t size;
    public:
        Buffer() {};
        Buffer(const size_t size) : data(new uint8_t[size], delete_array()), size(size) { alias = data.get(); } // allocating
        Buffer(uint8_t * ptr) { alias = ptr; } // non-allocating, fixme: set size?
        uint8_t * get() { return alias; }
        size_t size_bytes() const { return size; }
    };

    struct PlyData
    {
        Type t;
        size_t count;
        Buffer buffer;
    };

    struct PlyProperty
    {
        PlyProperty(std::istream & is);
        PlyProperty(Type type, std::string & _name) : name(_name), propertyType(type) {}
        PlyProperty(Type list_type, Type prop_type, std::string & _name, int list_count) : name(_name), propertyType(prop_type), isList(true), listType(list_type), listCount(list_count) {}
        std::string name;
        Type propertyType;
        bool isList{ false };
        Type listType{ Type::INVALID };
        int listCount{ 0 };
    };

    struct PlyElement
    {
        PlyElement(std::istream & istream);
        PlyElement(const std::string & _name, size_t count) : name(_name), size(count) {}
        std::string name;
        size_t size;
        std::vector<PlyProperty> properties;
    };

    struct PlyFile
    {
        struct PlyFileImpl;
        std::unique_ptr<PlyFileImpl> impl;

        PlyFile();
        ~PlyFile();

        bool parse_header(std::istream & is);

        void read(std::istream & is);

        void write(std::ostream & os, bool isBinary);

        std::vector<PlyElement> get_elements() const;
        std::vector<std::string> & get_comments();
        std::vector<std::string> get_info() const;

        std::shared_ptr<PlyData> request_properties_from_element(const std::string & elementKey, const std::initializer_list<std::string> propertyKeys);
        void add_properties_to_element(const std::string & elementKey, const std::initializer_list<std::string> propertyKeys, const Type type, const size_t count, uint8_t * data, const Type listType, const size_t listCount);
    };

} // namesapce tinyply

#endif // tinyply_h
