// This software is in the public domain. Where that dedication is not
// recognized, you are granted a perpetual, irrevocable license to copy,
// distribute, and modify this file as you see fit.
// Authored in 2015 by Dimitri Diakopoulos (http://www.dimitridiakopoulos.com)
// https://github.com/ddiakopoulos/tinyply
// Version 2.0

#include "tinyply.h"
#include <algorithm>
#include <functional>
#include <type_traits>
#include <iostream>
#include <cstring>

using namespace tinyply;
using namespace std;

//////////////////
// Endian Utils //
//////////////////

template<typename T> T endian_swap(const T & v) { return v; }
template<> inline uint16_t endian_swap(const uint16_t & v) { return (v << 8) | (v >> 8); }
template<> inline uint32_t endian_swap(const uint32_t & v) { return (v << 24) | ((v << 8) & 0x00ff0000) | ((v >> 8) & 0x0000ff00) | (v >> 24); }
template<> inline uint64_t endian_swap(const uint64_t & v)
{
    return (((v & 0x00000000000000ffLL) << 56) |
        ((v & 0x000000000000ff00LL) << 40) |
        ((v & 0x0000000000ff0000LL) << 24) |
        ((v & 0x00000000ff000000LL) << 8) |
        ((v & 0x000000ff00000000LL) >> 8) |
        ((v & 0x0000ff0000000000LL) >> 24) |
        ((v & 0x00ff000000000000LL) >> 40) |
        ((v & 0xff00000000000000LL) >> 56));
}
template<> inline int16_t endian_swap(const int16_t & v) { uint16_t r = endian_swap(*(uint16_t*)&v); return *(int16_t*)&r; }
template<> inline int32_t endian_swap(const int32_t & v) { uint32_t r = endian_swap(*(uint32_t*)&v); return *(int32_t*)&r; }
template<> inline int64_t endian_swap(const int64_t & v) { uint64_t r = endian_swap(*(uint64_t*)&v); return *(int64_t*)&r; }
inline float endian_swap_float(const uint32_t & v) { union { float f; uint32_t i; }; i = endian_swap(v); return f; }
inline double endian_swap_double(const uint64_t & v) { union { double d; uint64_t i; }; i = endian_swap(v); return d; }

/////////////////////////////
// Internal Implementation //
/////////////////////////////

inline Type property_type_from_string(const std::string & t)
{
    if (t == "int8" || t == "char")             return Type::INT8;
    else if (t == "uint8" || t == "uchar")      return Type::UINT8;
    else if (t == "int16" || t == "short")      return Type::INT16;
    else if (t == "uint16" || t == "ushort")    return Type::UINT16;
    else if (t == "int32" || t == "int")        return Type::INT32;
    else if (t == "uint32" || t == "uint")      return Type::UINT32;
    else if (t == "float32" || t == "float")    return Type::FLOAT32;
    else if (t == "float64" || t == "double")   return Type::FLOAT64;
    return Type::INVALID;
}

struct PlyFile::PlyFileImpl
{
    struct PlyCursor
    {
        size_t byteOffset;
        size_t totalSizeBytes;
    };

    struct ParsingHelper
    {
        std::shared_ptr<PlyData> data;
        std::shared_ptr<PlyCursor> cursor;
    };

    std::map<std::string, ParsingHelper> userData;

    bool isBinary = false;
    bool isBigEndian = false;
    std::vector<PlyElement> elements;
    std::vector<std::string> comments;
    std::vector<std::string> objInfo;

    void read(std::istream & is);
    void write(std::ostream & os, bool isBinary);

    std::shared_ptr<PlyData> request_properties_from_element(const std::string & elementKey, const std::initializer_list<std::string> propertyKeys);
    void add_properties_to_element(const std::string & elementKey, const std::initializer_list<std::string> propertyKeys, const Type type, const size_t count, uint8_t * data, const Type listType, const size_t listCount);

    size_t read_property_binary(const Type t, void * dest, size_t & destOffset, std::istream & is);
    size_t read_property_ascii(const Type t, void * dest, size_t & destOffset, std::istream & is);
    size_t skip_property_binary(const PlyProperty & property, std::istream & is);
    size_t skip_property_ascii(const PlyProperty & property, std::istream & is);

    bool parse_header(std::istream & is);
    void parse_data(std::istream & is, bool firstPass);
    void read_header_format(std::istream & is);
    void read_header_element(std::istream & is);
    void read_header_property(std::istream & is);
    void read_header_text(std::string line, std::vector<std::string> & place, int erase = 0);

    void write_header(std::ostream & os);
    void write_ascii_internal(std::ostream & os);
    void write_binary_internal(std::ostream & os);
    void write_property_ascii(Type t, std::ostream & os, uint8_t * src, size_t & srcOffset);
    void write_property_binary(Type t, std::ostream & os, uint8_t * src, size_t & srcOffset);
};

//////////////////
// PLY Property //
//////////////////

PlyProperty::PlyProperty(std::istream & is) : isList(false)
{
    std::string type;
    is >> type;
    if (type == "list")
    {
        std::string countType;
        is >> countType >> type;
        listType = property_type_from_string(countType);
        isList = true;
    }
    propertyType = property_type_from_string(type);
    is >> name;
}

/////////////////
// PLY Element //
/////////////////

PlyElement::PlyElement(std::istream & is)
{
    is >> name >> size;
}

///////////
// Utils //
///////////

std::string make_key(const std::string & a, const std::string & b)
{
    return (a + "-" + b);
}

template<typename T> void ply_cast(void * dest, const char * src, bool be)
{
    *(static_cast<T *>(dest)) = (be) ? endian_swap(*(reinterpret_cast<const T *>(src))) : *(reinterpret_cast<const T *>(src));
}

template<typename T> void ply_cast_float(void * dest, const char * src, bool be)
{
    *(static_cast<T *>(dest)) = (be) ? endian_swap_float(*(reinterpret_cast<const uint32_t *>(src))) : *(reinterpret_cast<const T *>(src));
}

template<typename T> void ply_cast_double(void * dest, const char * src, bool be)
{
    *(static_cast<T *>(dest)) = (be) ? endian_swap_double(*(reinterpret_cast<const uint64_t *>(src))) : *(reinterpret_cast<const T *>(src));
}

template<typename T> T ply_read_ascii(std::istream & is)
{
    T data;
    is >> data;
    return data;
}

template<typename T> void ply_cast_ascii(void * dest, std::istream & is)
{
    *(static_cast<T *>(dest)) = ply_read_ascii<T>(is);
}

size_t find_element(const std::string & key, const std::vector<PlyElement> & list)
{
    for (size_t i = 0; i < list.size(); i++) if (list[i].name == key) return i;
    return -1;
}

size_t find_property(const std::string & key, const std::vector<PlyProperty> & list)
{
    for (size_t i = 0; i < list.size(); ++i) if (list[i].name == key) return i;
    return -1;
}

//////////////
// PLY File //
//////////////

bool PlyFile::PlyFileImpl::parse_header(std::istream & is)
{
    std::string line;
    while (std::getline(is, line))
    {
        std::istringstream ls(line);
        std::string token;
        ls >> token;
        if (token == "ply" || token == "PLY" || token == "") continue;
        else if (token == "comment")    read_header_text(line, comments, 8);
        else if (token == "format")     read_header_format(ls);
        else if (token == "element")    read_header_element(ls);
        else if (token == "property")   read_header_property(ls);
        else if (token == "obj_info")   read_header_text(line, objInfo, 9);
        else if (token == "end_header") break;
        else return false;
    }
    return true;
}

void PlyFile::PlyFileImpl::read_header_text(std::string line, std::vector<std::string>& place, int erase)
{
    place.push_back((erase > 0) ? line.erase(0, erase) : line);
}

void PlyFile::PlyFileImpl::read_header_format(std::istream & is)
{
    std::string s;
    (is >> s);
    if (s == "binary_little_endian") isBinary = true;
    else if (s == "binary_big_endian") isBinary = isBigEndian = true;
}

void PlyFile::PlyFileImpl::read_header_element(std::istream & is)
{
    elements.emplace_back(is);
}

void PlyFile::PlyFileImpl::read_header_property(std::istream & is)
{
    elements.back().properties.emplace_back(is);
}

size_t PlyFile::PlyFileImpl::skip_property_binary(const PlyProperty & p, std::istream & is)
{
    static std::vector<char> skip(PropertyTable[p.propertyType].stride);
    if (p.isList)
    {
        size_t listSize = 0;
        size_t dummyCount = 0;
        read_property_binary(p.listType, &listSize, dummyCount, is);
        for (size_t i = 0; i < listSize; ++i) is.read(skip.data(), PropertyTable[p.propertyType].stride);
        return listSize * PropertyTable[p.propertyType].stride; // in bytes
    }
    else
    {
        is.read(skip.data(), PropertyTable[p.propertyType].stride);
        return PropertyTable[p.propertyType].stride;
    }
}

size_t PlyFile::PlyFileImpl::skip_property_ascii(const PlyProperty & p, std::istream & is)
{
    std::string skip;
    if (p.isList)
    {
        size_t listSize = 0;
        size_t dummyCount = 0;
        read_property_ascii(p.listType, &listSize, dummyCount, is);
        for (size_t i = 0; i < listSize; ++i) is >> skip;
        return listSize * PropertyTable[p.propertyType].stride; // in bytes
    }
    else
    {
        is >> skip;
        return PropertyTable[p.propertyType].stride;
    }
}

size_t PlyFile::PlyFileImpl::read_property_binary(const Type t, void * dest, size_t & destOffset, std::istream & is)
{
    destOffset += PropertyTable[t].stride;

    std::vector<char> src(PropertyTable[t].stride);
    is.read(src.data(), PropertyTable[t].stride);

    switch (t)
    {
        case Type::INT8:       ply_cast<int8_t>(dest, src.data(), isBigEndian);        break;
        case Type::UINT8:      ply_cast<uint8_t>(dest, src.data(), isBigEndian);       break;
        case Type::INT16:      ply_cast<int16_t>(dest, src.data(), isBigEndian);       break;
        case Type::UINT16:     ply_cast<uint16_t>(dest, src.data(), isBigEndian);      break;
        case Type::INT32:      ply_cast<int32_t>(dest, src.data(), isBigEndian);       break;
        case Type::UINT32:     ply_cast<uint32_t>(dest, src.data(), isBigEndian);      break;
        case Type::FLOAT32:    ply_cast_float<float>(dest, src.data(), isBigEndian);   break;
        case Type::FLOAT64:    ply_cast_double<double>(dest, src.data(), isBigEndian); break;
        case Type::INVALID:    throw std::invalid_argument("invalid ply property");
    }

    return PropertyTable[t].stride;
}

size_t PlyFile::PlyFileImpl::read_property_ascii(const Type t, void * dest, size_t & destOffset, std::istream & is)
{
    destOffset += PropertyTable[t].stride;

    switch (t)
    {
        case Type::INT8:       *((int8_t *)dest) = ply_read_ascii<int32_t>(is);        break;
        case Type::UINT8:      *((uint8_t *)dest) = ply_read_ascii<uint32_t>(is);      break;
        case Type::INT16:      ply_cast_ascii<int16_t>(dest, is);                      break;
        case Type::UINT16:     ply_cast_ascii<uint16_t>(dest, is);                     break;
        case Type::INT32:      ply_cast_ascii<int32_t>(dest, is);                      break;
        case Type::UINT32:     ply_cast_ascii<uint32_t>(dest, is);                     break;
        case Type::FLOAT32:    ply_cast_ascii<float>(dest, is);                        break;
        case Type::FLOAT64:    ply_cast_ascii<double>(dest, is);                       break;
        case Type::INVALID:    throw std::invalid_argument("invalid ply property");
    }
    return PropertyTable[t].stride;
}

void PlyFile::PlyFileImpl::write_property_ascii(Type t, std::ostream & os, uint8_t * src, size_t & srcOffset)
{
    switch (t)
    {
        case Type::INT8:       os << static_cast<int32_t>(*reinterpret_cast<int8_t*>(src));    break;
        case Type::UINT8:      os << static_cast<uint32_t>(*reinterpret_cast<uint8_t*>(src));  break;
        case Type::INT16:      os << *reinterpret_cast<int16_t*>(src);     break;
        case Type::UINT16:     os << *reinterpret_cast<uint16_t*>(src);    break;
        case Type::INT32:      os << *reinterpret_cast<int32_t*>(src);     break;
        case Type::UINT32:     os << *reinterpret_cast<uint32_t*>(src);    break;
        case Type::FLOAT32:    os << *reinterpret_cast<float*>(src);       break;
        case Type::FLOAT64:    os << *reinterpret_cast<double*>(src);      break;
        case Type::INVALID:    throw std::invalid_argument("invalid ply property");
    }
    os << " ";
    srcOffset += PropertyTable[t].stride;
}

void PlyFile::PlyFileImpl::write_property_binary(Type t, std::ostream & os, uint8_t * src, size_t & srcOffset)
{
    os.write((char *)src, PropertyTable[t].stride);
    srcOffset += PropertyTable[t].stride;
}

void PlyFile::PlyFileImpl::read(std::istream & is)
{
    // Parse but only get the data size
    parse_data(is, true);

    std::vector<std::shared_ptr<PlyData>> buffers;
    for (auto & entry : userData) buffers.push_back(entry.second.data);

    // Since group-requested properties share the same cursor, we need to find unique cursors so we only allocate once
    std::sort(buffers.begin(), buffers.end());
    buffers.erase(std::unique(buffers.begin(), buffers.end()), buffers.end());

    // Not great, but since we sorted by ptrs on PlyData, need to remap back onto its cursor in the userData table
    for (auto & b : buffers)
    {
        for (auto & entry : userData)
        {
            if (entry.second.data == b && b->buffer.get() == nullptr)
            {
                b->buffer = Buffer(entry.second.cursor->totalSizeBytes);
            }
        }
    }

    // Populate the data
    parse_data(is, false);
}

void PlyFile::PlyFileImpl::write(std::ostream & os, bool _isBinary)
{
    if (_isBinary) write_binary_internal(os);
    else write_ascii_internal(os);
}

void PlyFile::PlyFileImpl::write_binary_internal(std::ostream & os)
{
    isBinary = true;
    write_header(os);

    for (auto & e : elements)
    {
        for (size_t i = 0; i < e.size; ++i)
        {
            for (auto & p : e.properties)
            {
                auto & helper = userData[make_key(e.name, p.name)];
                if (p.isList)
                {
                    uint8_t listSize[4] = {0, 0, 0, 0};
                    std::memcpy(listSize, &p.listCount, sizeof(uint32_t));
                    size_t dummyCount = 0;
                    write_property_binary(p.listType, os, listSize, dummyCount);
                    for (int j = 0; j < p.listCount; ++j)
                    {
                        write_property_binary(p.propertyType, os, (helper.data->buffer.get() + helper.cursor->byteOffset), helper.cursor->byteOffset);
                    }
                }
                else
                {
                    write_property_binary(p.propertyType, os, (helper.data->buffer.get() + helper.cursor->byteOffset), helper.cursor->byteOffset);
                }
            }
        }
    }
}

void PlyFile::PlyFileImpl::write_ascii_internal(std::ostream & os)
{
    write_header(os);

    for (auto & e : elements)
    {
        for (size_t i = 0; i < e.size; ++i)
        {
            for (auto & p : e.properties)
            {
                auto & helper = userData[make_key(e.name, p.name)];
                if (p.isList)
                {
                    os << p.listCount << " ";
                    for (int j = 0; j < p.listCount; ++j)
                    {
                        write_property_ascii(p.propertyType, os, (helper.data->buffer.get() + helper.cursor->byteOffset), helper.cursor->byteOffset);
                    }
                }
                else
                {
                    write_property_ascii(p.propertyType, os, (helper.data->buffer.get() + helper.cursor->byteOffset), helper.cursor->byteOffset);
                }
            }
            os << "\n";
        }
    }
}

void PlyFile::PlyFileImpl::write_header(std::ostream & os)
{
    const std::locale & fixLoc = std::locale("C");
    os.imbue(fixLoc);

    os << "ply\n";
    if (isBinary) os << ((isBigEndian) ? "format binary_big_endian 1.0" : "format binary_little_endian 1.0") << "\n";
    else os << "format ascii 1.0\n";

    for (const auto & comment : comments) os << "comment " << comment << "\n";

    for (auto & e : elements)
    {
        os << "element " << e.name << " " << e.size << "\n";
        for (const auto & p : e.properties)
        {
            if (p.isList)
            {
                os << "property list " << PropertyTable[p.listType].str << " "
                << PropertyTable[p.propertyType].str << " " << p.name << "\n";
            }
            else
            {
                os << "property " << PropertyTable[p.propertyType].str << " " << p.name << "\n";
            }
        }
    }
    os << "end_header\n";
}

// Returns the size (in bytes)
std::shared_ptr<PlyData> PlyFile::PlyFileImpl::request_properties_from_element(const std::string & elementKey, const std::initializer_list<std::string> propertyKeys)
{
    // All requested properties in the userDataTable share the same cursor (thrown into the same flat array)
    ParsingHelper helper;
    helper.data = std::make_shared<PlyData>();
    helper.data->count = 0;
    helper.data->t = Type::INVALID;
    helper.cursor = std::make_shared<PlyCursor>();
    helper.cursor->byteOffset = 0;
    helper.cursor->totalSizeBytes = 0;

    if (elements.size() == 0) throw std::runtime_error("parsed header had no elements defined. malformed file?");
    if (!propertyKeys.size()) throw std::invalid_argument("`propertyKeys` argument is empty");
    if (elementKey.size() == 0) throw std::invalid_argument("`elementKey` argument is empty");

    const int elementIndex = find_element(elementKey, elements);

    // Sanity check if the user requested element is in the pre-parsed header
    if (elementIndex >= 0)
    {
        // We found the element
        const PlyElement & element = elements[elementIndex];

        helper.data->count = element.size;

        // Find each of the keys
        for (auto key : propertyKeys)
        {
            const int propertyIndex = find_property(key, element.properties);

            if (propertyIndex >= 0)
            {
                // We found the property
                const PlyProperty & property = element.properties[propertyIndex];

                helper.data->t = property.propertyType; // hmm....

                auto result = userData.insert(std::pair<std::string, ParsingHelper>(make_key(element.name, property.name), helper));
                if (result.second == false) throw std::invalid_argument("element-property key has already been requested: " + make_key(element.name, property.name));
            }
            else throw std::invalid_argument("one of the property keys was not found in the header: " + key);
        }
    }
    else throw std::invalid_argument("the element key was not found in the header: " + elementKey);

    return helper.data;
}

void PlyFile::PlyFileImpl::add_properties_to_element(const std::string & elementKey, const std::initializer_list<std::string> propertyKeys, const Type type, const size_t count, uint8_t * data, const Type listType, const size_t listCount)
{
    ParsingHelper helper;
    helper.data = std::make_shared<PlyData>();
    helper.data->count = count;
    helper.data->t = type;
    helper.data->buffer = Buffer(data);
    helper.cursor = std::make_shared<PlyCursor>();
    helper.cursor->byteOffset = 0;
    helper.cursor->totalSizeBytes = 0;

    auto create_property_on_element = [&](PlyElement & e)
    {
        for (auto key : propertyKeys)
        {
            PlyProperty newProp = (listType == Type::INVALID) ? PlyProperty(type, key) : PlyProperty(listType, type, key, listCount);
            /* auto result = */userData.insert(std::pair<std::string, ParsingHelper>(make_key(elementKey, key), helper));
            e.properties.push_back(newProp);
        }
    };

    int idx = find_element(elementKey, elements);
    if (idx >= 0)
    {
        PlyElement & e = elements[idx];
        create_property_on_element(e);
    }
    else
    {
        PlyElement newElement = (listType == Type::INVALID) ? PlyElement(elementKey, count / propertyKeys.size()) : PlyElement(elementKey, count / listCount);
        create_property_on_element(newElement);
        elements.push_back(newElement);
    }
}

void PlyFile::PlyFileImpl::parse_data(std::istream & is, bool firstPass)
{
    std::function<size_t(const Type t, void * dest, size_t & destOffset, std::istream & is)> read;
    std::function<size_t(const PlyProperty & p, std::istream & is)> skip;

    const auto start = is.tellg();

    if (isBinary)
    {
        read = [&](const Type t, void * dest, size_t & destOffset, std::istream & _is) { return read_property_binary(t, dest, destOffset, _is); };
        skip = [&](const PlyProperty & p, std::istream & _is) { return skip_property_binary(p, _is); };
    }
    else
    {
        read = [&](const Type t, void * dest, size_t & destOffset, std::istream & _is) { return read_property_ascii(t, dest, destOffset, _is); };
        skip = [&](const PlyProperty & p, std::istream & _is) { return skip_property_ascii(p, _is); };
    }

    for (auto & element : elements)
    {
        for (size_t count = 0; count < element.size; ++count)
        {
            for (auto & property : element.properties)
            {
                auto cursorIt = userData.find(make_key(element.name, property.name));
                if (cursorIt != userData.end())
                {
                    auto & helper = cursorIt->second;
                    if (!firstPass)
                    {
                        if (property.isList)
                        {
                            size_t listSize = 0;
                            size_t dummyCount = 0;
                            read(property.listType, &listSize, dummyCount, is);
                            for (size_t i = 0; i < listSize; ++i)
                            {
                                read(property.propertyType, (helper.data->buffer.get() + helper.cursor->byteOffset), helper.cursor->byteOffset, is);
                            }
                        }
                        else
                        {
                            read(property.propertyType, (helper.data->buffer.get() + helper.cursor->byteOffset), helper.cursor->byteOffset, is);
                        }
                    }
                    else
                    {
                        helper.cursor->totalSizeBytes += skip(property, is);
                    }
                }
                else
                {
                    skip(property, is);
                }
            }
        }
    }

    // Reset istream reader to the beginning
    if (firstPass) is.seekg(start, is.beg);
}

///////////////////////////////////
// Pass-Through Public Interface //
///////////////////////////////////

PlyFile::PlyFile() { impl.reset(new PlyFileImpl()); };
PlyFile::~PlyFile() { };
bool PlyFile::parse_header(std::istream & is) { return impl->parse_header(is); }
void PlyFile::read(std::istream & is) { return impl->read(is); }
void PlyFile::write(std::ostream & os, bool isBinary) { return impl->write(os, isBinary); }
std::vector<PlyElement> PlyFile::get_elements() const { return impl->elements; }
std::vector<std::string> & PlyFile::get_comments() { return impl->comments; }
std::vector<std::string> PlyFile::get_info() const { return impl->objInfo; }
std::shared_ptr<PlyData> PlyFile::request_properties_from_element(const std::string & elementKey, const std::initializer_list<std::string> propertyKeys)
{
    return impl->request_properties_from_element(elementKey, propertyKeys);
}
void PlyFile::add_properties_to_element(const std::string & elementKey, const std::initializer_list<std::string> propertyKeys, const Type type, const size_t count, uint8_t * data, const Type listType, const size_t listCount)
{
    return impl->add_properties_to_element(elementKey, propertyKeys, type, count, data, listType, listCount);
}
