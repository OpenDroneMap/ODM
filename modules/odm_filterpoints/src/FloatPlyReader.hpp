/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#pragma once
#include <pdal/io/PlyReader.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Options.hpp>
#include <pdal/Filter.hpp>

#include <stack>

#include <pdal/Dimension.hpp>
#include <pdal/Reader.hpp>
#include <pdal/StageFactory.hpp>

namespace pdal
{

class PDAL_DLL FloatPlyReader : public Reader
{
public:
    std::string getName() const;

    typedef std::map<std::string, Dimension::Id> DimensionMap;

    FloatPlyReader();

private:
    enum class Format
    {
        Ascii,
        BinaryLe,
        BinaryBe
    };

    struct Property
    {
        Property(const std::string& name) : m_name(name)
        {}
        virtual ~Property()
        {}

        std::string m_name;

        virtual void setDim(Dimension::Id id)
        {}
        virtual void read(std::istream *stream, FloatPlyReader::Format format,
            PointRef& point) = 0;
    };

    struct SimpleProperty : public Property
    {
        SimpleProperty(const std::string& name, Dimension::Type type) :
            Property(name), m_type(type), m_dim(Dimension::Id::Unknown)
        {}

        Dimension::Type m_type;
        Dimension::Id m_dim;

        virtual void read(std::istream *stream, FloatPlyReader::Format format,
            PointRef& point) override;
        virtual void setDim(Dimension::Id id) override
        { m_dim = id; }
    };

    struct ListProperty : public Property
    {
        ListProperty(const std::string& name, Dimension::Type countType,
            Dimension::Type listType) : Property(name), m_countType(countType),
            m_listType(listType)
        {}

        Dimension::Type m_countType;
        Dimension::Type m_listType;

        virtual void read(std::istream *stream, FloatPlyReader::Format format,
            PointRef& point) override;
    };

    struct Element
    {
        Element(const std::string name, size_t count) :
            m_name(name), m_count(count)
        {}

        std::string m_name;
        size_t m_count;
        std::vector<std::unique_ptr<Property>> m_properties;
    };

    Format m_format;
    std::string m_line;
    std::string::size_type m_linePos;
    std::stack<std::string> m_lines;
    std::istream *m_stream;
    std::istream::streampos m_dataPos;
    std::vector<Element> m_elements;
    PointId m_index;
    Element *m_vertexElt;

    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t num);
    virtual void done(PointTableRef table);
    virtual bool processOne(PointRef& point);

    std::string readLine();
    void pushLine();
    std::string nextWord();
    void extractMagic();
    void extractEnd();
    void extractFormat();
    Dimension::Type getType(const std::string& name);
    void extractProperty(Element& element);
    void extractProperties(Element& element);
    bool extractElement();
    void extractHeader();
    void readElement(Element& elt, PointRef& point);
    bool readProperty(Property *prop, PointRef& point);
};

} // namespace pdal

