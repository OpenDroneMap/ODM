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

#include <pdal/PointView.hpp>
#include <pdal/Writer.hpp>

namespace pdal
{

class Triangle;

class PDAL_DLL ModifiedPlyWriter : public Writer
{
public:
    enum class Format
    {
        Ascii,
        BinaryLe,
        BinaryBe
    };

    std::string getName() const;

    ModifiedPlyWriter();

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void prepared(PointTableRef table);
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr data);
    virtual void done(PointTableRef table);

    std::string getType(Dimension::Type type) const;
    void writeHeader(PointLayoutPtr layout) const;
    void writeValue(PointRef& point, Dimension::Id dim, Dimension::Type type);
    void writePoint(PointRef& point, PointLayoutPtr layout);
    void writeTriangle(const Triangle& t, size_t offset);

    std::ostream *m_stream;
    std::string m_filename;
    Format m_format;
    bool m_faces;
    StringList m_dimNames;
    Dimension::IdList m_dims;
    int m_precision;
    Arg *m_precisionArg;
    std::vector<PointViewPtr> m_views;
};

inline std::istream& operator>>(std::istream& in, ModifiedPlyWriter::Format& f)
{
    std::string s;
    std::getline(in, s);
    Utils::trim(s);
    Utils::tolower(s);
    if (s == "ascii" || s == "default")
        f = ModifiedPlyWriter::Format::Ascii;
    else if (s == "little endian" || s == "binary_little_endian")
        f = ModifiedPlyWriter::Format::BinaryLe;
    else if (s == "big endian" || s == "binary_big_endian")
        f = ModifiedPlyWriter::Format::BinaryBe;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}


inline std::ostream& operator<<(std::ostream& out, const ModifiedPlyWriter::Format& f)
{
    switch (f)
    {
    case ModifiedPlyWriter::Format::Ascii:
        out << "ascii";
        break;
    case ModifiedPlyWriter::Format::BinaryLe:
        out << "binary_little_endian";
        break;
    case ModifiedPlyWriter::Format::BinaryBe:
        out << "binary_big_endian";
        break;
    }
    return out;
}

}
