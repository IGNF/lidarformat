/***********************************************************************

This file is part of the LidarFormat project source files.

LidarFormat is an open source library for efficiently handling 3D point 
clouds with a variable number of attributes at runtime. 


Homepage: 

    http://code.google.com/p/lidarformat

Copyright:

    Institut Geographique National & CEMAGREF (2009)

Author: 

    Adrien Chauve

Contributors:

    Nicolas David, Olivier Tournaire, Bruno Vallet



    LidarFormat is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    LidarFormat is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with LidarFormat.  If not, see <http://www.gnu.org/licenses/>.

***********************************************************************/


#include "LidarFormat/LidarIOFactory.h"
#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/apply.h"

#include "ASCIILidarFileIO.h"
#include <boost/filesystem.hpp>

namespace Lidar
{

template<EnumLidarDataType T>
struct ReadValueFunctor
{
    void operator()(std::istream &is, const LidarIteratorEcho& itEcho, const unsigned int decalage)
    {
        typename LidarEnumTypeTraits<T>::type var;
        is>>var;
        itEcho.value<typename LidarEnumTypeTraits<T>::type>(decalage)=var;
    }
};

template<>
struct ReadValueFunctor<LidarDataType::int8>
{
    void operator()(std::istream &is, const LidarIteratorEcho& itEcho, const unsigned int decalage)
    {
        int var;
        is>>var;
        itEcho.value<int8>(decalage)=var;
    }
};

template<>
struct ReadValueFunctor<LidarDataType::uint8>
{
    void operator()(std::istream &is, const LidarIteratorEcho& itEcho, const unsigned int decalage)
    {
        unsigned int var;
        is>>var;
        itEcho.value<uint8>(decalage)=var;
    }
};

// BV: allow , ; : as separators by changing the locale
struct field_reader: std::ctype<char>
{
    field_reader(): std::ctype<char>(get_table()) {}

    static std::ctype_base::mask const* get_table() {
        static std::vector<std::ctype_base::mask>
                rc(table_size, std::ctype_base::mask());

        rc['\n'] = std::ctype_base::space;
        rc[' '] = std::ctype_base::space;
        rc[','] = std::ctype_base::space;
        rc[';'] = std::ctype_base::space;
        rc[':'] = std::ctype_base::space;
        return &rc[0];
    }
};

void ASCIILidarFileIO::loadData(LidarDataContainer& lidarContainer, std::string filename)
{
    getPaths(lidarContainer, filename);
    std::ifstream data_file(m_data_path.c_str());
    if(!data_file.good()) throw std::logic_error(std::string(__FUNCTION__) + ": Failed to open " + m_data_path +"\n");
    data_file.imbue(std::locale(std::locale(), new field_reader())); // use the redefined locale
    std::cout.precision(12);

    LidarIteratorEcho itbEcho = lidarContainer.begin();
    const LidarIteratorEcho iteEcho = lidarContainer.end();

    AttributeMapType::const_iterator itMapBegin = lidarContainer.getAttributeMap().begin();
    const AttributeMapType::const_iterator ite = lidarContainer.getAttributeMap().end();

    for(; (itbEcho != iteEcho) && (data_file.good()); ++itbEcho)
    {
        AttributeMapType::const_iterator itb = itMapBegin;
        for(; itb!=ite; ++itb)
        {
            apply<ReadValueFunctor, void, std::istream &, const LidarIteratorEcho&, const unsigned int>(
                        itb->second.dataType(), data_file, itbEcho, itb->second.decalage);
        }
    }
}


void ASCIILidarFileIO::save(const LidarDataContainer& lidarContainer, std::string filename)
{
    saveXml(lidarContainer, filename);

    // save txt
    std::ofstream txt_ofs(m_data_path.c_str());
    if(!txt_ofs.good()) throw std::logic_error(std::string(__FUNCTION__) + ": Failed to open " + m_data_path +"\n");
    txt_ofs.precision(12);
    LidarConstIteratorEcho itb(lidarContainer.begin());
    LidarConstIteratorEcho ite(lidarContainer.end());
    for(; itb!=ite; ++itb)
    {
        txt_ofs << LidarEcho(*itb) << "\n";
    }
}


boost::shared_ptr<ASCIILidarFileIO> createASCIILidarFileReader()
{
    return boost::shared_ptr<ASCIILidarFileIO>(new ASCIILidarFileIO());
}

bool ASCIILidarFileIO::Register()
{
    //	std::cout << "Format cs : " << cs::DataFormatType(cs::DataFormatType::ascii) << std::endl;
    LidarIOFactory::instance().Register(cs::DataFormatType(cs::DataFormatType::ascii), createASCIILidarFileReader);
    return true;
}


bool ASCIILidarFileIO::m_isRegistered = ASCIILidarFileIO::Register();

ASCIILidarFileIO::ASCIILidarFileIO():StandardLidarFileIO()
{
}

ASCIILidarFileIO::~ASCIILidarFileIO()
{
}

} //namespace Lidar
