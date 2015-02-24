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
#include <boost/filesystem.hpp>

#include "BinaryLidarFileIO.h"

using namespace std;

namespace Lidar
{

boost::shared_ptr<cs::LidarDataType> StandardMetaDataIO::load(const string& filename)
{
    // filename is supposed to be xml and dataFilename given in xml
    // if not, assume filename is a datafilename and try to do something intelligent with it rather than crashing
    string xml_filename = filename;
    boost::filesystem::path file_path(filename);
    bool ext_was_changed = false;
    if(".xml" != file_path.extension().string())
    {
        xml_filename = file_path.replace_extension(".xml").string();
        ext_was_changed = true;
    }
    // load the xml (either given as argument or generated)
    //auto_ptr<cs::LidarDataType> ap(cs::lidarData(dataFileName, xml_schema::Flags::dont_validate));
    boost::shared_ptr<cs::LidarDataType> xmlStructure(cs::lidarData(xml_filename, xml_schema::Flags::dont_validate));

    if(ext_was_changed && xmlStructure->attributes().dataFileName().present())
    {
        string data_filename = xmlStructure->attributes().dataFileName().get();
        if(filename != data_filename)
            throw logic_error("Called XmlMetaDataIO::load("+filename+") but "+xml_filename+" says data is in "+data_filename+"\n");
    }
    return xmlStructure;
}

boost::shared_ptr<StandardMetaDataIO> createStandardMetaDataReader()
{
    return boost::shared_ptr<StandardMetaDataIO>(new StandardMetaDataIO());
}

bool StandardMetaDataIO::Register()
{
    MetaDataIOFactory::instance().Register(".xml", createStandardMetaDataReader);
    return true;
}

bool StandardMetaDataIO::m_isRegistered = StandardMetaDataIO::Register();


void StandardLidarFileIO::getPaths(const LidarDataContainer& lidarContainer, std::string filename)
{
    boost::filesystem::path xml_path(filename);
    xml_path.replace_extension(".xml").string();
    boost::filesystem::path data_path(lidarContainer.getDataFilename());
    if(data_path.is_relative()) data_path = xml_path.parent_path() / data_path;
    m_xml_path = xml_path.string();
    m_data_path = data_path.string();
}

void StandardLidarFileIO::saveXml(const LidarDataContainer& lidarContainer, std::string filename)
{
    getPaths(lidarContainer, filename);

    // save xml
    xml_schema::NamespaceInfomap map;
    map[""].name = "cs";
    ofstream xml_ofs(m_xml_path.c_str());
    if(!xml_ofs.good()) throw std::logic_error("StandardLidarFileIO::saveXml: " + m_xml_path + " is not writable\n");
    cs::lidarData(xml_ofs, *(lidarContainer.getXmlStructure()), map);
}

StandardLidarFileIO::~StandardLidarFileIO(){}

StandardLidarFileIO::StandardLidarFileIO():m_xml_path(""), m_data_path(""){}

} //namespace Lidar
