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
    boost::filesystem::path xml_path(filename), data_path(filename);
    bool ext_was_changed = false;
    if(".xml" != xml_path.extension().string())
    {
        xml_path.replace_extension(".xml");
        ext_was_changed = true;
    }
    if(!boost::filesystem::exists(xml_path))
    {
        std::cout << xml_path << " does not exist" << std::endl;
        xml_path = xml_path.parent_path() / "common.xml";
        if(!boost::filesystem::exists(xml_path))
        {
            throw logic_error("No .xml file found for StandardMetaDataIO::load("+filename+")\n");
        } else std::cout << "StandardMetaDataIO::load using " << xml_path << endl;
    }
    // load the xml (either given as argument or generated)
    //auto_ptr<cs::LidarDataType> ap(cs::lidarData(dataFileName, xml_schema::Flags::dont_validate));
    boost::shared_ptr<cs::LidarDataType> xmlStructure(cs::lidarData(xml_path.string(), xml_schema::Flags::dont_validate));

    if(ext_was_changed && xmlStructure->attributes().dataFileName().present())
    {
        boost::filesystem::path data_path_from_xml(xmlStructure->attributes().dataFileName().get());
        if(data_path_from_xml.filename() != data_path.filename())
            throw logic_error("Called StandardMetaDataIO::load("+filename+") but "+xml_path.string()
                              +" says data is in "+data_path_from_xml.string()+"\n");
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
    MetaDataIOFactory::instance().Register(".bin", createStandardMetaDataReader);
    MetaDataIOFactory::instance().Register(".txt", createStandardMetaDataReader);
    return true;
}

bool StandardMetaDataIO::m_isRegistered = StandardMetaDataIO::Register();


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

StandardLidarFileIO::StandardLidarFileIO(std::string ext):LidarFileIO(ext){}

} //namespace Lidar
