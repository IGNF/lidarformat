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


#include "LidarFileIO.h"
#include "LidarDataContainer.h"
#include "boost/filesystem.hpp"

namespace Lidar
{

MetaDataIO::MetaDataIO()
{
}

MetaDataIO::~MetaDataIO()
{
}

void LidarFileIO::getPaths(const LidarDataContainer& lidarContainer, std::string filename)
{
    boost::filesystem::path xml_path(filename), data_path(filename);
    if(!lidarContainer.getDataFilename(m_data_path))
        data_path.replace_extension(m_ext);
    else data_path = m_data_path;
    if(data_path.is_relative()) data_path = xml_path.parent_path() / data_path;

    if(".xml" != xml_path.extension().string())
    {
        if(data_path != xml_path) // here xml_path is not an xml
            throw std::logic_error("Called " + std::string(__FUNCTION__) + "(lidarContainer," + filename +
                                   ") with lidarContainer.getDataFilename()=" + lidarContainer.getDataFilename() + "\n");
        xml_path.replace_extension(".xml").string();
    }
    m_xml_path = xml_path.string();
    m_data_path = data_path.string();
    std::cout << __FILE__ << ":" << __LINE__ << ": xml_path=" << xml_path.string() << std::endl;
    std::cout << __FILE__ << ":" << __LINE__ << ": data_path=" << data_path.string() << std::endl;

}

void LidarFileIO::setXMLData(const boost::shared_ptr<cs::LidarDataType>& xmlData)
{
}


LidarFileIO::LidarFileIO(std::string ext):m_xml_path(""), m_data_path(""),m_ext(ext)
{
}

LidarFileIO::~LidarFileIO()
{
}

} //namespace Lidar
