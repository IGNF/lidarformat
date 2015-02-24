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



#include <liblas/point.hpp>
#include <liblas/reader.hpp>


#include "LidarFormat/LidarIOFactory.h"
#include "LidarFormat/LidarDataContainer.h"

#include "LasIO.h"

namespace Lidar
{

boost::shared_ptr<cs::LidarDataType> LasMetaDataIO::load(const std::string& filename)
{
    std::ifstream las_ifs(filename.c_str(), std::ios::binary);
    if(!las_ifs.good()) throw std::logic_error(std::string(__FUNCTION__) + ": Failed to open " + filename +"\n");
    std::cout << __FUNCTION__ << " " << filename << std::endl;
    liblas::Reader reader(las_ifs);

    liblas::Header const& header = reader.GetHeader();
    std::cout << "Signature: " << header.GetFileSignature() << std::endl;
    std::cout << "Points count: " << header.GetPointRecordsCount() << std::endl;

    cs::LidarDataType::AttributesType attributes(header.GetPointRecordsCount(),cs::DataFormatType::las);
    attributes.dataFileName() = filename;
    // attributes.centeringTransfo(cs::CenteringTransfoType(tx, ty)); // TODO: handle centering
    // TODO: handle various las versions and user fields
    attributes.attribute().push_back(cs::AttributeContainerType::AttributeType(LidarDataType::float64, "x"));
    attributes.attribute().push_back(cs::AttributeContainerType::AttributeType(LidarDataType::float64, "y"));
    attributes.attribute().push_back(cs::AttributeContainerType::AttributeType(LidarDataType::float64, "z"));
    attributes.attribute().push_back(cs::AttributeContainerType::AttributeType(LidarDataType::int32, "intensity"));
    attributes.attribute().push_back(cs::AttributeContainerType::AttributeType(LidarDataType::int32, "classification"));
    attributes.attribute().push_back(cs::AttributeContainerType::AttributeType(LidarDataType::int32, "returnNumber"));
    attributes.attribute().push_back(cs::AttributeContainerType::AttributeType(LidarDataType::int32, "numberOfReturns"));

    boost::shared_ptr<cs::LidarDataType> xmlStructure(new cs::LidarDataType(attributes));
    return xmlStructure;
}

boost::shared_ptr<LasMetaDataIO> createLasMetaDataReader()
{
    return boost::shared_ptr<LasMetaDataIO>(new LasMetaDataIO());
}

bool LasMetaDataIO::Register()
{
    MetaDataIOFactory::instance().Register(".las", createLasMetaDataReader);
    return true;
}

bool LasMetaDataIO::m_isRegistered = LasMetaDataIO::Register();


void LasIO::loadData(LidarDataContainer& lidarContainer, std::string filename)
{
    getPaths(lidarContainer, filename);
    std::ifstream las_ifs(m_data_path.c_str(), std::ios::binary);
    if(!las_ifs.good()) throw std::logic_error(std::string(__FUNCTION__) + ": Failed to open " + m_data_path +"\n");
    liblas::Reader reader(las_ifs);
    boost::uint32_t n_points = reader.GetHeader().GetPointRecordsCount();
    if(n_points != lidarContainer.size())
    {
        std::cout << __FILE__ << ":" << __LINE__ << ": WARNING: Number of points in header=" << n_points <<
                     " differs from container size " << lidarContainer.size() << "->fixing container" << std::endl;
        lidarContainer.resize(n_points);
        lidarContainer.getXmlStructure()->attributes().dataSize(n_points);
    }

    // fill the container
    int decalage_x = lidarContainer.getDecalage("x");
    int decalage_y = lidarContainer.getDecalage("y");
    int decalage_z = lidarContainer.getDecalage("z");
    int decalage_intensity = lidarContainer.getDecalage("intensity");
    int decalage_echo = lidarContainer.getDecalage("returnNumber");
    int decalage_classification = lidarContainer.getDecalage("classification");
    int decalage_numberOfReturns = lidarContainer.getDecalage("numberOfReturns");

    LidarIteratorEcho itEcho = lidarContainer.begin();
    while (reader.ReadNextPoint() ) //&& itEcho!=lidarContainer.endEcho())
    {
        liblas::Point const& p = reader.GetPoint();
        itEcho.value<double>(decalage_x) = p.GetX();
        itEcho.value<double>(decalage_y) = p.GetY();
        itEcho.value<double>(decalage_z) = p.GetZ();
        itEcho.value<int32>(decalage_intensity) = p.GetIntensity();
        itEcho.value<int32>(decalage_classification) = p.GetClassification().GetClass();
        itEcho.value<int32>(decalage_echo) = p.GetReturnNumber();
        itEcho.value<int32>(decalage_numberOfReturns) = p.GetNumberOfReturns();
        ++itEcho;
    }
}

void LasIO::save(const LidarDataContainer& lidarContainer, std::string filename)
{
    // TODO: use xml struture to write a real las file (not just a .bin)
    throw std::logic_error("Not implemented yet\n");
}

boost::shared_ptr<LasIO> createLasIO()
{
    return boost::shared_ptr<LasIO>(new LasIO());
}

bool LasIO::Register()
{
    //	std::cout << "Format cs : " << cs::DataFormatType(cs::DataFormatType::binary_one_file_ungrouped) << std::endl;
    LidarIOFactory::instance().Register(cs::DataFormatType(cs::DataFormatType::las), createLasIO);
    return true;
}


LasIO::LasIO():LidarFileIO(".las"){}

LasIO::~LasIO(){}

bool registerLasOk = LasIO::Register();

} //namespace Lidar
