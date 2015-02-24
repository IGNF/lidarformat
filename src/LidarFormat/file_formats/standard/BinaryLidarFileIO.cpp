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

namespace Lidar
{

void BinaryLidarFileIO::loadData(LidarDataContainer& lidarContainer, std::string filename)
{
    getPaths(lidarContainer, filename);

    // compute file size
    {
        std::ifstream data_file(m_data_path.c_str(), std::ios::binary);
        if(!data_file.good()) throw std::logic_error("BinaryLidarFileIO::loadData: Failed to open " + m_data_path +"\n");

        data_file.seekg(0, std::ios::end);
        const unsigned int bin_file_size = data_file.tellg();
        std::cout << "Binary file size: " << bin_file_size << std::endl;
        std::cout << "Echo size:" << lidarContainer.pointSize() << std::endl;
        const unsigned int n_points = bin_file_size/lidarContainer.pointSize();
        std::cout << "Nb of points : " << n_points << std::endl;

        if(lidarContainer.size() != n_points)
        {
            std::cout << "Warning : xml structure does not match binary file size->fixing container" << std::endl;
            lidarContainer.resize(n_points);
            lidarContainer.getXmlStructure()->attributes().dataSize(n_points);
        }
    }

    std::ifstream data_file(m_data_path.c_str(), std::ios::binary);
    if(data_file.good())
        data_file.read(lidarContainer.rawData(), lidarContainer.size() * lidarContainer.pointSize());
    else throw std::logic_error("BinaryLidarFileIO::loadData: Failed to open " + m_data_path +"\n");
}

void BinaryLidarFileIO::save(const LidarDataContainer& lidarContainer, std::string filename)
{
    saveXml(lidarContainer, filename);

    // save bin
    std::ofstream bin_ofs(m_data_path.c_str(), std::ios::binary);
    if(bin_ofs.good())
        bin_ofs.write(lidarContainer.rawData(), lidarContainer.size() * lidarContainer.pointSize());
    else throw std::logic_error("BinaryLidarFileIO::loadData: Failed to open " + m_data_path +"\n");
}

boost::shared_ptr<BinaryLidarFileIO> createBinaryLidarFileReader()
{
    return boost::shared_ptr<BinaryLidarFileIO>(new BinaryLidarFileIO());
}

bool BinaryLidarFileIO::Register()
{
    LidarIOFactory::instance().Register(cs::DataFormatType(cs::DataFormatType::binary), createBinaryLidarFileReader);
    return true;
}


BinaryLidarFileIO::BinaryLidarFileIO():StandardLidarFileIO(".bin"){}

BinaryLidarFileIO::~BinaryLidarFileIO(){}

bool BinaryLidarFileIO::m_isRegistered = BinaryLidarFileIO::Register();

} //namespace Lidar
