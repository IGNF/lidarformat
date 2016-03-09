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

#include "BinaryPLYArchiLidarFileIO.h"
#include "Ply2Lf.h"

namespace Lidar
{

void BinaryPLYArchiLidarFileIO::loadData(LidarDataContainer& lidarContainer, std::string filename)
{
    getPaths(lidarContainer,filename);
    // BV: very subtle problem: windows endlines are counted double in ascii (\R\N), but not in binary (0a)
    // so under windows there will be a shift between ascii and binary positions
    // My solution: we get the data size from the lidarContainer and get the end header position from end of file based on it
    std::ifstream ply_ifs(m_data_path.c_str(), std::ios::binary);
    if(!ply_ifs.good()) throw std::logic_error(std::string(__FUNCTION__) + ": Failed to open " + m_data_path +"\n");
    ply_ifs.seekg(0, std::ios::end);
    const int dataSize = lidarContainer.size()*lidarContainer.pointSize();
    if(ply_ifs.tellg() < dataSize)
    {
        std::ostringstream oss;
        oss << "BinaryPLYArchiLidarFileIO::loadData: binary file size " << ply_ifs.tellg() << "< what xml expects: " << dataSize << std::endl;
        oss <<  "lidarContainer.size()=" << lidarContainer.size() << ", lidarContainer.pointSize()=" << lidarContainer.pointSize() << std:: endl;
        throw std::logic_error(oss.str());
    }
    ply_ifs.seekg(-dataSize, std::ios::end);
    //std::cout << "Binary part starts at " << fileInBin.tellg() << std::endl;
    // lidarContainer.allocate(lidarMetaData.nbPoints_); // BV: do we need that ? works well without
    ply_ifs.read(lidarContainer.rawData(), dataSize);
}


void BinaryPLYArchiLidarFileIO::save(const LidarDataContainer& lidarContainer, std::string filename)
{
    getPaths(lidarContainer, filename);
    SavePly(lidarContainer, m_data_path);
}

boost::shared_ptr<BinaryPLYArchiLidarFileIO> createBinaryPLYArchiLidarFileReader()
{
    return boost::shared_ptr<BinaryPLYArchiLidarFileIO>(new BinaryPLYArchiLidarFileIO());
}

bool BinaryPLYArchiLidarFileIO::Register()
{
    //	std::cout << "Format cs : " << cs::DataFormatType(cs::DataFormatType::binary_one_file_ungrouped) << std::endl;
    LidarIOFactory::instance().Register(cs::DataFormatType(cs::DataFormatType::plyarchi), createBinaryPLYArchiLidarFileReader);
    return true;
}

BinaryPLYArchiLidarFileIO::BinaryPLYArchiLidarFileIO():LidarFileIO(".ply"){}

bool BinaryPLYArchiLidarFileIO::m_isRegistered = BinaryPLYArchiLidarFileIO::Register();


}
