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

#include "AsciiPLYArchiLidarFileIO.h"
#include "Ply2Lf.h"

namespace Lidar
{

void AsciiPLYArchiLidarFileIO::loadData(LidarDataContainer& lidarContainer, std::string filename)
{
    getPaths(lidarContainer,filename);
    LoadPlyAsciiData(m_data_path, lidarContainer);
}

void AsciiPLYArchiLidarFileIO::save(const LidarDataContainer& lidarContainer, std::string filename)
{
    getPaths(lidarContainer, filename);
    SavePly(lidarContainer, m_data_path, false);
}

boost::shared_ptr<AsciiPLYArchiLidarFileIO> createAsciiPLYArchiLidarFileReader()
{
    return boost::shared_ptr<AsciiPLYArchiLidarFileIO>(new AsciiPLYArchiLidarFileIO());
}

bool AsciiPLYArchiLidarFileIO::Register()
{
    //	std::cout << "Format cs : " << cs::DataFormatType(cs::DataFormatType::binary_one_file_ungrouped) << std::endl;
    LidarIOFactory::instance().Register(cs::DataFormatType(cs::DataFormatType::plyascii), createAsciiPLYArchiLidarFileReader);
    return true;
}

AsciiPLYArchiLidarFileIO::AsciiPLYArchiLidarFileIO():LidarFileIO(".ply"){}

bool AsciiPLYArchiLidarFileIO::m_isRegistered = AsciiPLYArchiLidarFileIO::Register();


}
