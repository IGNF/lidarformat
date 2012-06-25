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

	Nicolas David, Olivier Tournaire
	
	

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

#include "LidarFormat/file_formats/standard/ASCIILidarFileIO.h"
#include "LidarFormat/file_formats/standard/BinaryLidarFileIO.h"
#ifdef ENABLE_PLYARCHI
#include "LidarFormat/file_formats/PlyArchi/BinaryPLYArchiLidarFileIO.h"
#endif // ENABLE_PLYARCHI
#ifdef ENABLE_TERRABIN
#include "LidarFormat/file_formats/TerraBin/TerraBINLidarFileIO.h"
#endif // ENABLE_TERRABIN
#ifdef ENABLE_LAS
#include "LidarFormat/file_formats/LAS/LasIO.h"
#endif // ENABLE_LAS

void registerAllFileFormats()
{
	using namespace Lidar;
	ASCIILidarFileIO::Register();
	BinaryLidarFileIO::Register();
#ifdef ENABLE_PLYARCHI
        BinaryPLYArchiLidarFileIO::Register();
#endif // ENABLE_PLYARCHI
#ifdef ENABLE_TERRABIN
        BinaryPLYArchiLidarFileIO::Register();
#endif // ENABLE_TERRABIN
#ifdef ENABLE_LAS
        BinaryPLYArchiLidarFileIO::Register();
#endif // ENABLE_LAS
}
