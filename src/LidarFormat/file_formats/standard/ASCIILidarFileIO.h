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


#ifndef ASCIILIDARFILEIO_H_
#define ASCIILIDARFILEIO_H_


#include "LidarFormat/LidarFileIO.h"

namespace Lidar
{

class ASCIILidarFileIO : public LidarFileIO
{
	public:
		virtual ~ASCIILidarFileIO();

		virtual void loadData(LidarDataContainer& lidarContainer, const XMLLidarMetaData& lidarMetaData, const XMLAttributeMetaDataContainerType& attributesDescription);
		virtual void save(const LidarDataContainer& lidarContainer, const std::string& binaryDataFileName);

		static bool Register();
		friend boost::shared_ptr<ASCIILidarFileIO> createASCIILidarFileReader();

	private:
		ASCIILidarFileIO();

		static bool m_isRegistered;
};

} //namespace Lidar

#endif /* ASCIILIDARFILEIO_H_ */
