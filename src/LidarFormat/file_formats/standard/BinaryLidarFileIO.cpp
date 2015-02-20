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


#include "LidarFormat/LidarIOFactory.h"
#include "LidarFormat/LidarDataContainer.h"


#include "BinaryLidarFileIO.h"

namespace Lidar
{

void BinaryLidarFileIO::loadData(LidarDataContainer& lidarContainer, const XMLLidarMetaData& lidarMetaData, const XMLAttributeMetaDataContainerType& attributesDescription)
{
	{
		//calcul de la taille
		std::ifstream fileIn(lidarMetaData.binaryDataFileName_.c_str(), std::ios::binary );
		fileIn.seekg(0, std::ios::end);
		const unsigned int tailleFicOctets = fileIn.tellg();
        std::cout << "Binary file size: " << tailleFicOctets << std::endl;
		const unsigned int taillePt = lidarContainer.pointSize();
        std::cout << "Echo size:" << taillePt << std::endl;
		const unsigned int nbPts = tailleFicOctets/taillePt;
        std::cout << "Nb of points : " << nbPts << std::endl;

		if(lidarMetaData.nbPoints_ != nbPts)
            std::cout << "Warning : xml structure does not match binary file!" << std::endl;


		lidarContainer.resize(nbPts);
	}

	std::ifstream fileIn(lidarMetaData.binaryDataFileName_.c_str(), std::ios::binary);
	if(fileIn.good())
		fileIn.read(lidarContainer.rawData(), lidarMetaData.nbPoints_ * lidarContainer.pointSize());
	else
        throw std::logic_error("Error writing file in BinaryLidarFileIO::loadData : file does not exist or cannot be read! \n");


}

void BinaryLidarFileIO::save(const LidarDataContainer& lidarContainer,
                             const cs::LidarDataType& xmlStructure,
                             const std::string& binaryDataFileName)
{
	std::ofstream fileOut(binaryDataFileName.c_str(), std::ios::binary);

	if(fileOut.good())
		fileOut.write(lidarContainer.rawData(), lidarContainer.size() * lidarContainer.pointSize());
	else
        throw std::logic_error("Error writing file in BinaryLidarFileIO::save : file does not exist or is read only! \n");

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


BinaryLidarFileIO::BinaryLidarFileIO()
{
}

BinaryLidarFileIO::~BinaryLidarFileIO()
{
}

bool BinaryLidarFileIO::m_isRegistered = BinaryLidarFileIO::Register();

} //namespace Lidar
