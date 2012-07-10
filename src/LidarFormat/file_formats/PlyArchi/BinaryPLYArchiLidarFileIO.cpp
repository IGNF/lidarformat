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

#include "BinaryPLYArchiLidarFileIO.h"

namespace Lidar
{

	void BinaryPLYArchiLidarFileIO::loadData(LidarDataContainer& lidarContainer, const XMLLidarMetaData& lidarMetaData, const XMLAttributeMetaDataContainerType& attributesDescription)
	{
		std::cout << lidarMetaData.binaryDataFileName_.c_str() << std::endl;
		std::streampos endHeader;
		{
			// ouverture du fichier en lecture ascii pour determiner la taille du header
			std::ifstream fileInASCII(lidarMetaData.binaryDataFileName_.c_str());
			if ( !fileInASCII.good() )
			{
				std::cout << "Unable to open file " << lidarMetaData.binaryDataFileName_.c_str() << std::endl;
				return;
			}
			char line[1024];
			int n_line=0;
			while( std::string(line).substr(0,10) != "end_header" )
			{
				fileInASCII.getline(line,1024);
				std::cout << line << std::endl;
				n_line++;
			}
			endHeader = fileInASCII.tellg();
			fileInASCII.close();
#ifdef _WIN32
			// BV: very subtle: windows endlines are double in ascii (\R\N), but not in binary
			endHeader -= n_line;
#endif _WIN32

			//calcul de la taille
			std::ifstream fileIn(lidarMetaData.binaryDataFileName_.c_str(), std::ios::binary );
			fileIn.seekg(0, std::ios::end);
			const unsigned int endFile = fileIn.tellg(), binSize = endFile-endHeader;
			std::cout << "End header: " << endHeader << ", end file: " << endFile << " => bin size (octets): " << binSize << std::endl;
			const unsigned int echoSize = lidarContainer.pointSize();
			std::cout << "Echo size : " << echoSize << std::endl;
			const unsigned int nbPts = binSize/echoSize;
			std::cout << "Nb points : " << nbPts << std::endl;

			if(lidarMetaData.nbPoints_ != nbPts)
				std::cout << "Attention : la structure d'attributs du fichier xml ne correspond pas au contenu du fichier binaire !" << std::endl;


			//	lidarContainer.allocate(nbPts);
		}

		std::ifstream fileIn(lidarMetaData.binaryDataFileName_.c_str(), std::ios::binary);
		if(fileIn.good())
		{
			fileIn.seekg(endHeader, std::ios::beg);
			fileIn.read(lidarContainer.rawData(), lidarMetaData.nbPoints_ * lidarContainer.pointSize());
		}
		else
			throw std::logic_error("Erreur au chargement du fichier dans BinaryPLYArchiLidarFileIO::loadData : le fichier n'existe pas ou n'est pas accessible en lecture ! \n");


	}

	void BinaryPLYArchiLidarFileIO::save(const LidarDataContainer& lidarContainer, const std::string& binaryDataFileName)
	{
		std::ofstream fileOut(binaryDataFileName.c_str(), std::ios::binary);

		if(fileOut.good())
			fileOut.write(lidarContainer.rawData(), lidarContainer.size() * lidarContainer.pointSize());
		else
			throw std::logic_error("Erreur à l'écriture du fichier dans BinaryOneFileUngroupedLidarFileReader::save : le fichier n'existe pas ou n'est pas accessible en écriture ! \n");
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

	BinaryPLYArchiLidarFileIO::BinaryPLYArchiLidarFileIO()
	{}

	bool BinaryPLYArchiLidarFileIO::m_isRegistered = BinaryPLYArchiLidarFileIO::Register();


}
