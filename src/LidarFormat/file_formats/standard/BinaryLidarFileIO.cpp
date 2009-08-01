/***********************************************************************

This file is part of the LidarFormat project source files.

LidarFormat is an open source library for efficiently handling 3D point 
clouds with a variable number of attributes at runtime. LidarFormat is 
distributed under the CeCILL-B licence. See Licence_CeCILL-B_V1-en.txt 
or http://www.cecill.info for more details.


Homepage: 

	https://fullanalyze.ign.fr/trac/LidarFormat
	
Copyright:
	
	Institut Geographique National & CEMAGREF (2009)

Author: 

	Adrien Chauve



This software is governed by the CeCILL-B license under French law and
abiding by the rules of distribution of free software.  You can  use, 
modify and/ or redistribute the software under the terms of the CeCILL-B
license as circulated by CEA, CNRS and INRIA at the following URL
"http://www.cecill.info". 

As a counterpart to the access to the source code and  rights to copy,
modify and redistribute granted by the license, users are provided only
with a limited warranty  and the software's author,  the holder of the
economic rights,  and the successive licensors  have only  limited
liability. 

In this respect, the user's attention is drawn to the risks associated
with loading,  using,  modifying and/or developing or reproducing the
software by the user in light of its specific status of free software,
that may mean  that it is complicated to manipulate,  and  that  also
therefore means  that it is reserved for developers  and  experienced
professionals having in-depth computer knowledge. Users are therefore
encouraged to load and test the software's suitability as regards their
requirements in conditions enabling the security of their systems and/or 
data to be ensured and,  more generally, to use and operate it in the 
same conditions as regards security. 

The fact that you are presently reading this means that you have had
knowledge of the CeCILL-B license and that you accept its terms.
 
***********************************************************************/

/*!
 * \file BinaryOneFileUngroupedLidarFileReader.cpp
 * \brief
 * \author Adrien Chauve
 * \date 2 déc. 2008
 */
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
		std::cout << "Taille du fichier binaire en octets : " << tailleFicOctets << std::endl;
		const unsigned int taillePt = lidarContainer.pointSize();
		std::cout << "Taille d'un enregistrement : " << taillePt << std::endl;
		const unsigned int nbPts = tailleFicOctets/taillePt;
		std::cout << "Nb de points : " << nbPts << std::endl;

		if(lidarMetaData.nbPoints_ != nbPts)
			std::cout << "Attention : la structure d'attributs du fichier xml ne correspond pas au contenu du fichier binaire !" << std::endl;


		lidarContainer.resize(nbPts);
	}

	std::ifstream fileIn(lidarMetaData.binaryDataFileName_.c_str(), std::ios::binary);
	if(fileIn.good())
		fileIn.read(lidarContainer.rawData(), lidarMetaData.nbPoints_ * lidarContainer.pointSize());
	else
		throw std::logic_error("Erreur au chargement du fichier dans BinaryOneFileUngroupedLidarFileReader::loadData : le fichier n'existe pas ou n'est pas accessible en lecture ! \n");


}

void BinaryLidarFileIO::save(const LidarDataContainer& lidarContainer, const std::string& binaryDataFileName)
{
	std::ofstream fileOut(binaryDataFileName.c_str(), std::ios::binary);

	if(fileOut.good())
		fileOut.write(lidarContainer.rawData(), lidarContainer.size() * lidarContainer.pointSize());
	else
		throw std::logic_error("Erreur à l'écriture du fichier dans BinaryOneFileUngroupedLidarFileReader::save : le fichier n'existe pas ou n'est pas accessible en écriture ! \n");

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
