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
	
Contributors:

	Nicolas David, Olivier Tournaire



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
 * \file LasIO.cpp
 * \brief
 * \author Adrien Chauve
 * \date 22 janv. 2009
 */

#include <liblas/laspoint.hpp>
#include <liblas/lasreader.hpp>


#include "LidarFormat/LidarIOFactory.h"
#include "LidarFormat/LidarDataContainer.h"

#include "LasIO.h"

namespace Lidar
{



void LasIO::loadData(LidarDataContainer& lidarContainer, const XMLLidarMetaData& lidarMetaData, const XMLAttributeMetaDataContainerType& attributesDescription)
{
	std::ifstream fileIn(lidarMetaData.binaryDataFileName_.c_str(), std::ios::binary);

	liblas::LASReader reader(fileIn);

	liblas::LASHeader const& header = reader.GetHeader();
	std::cout << "Reading LAS file..." << "\n";
	std::cout << "Signature: " << header.GetFileSignature() << '\n';
	std::cout << "Points count: " << header.GetPointRecordsCount() << '\n';

	lidarContainer.resize(header.GetPointRecordsCount());


//	lidarContainer.addAttribute("x", LidarDataType::float64);
//	lidarContainer.addAttribute("y", LidarDataType::float64);
//	lidarContainer.addAttribute("z", LidarDataType::float64);
//	lidarContainer.addAttribute("intensity", LidarDataType::int32);
//	lidarContainer.addAttribute("classification", LidarDataType::int32);
//	lidarContainer.addAttribute("returnNumber", LidarDataType::int32);
//	lidarContainer.addAttribute("numberOfReturns", LidarDataType::int32);

	std::vector<std::string> listeAttributs;
	lidarContainer.getAttributeList(listeAttributs);

	int decalage_x=-1;
	if(find(listeAttributs.begin(),listeAttributs.end(),"x")!= listeAttributs.end())
		decalage_x = lidarContainer.getDecalage("x");

	int decalage_y=-1;
	if(find(listeAttributs.begin(),listeAttributs.end(),"y")!= listeAttributs.end())
		decalage_y = lidarContainer.getDecalage("y");

	int decalage_z=-1;
	if(find(listeAttributs.begin(),listeAttributs.end(),"z")!= listeAttributs.end())
		decalage_z = lidarContainer.getDecalage("z");

	int decalage_intensity=-1;
	if(find(listeAttributs.begin(),listeAttributs.end(),"intensity")!= listeAttributs.end())
		decalage_intensity = lidarContainer.getDecalage("intensity");

	int decalage_echo=-1;
	if(find(listeAttributs.begin(),listeAttributs.end(),"returnNumber")!= listeAttributs.end())
		decalage_echo = lidarContainer.getDecalage("returnNumber");

	int decalage_classification=-1;
	if(find(listeAttributs.begin(),listeAttributs.end(),"classification")!= listeAttributs.end())
		decalage_classification = lidarContainer.getDecalage("classification");

	int decalage_numberOfReturns = -1;
	if(find(listeAttributs.begin(),listeAttributs.end(),"numberOfReturns")!= listeAttributs.end())
		decalage_numberOfReturns = lidarContainer.getDecalage("numberOfReturns");

	LidarIteratorEcho itEcho = lidarContainer.begin();


	while (reader.ReadNextPoint() ) //&& itEcho!=lidarContainer.endEcho())
	{
	   liblas::LASPoint const& p = reader.GetPoint();
//	   std::cout << p.GetX() << ", " << p.GetY() << ", " << p.GetZ() << "\n";


	   itEcho.value<double>(decalage_x) = p.GetX();
	   itEcho.value<double>(decalage_y) = p.GetY();
	   itEcho.value<double>(decalage_z) = p.GetZ();

	   if(decalage_intensity>0)
		   itEcho.value<int32>(decalage_intensity) = p.GetIntensity();
	   if(decalage_classification>0)
		   itEcho.value<int32>(decalage_classification) = p.GetClassification();
	   if(decalage_echo>0)
		   itEcho.value<int32>(decalage_echo) = p.GetReturnNumber();
	   if(decalage_numberOfReturns>0)
		   itEcho.value<int32>(decalage_numberOfReturns) = p.GetNumberOfReturns();



	   ++itEcho;

	}

}

void LasIO::save(const LidarDataContainer& lidarContainer, const std::string& binaryDataFileName)
{
	std::ofstream fileOut(binaryDataFileName.c_str(), std::ios::binary);

	if(fileOut.good())
		fileOut.write(lidarContainer.rawData(), lidarContainer.size() * lidarContainer.pointSize());
	else
		throw std::logic_error("Erreur à l'écriture du fichier dans BinaryOneFileUngroupedLidarFileReader::save : le fichier n'existe pas ou n'est pas accessible en écriture ! \n");

}



boost::shared_ptr<LasIO> createLasIO()
{
	return boost::shared_ptr<LasIO>(new LasIO());
}

bool LasIO::Register()
{
//	std::cout << "Format cs : " << cs::DataFormatType(cs::DataFormatType::binary_one_file_ungrouped) << std::endl;
	LidarIOFactory::Instance()->Register(cs::DataFormatType(cs::DataFormatType::las), createLasIO);
	return true;
}


LasIO::LasIO()
{
}

LasIO::~LasIO()
{
}

bool registerLasOk = LasIO::Register();

} //namespace Lidar
