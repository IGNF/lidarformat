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
