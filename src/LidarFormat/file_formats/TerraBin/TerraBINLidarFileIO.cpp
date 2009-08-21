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

/*
 * TerraBINLidarFileIO.cpp
 *
 *  Created on: 16 févr. 2009
 *      Author: frederic
 */

using namespace std;
#include "extern/terrabin/TerraBin.h"

#include "LidarFormat/LidarIOFactory.h"
#include "LidarFormat/LidarDataContainer.h"

#include "TerraBINLidarFileIO.h"
namespace Lidar
{

TerraBINLidarFileIO::TerraBINLidarFileIO() {
	// TODO Auto-generated constructor stub

}

TerraBINLidarFileIO::~TerraBINLidarFileIO() {
	// TODO Auto-generated destructor stub
}
void TerraBINLidarFileIO::loadData(LidarDataContainer& lidarContainer, const XMLLidarMetaData& lidarMetaData, const XMLAttributeMetaDataContainerType& attributesDescription)
{
//	std::ifstream fileIn(lidarMetaData.binaryDataFileName_.c_str());

//	if(!fileIn.good())
//		throw std::logic_error("Erreur au chargement du fichier dans TerraBINFileReader::loadData : le fichier n'existe pas ou n'est pas accessible en lecture ! \n");

	std::cout.precision(12);

	//********
	//lecture du Header
	TerraScanHeader myheader;
	ScanGetHeader(&myheader,lidarMetaData.binaryDataFileName_.c_str());

	cout<< " Format TerraBIN "<<std::endl;
//	cout<< "   HdrSize="<<myheader.HdrSize<<std::endl;
	cout<< "   HdrVersion="<<myheader.HdrVersion<<std::endl;
	cout<< "   NbPoints="<<myheader.PntCnt<<std::endl;
//	cout<< "   OrgX="<<myheader.OrgX<<std::endl;
//	cout<< "   OrgY="<<myheader.OrgY<<std::endl;
//	cout<< "   OrgZ="<<myheader.OrgZ<<std::endl;
//	cout<< "   Units="<<myheader.Units<<std::endl;

	lidarContainer.resize(myheader.PntCnt);

	cout << "Taille struct : " << sizeof(TerraRgbClr) << endl;

	//*******
	//lexture du .bin
//	TerraScanPnt  * data = new TerraScanPnt[myheader.PntCnt*sizeof(TerraScanPnt)];
//	TerraRgbClr * Clr= new TerraRgbClr[myheader.PntCnt*sizeof(TerraScanPnt)];
	//	unsigned int * sec= new unsigned int[myheader.PntCnt*sizeof(unsigned int)];
	std::vector<TerraScanPnt> data(myheader.PntCnt);
	std::vector<TerraRgbClr> Clr(myheader.PntCnt);
	std::vector<unsigned int > sec(myheader.PntCnt);
	ScanReadBinary(&data[0], &sec[0], &Clr[0], myheader.PntCnt,lidarMetaData.binaryDataFileName_.c_str());

	//parcours du data et recopie dans un lidarEcho.
	LidarEcho echo = lidarContainer.createEcho();

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

	int decalage_line=-1;
	if(find(listeAttributs.begin(),listeAttributs.end(),"line")!= listeAttributs.end())
		decalage_line = lidarContainer.getDecalage("line");



	const LidarIteratorEcho begin = lidarContainer.begin();


	for (unsigned int i=0; i<(unsigned int)myheader.PntCnt; ++i)
	{
		//lecture d'un record de TerraScanPnt
		//	X = (Pnt.X - Hdr.OrgX) / (double) Hdr.Units ;
		//	Y = (Pnt.Y - Hdr.OrgY) / (double) Hdr.Units ;
		//	Z = (Pnt.Z - Hdr.OrgZ) / (double) Hdr.Units ;

		if(decalage_x>-1)
		{
			double X = ((double)data[i].Pnt.x - myheader.OrgX) / (double) myheader.Units ;
			echo.value<double>(decalage_x)=X;
		}
		if(decalage_y>-1)
		{
			double Y = ((double)data[i].Pnt.y - myheader.OrgY) / (double) myheader.Units ;
			echo.value<double>(decalage_y)=Y;
		}
		if(decalage_z>-1)
		{
			double Z = ((double)data[i].Pnt.z - myheader.OrgZ) / (double) myheader.Units ;
			echo.value<double>(decalage_z)=Z;
		}

		if(decalage_intensity>-1)
			echo.value<short>(decalage_intensity)=data[i].Intensity;


//		0 Only echo
//		1 First of many echo
//		2 Intermediate echo
//		3 Last of many echo
		if(decalage_echo>-1)
			echo.value<short>(decalage_echo)=data[i].Echo;

		if(decalage_line>-1)
			echo.value<short>(decalage_line)=data[i].Line;


		*(begin+i) = echo;


	}
}


void TerraBINLidarFileIO::save(const LidarDataContainer& lidarContainer, const std::string& binaryDataFileName)
{
	throw std::logic_error("Non Implemente ! \n");

}



boost::shared_ptr<TerraBINLidarFileIO> createTerraBINLidarFileReader()
{
	return boost::shared_ptr<TerraBINLidarFileIO>(new TerraBINLidarFileIO());
}

bool TerraBINLidarFileIO::Register()
{
//	std::cout << "Format cs : " << cs::DataFormatType(cs::DataFormatType::binary_one_file_ungrouped) << std::endl;
	LidarIOFactory::instance().Register(cs::DataFormatType(cs::DataFormatType::terrabin), createTerraBINLidarFileReader);
	return true;
}

bool TerraBINLidarFileIO::m_isRegistered = TerraBINLidarFileIO::Register();

}//namespace Lidar
