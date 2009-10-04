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
#include "LidarFormat/apply.h"

#include "ASCIILidarFileIO.h"

namespace Lidar
{

template<EnumLidarDataType T>
struct ReadValueFunctor
{
	void operator()(std::istream &is, const LidarIteratorEcho& itEcho, const unsigned int decalage)
	{
		typename LidarEnumTypeTraits<T>::type var;
		is>>var;
		itEcho.value<typename LidarEnumTypeTraits<T>::type>(decalage)=var;
	}
};

template<>
struct ReadValueFunctor<LidarDataType::int8>
{
	void operator()(std::istream &is, const LidarIteratorEcho& itEcho, const unsigned int decalage)
	{
		int var;
		is>>var;
		itEcho.value<int8>(decalage)=var;
	}
};

template<>
struct ReadValueFunctor<LidarDataType::uint8>
{
	void operator()(std::istream &is, const LidarIteratorEcho& itEcho, const unsigned int decalage)
	{
		unsigned int var;
		is>>var;
		itEcho.value<uint8>(decalage)=var;
	}
};

void ASCIILidarFileIO::loadData(LidarDataContainer& lidarContainer, const XMLLidarMetaData& lidarMetaData, const XMLAttributeMetaDataContainerType& attributesDescription)
{
	std::ifstream fileIn(lidarMetaData.binaryDataFileName_.c_str());

	if(!fileIn.good())
		throw std::logic_error("Erreur au chargement du fichier dans ASCIILidarFileReader::loadData : le fichier n'existe pas ou n'est pas accessible en lecture ! \n");

	std::cout.precision(12);

	LidarIteratorEcho itbEcho = lidarContainer.begin();
	const LidarIteratorEcho iteEcho = lidarContainer.end();

	AttributeMapType::const_iterator itMapBegin = lidarContainer.getAttributeMap().begin();
	const AttributeMapType::const_iterator ite = lidarContainer.getAttributeMap().end();

	for(; (itbEcho != iteEcho) && (fileIn.good()); ++itbEcho)
	{
		AttributeMapType::const_iterator itb = itMapBegin;
		for(; itb!=ite; ++itb)
		{
			apply<ReadValueFunctor, void, std::istream &, const LidarIteratorEcho&, const unsigned int>(itb->second.type, fileIn, itbEcho, itb->second.decalage);
		}
	}
}

void ASCIILidarFileIO::save(const LidarDataContainer& lidarContainer, const std::string& binaryDataFileName)
{
	std::ofstream fileOut(binaryDataFileName.c_str());

	if(fileOut.good())
	{
		fileOut.precision(12);
		LidarConstIteratorEcho itb(lidarContainer.begin());
		LidarConstIteratorEcho ite(lidarContainer.end());
		for(; itb!=ite; ++itb)
		{
			fileOut << LidarEcho(*itb) << "\n";
		}
	}
	else
		throw std::logic_error("Erreur à l'écriture du fichier dans ASCIILidarFileIO::save : le fichier n'existe pas ou n'est pas accessible en écriture ! \n");

}



boost::shared_ptr<ASCIILidarFileIO> createASCIILidarFileReader()
{
	return boost::shared_ptr<ASCIILidarFileIO>(new ASCIILidarFileIO());
}

bool ASCIILidarFileIO::Register()
{
//	std::cout << "Format cs : " << cs::DataFormatType(cs::DataFormatType::ascii) << std::endl;
	LidarIOFactory::instance().Register(cs::DataFormatType(cs::DataFormatType::ascii), createASCIILidarFileReader);
	return true;
}


bool ASCIILidarFileIO::m_isRegistered = ASCIILidarFileIO::Register();

ASCIILidarFileIO::ASCIILidarFileIO()
{
}

ASCIILidarFileIO::~ASCIILidarFileIO()
{
}

} //namespace Lidar
