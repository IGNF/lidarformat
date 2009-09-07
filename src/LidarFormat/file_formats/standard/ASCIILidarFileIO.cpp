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
 * \file ASCIILidarFileIO.cpp
 * \brief
 * \author Adrien Chauve
 * \date 2 déc. 2008
 */
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
