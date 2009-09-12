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



#include <stdexcept>
#include <iostream>
#include <sstream>

#include "boost/filesystem.hpp"
#include <boost/shared_ptr.hpp>

#include "LidarDataContainer.h"
#include "LidarIOFactory.h"
#include "LidarFormat/geometry/LidarCenteringTransfo.h"

#include "LidarFormat/LidarFile.h"

using namespace boost::filesystem;

namespace Lidar
{

std::string LidarFile::getMetaData() const
{
	if(!isValid())
		throw std::logic_error("Error : Lidar xml file is not valid !\n");

	using namespace std;
	ostringstream result;
	result << "Nb of points : " << m_xmlData->attributes().dataSize() << endl;
	result << "Format : " << m_xmlData->attributes().dataFormat() << endl;
	result << "Binary filename : " << basename(m_xmlData->attributes().dataFileName()) << endl;
	return result.str();
}

std::string LidarFile::getFormat() const
{
	if(!isValid())
		throw std::logic_error("Error : Lidar xml file is not valid !\n");

	return m_xmlData->attributes().dataFormat();
}

std::string LidarFile::getBinaryDataFileName() const
{
	if(!isValid())
		throw std::logic_error("Error : Lidar xml file is not valid !\n");

	return path(m_xmlFileName).branch_path().string() + "/" + m_xmlData->attributes().dataFileName();
}

unsigned int LidarFile::getNbPoints() const
{
	if(!isValid())
		throw std::logic_error("Error : Lidar xml file is not valid !\n");

	return (unsigned int)m_xmlData->attributes().dataSize();
}

LidarFile::LidarFile(const std::string &xmlFileName):
	m_xmlFileName(xmlFileName), m_isValid(false)
{

	try{
		std::auto_ptr<cs::LidarDataType> ap(cs::lidarData(xmlFileName, xml_schema::Flags::dont_validate));
		m_xmlData  = boost::shared_ptr<cs::LidarDataType>(ap);
		m_isValid =  true;

//		std::cout << "nb pts : " << m_xmlData->attributes().dataSize() << "\n";
	}

	catch( const std::exception &)
	{
//		std::cout << e.what() << std::endl;
		m_isValid =  false;
//		std::cout << "invalide erreur !!\n";
	}

}

void LidarFile::loadData(LidarDataContainer& lidarContainer)
{
	if(!isValid())
		throw std::logic_error("Error : Lidar xml file is not valid !\n");

	//création du reader approprié au format grâce à la factory
	boost::shared_ptr<LidarFileIO> reader = LidarIOFactory::instance().createObject(getFormat());

	loadMetaDataFromXML();
	setMapsFromXML(lidarContainer);

	lidarContainer.resize(m_lidarMetaData.nbPoints_);

	reader->setXMLData(m_xmlData);
	reader->loadData(lidarContainer, m_lidarMetaData, m_attributeMetaData);

}

void LidarFile::loadTransfo(LidarCenteringTransfo& transfo) const
{
	transfo.setTransfo(0,0);

	if(!isValid())
		return;

	if(!m_xmlData->attributes().centeringTransfo().present())
		return;

	cs::CenteringTransfoType transfoXML = m_xmlData->attributes().centeringTransfo().get();
	transfo.setTransfo(transfoXML.tx(), transfoXML.ty());

}

shared_ptr<cs::LidarDataType> LidarFile::createXMLStructure(const LidarDataContainer& lidarContainer, const std::string& xmlFileName, const LidarCenteringTransfo& transfo, const cs::DataFormatType format)
{
	//génération du fichier xml
	const std::string dataFileName = basename(xmlFileName) + ".bin";
	cs::LidarDataType::AttributesType attributes(dataFileName, lidarContainer.size(), format);

	//insertion des attributs dans le xml (avec leur nom et leur type)
	const AttributeMapType& attributeMap = lidarContainer.getAttributeMap();
	for(AttributeMapType::const_iterator it=attributeMap.begin(); it!=attributeMap.end(); ++it)
	{
		attributes.attribute().push_back(cs::AttributeType(it->second.type, it->first));
	}

	//cas de la transfo
	if(transfo.isSet())
	{
		attributes.centeringTransfo(cs::CenteringTransfoType(transfo.x(), transfo.y()));
	}

	shared_ptr<cs::LidarDataType> xmlStructure(new cs::LidarDataType(attributes));
	return xmlStructure;
}

void LidarFile::save(const LidarDataContainer& lidarContainer, const std::string& xmlFileName, const cs::LidarDataType& xmlStructure)
{
	//sauvegarde du xml
	xml_schema::NamespaceInfomap map;
	map[""].name = "cs";
	//map[""].schema = "/src/LidarFormat/models/xsd/format_me.xsd";
	std::ofstream ofs (xmlFileName.c_str());
	cs::lidarData (ofs, xmlStructure, map);

	//création du writer approprié au format grâce à la factory
	boost::shared_ptr<LidarFileIO> writer = LidarIOFactory::instance().createObject(xmlStructure.attributes().dataFormat());
	writer->save(lidarContainer, path(xmlFileName).branch_path().string() + "/" + xmlStructure.attributes().dataFileName());
}

void LidarFile::save(const LidarDataContainer& lidarContainer, const std::string& xmlFileName, const LidarCenteringTransfo& transfo, const cs::DataFormatType format)
{
	shared_ptr<cs::LidarDataType> xmlStructure = createXMLStructure(lidarContainer, xmlFileName, transfo, format);

	save(lidarContainer, xmlFileName, *xmlStructure);
}

void LidarFile::save(const LidarDataContainer& lidarContainer, const std::string& xmlFileName, const cs::DataFormatType format)
{
	save(lidarContainer, xmlFileName, LidarCenteringTransfo(), format);
}


void LidarFile::saveInPlace(const LidarDataContainer& lidarContainer, const std::string& xmlFileName)
{
	std::auto_ptr<cs::LidarDataType> xmlData(cs::lidarData(xmlFileName, xml_schema::Flags::dont_validate));

	const cs::DataFormatType format = xmlData->attributes().dataFormat();
	const std::string dataFileName = path(xmlFileName).branch_path().string() + "/" + xmlData->attributes().dataFileName();

	//création du writer approprié au format grâce à la factory
	boost::shared_ptr<LidarFileIO> writer = LidarIOFactory::instance().createObject(format);
	writer->save(lidarContainer, dataFileName);
}



void LidarFile::loadMetaDataFromXML()
{
	//parcours du fichier xml et récupération des métadonnées sur les attributs
//	m_lidarMetaData.ptSize_ = 0;
	cs::LidarDataType::AttributesType attributes = m_xmlData->attributes();
	for (cs::LidarDataType::AttributesType::AttributeIterator itAttribute = attributes.attribute().begin(); itAttribute != attributes.attribute().end(); ++itAttribute)
	{
		//TODO adapter si on ne load pas tout
		m_attributeMetaData.push_back( XMLAttributeMetaData(itAttribute->name(), itAttribute->dataType(), true) );

	}

	//meta données générales
	m_lidarMetaData.binaryDataFileName_ = getBinaryDataFileName();


	m_lidarMetaData.nbPoints_ = (size_t)attributes.dataSize(); //tailleFicOctets/m_lidarMetaData.ptSize_;

//	std::cout << "taille d'un enregistrement : " << m_lidarMetaData.ptSize_ << std::endl;
//	double reste = double(tailleFicOctets)/double(m_lidarMetaData.ptSize_) - m_lidarMetaData.nbPoints_;
//	std::cout << "reste : " << reste << std::endl;
//	if(reste > 0)
//		throw std::logic_error("Erreur dans LidarDataContainer::loadMetaDataFromXML : la structure d'attributs du fichier xml ne correspond pas au contenu du fichier binaire ! \n");

//	//TODO passer la taille en attribut optionnel du xsd
//	if(attributes.dataSize()>0)
//		if(attributes.dataSize() != m_lidarMetaData.nbPoints_)
//			throw std::logic_error("Erreur dans LidarDataContainer::loadMetaDataFromXML : le nb de points du fichier xml ne correspond pas au contenu du fichier binaire ! \n");
}



void LidarFile::setMapsFromXML(LidarDataContainer& lidarContainer) const
{
	for(XMLAttributeMetaDataContainerType::const_iterator it = m_attributeMetaData.begin(); it != m_attributeMetaData.end(); ++it)
	{
		if(it->loaded_)
		{
			lidarContainer.addAttribute(it->name_, it->type_);
		}
	}

}

LidarFile::~LidarFile()
{
	// TODO Auto-generated destructor stub
}


} //namespace Lidar

