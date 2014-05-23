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
#include "file_formats/PlyArchi/Ply2Lf.h"

using namespace boost::filesystem;

namespace Lidar
{

std::string LidarFile::getMetaData() const
{
    if(!isValid())
        throw std::logic_error("Error : Lidar xml file is not valid !\n");

    using namespace std;
    ostringstream result;
    result << "Nb of points : " << m_xmlData->attributes().dataSize() << "\n";
    result << "Format : " << m_xmlData->attributes().dataFormat() << "\n";
    result << "Binary filename : " << getBinaryDataFileName() << "\n";
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

    path fileName = path(m_xmlFileName).branch_path();

    if(m_xmlData->attributes().dataFileName().present())
    {
        path datafilename(std::string(m_xmlData->attributes().dataFileName().get()));
        // BV: check if path is absolute or relative
        if(datafilename.is_absolute()) fileName = datafilename;
        else fileName /= datafilename;
    }
    else
    {
        fileName /= (basename(m_xmlFileName) + ".bin");
    }

    return fileName.string();
}

unsigned int LidarFile::getNbPoints() const
{
    if(!isValid())
        throw std::logic_error("Error : Lidar xml file is not valid !\n");

    return (unsigned int)m_xmlData->attributes().dataSize();
}

LidarFile::LidarFile(const std::string &filename):
    m_isValid(false)
{
    // BV: we want to tolerate loading directly other formats =>
    // if ext is not .xml, do the following:
    // if there is a .xml next to it, use it
    // if not, generate it (in both cases dataFileName is relative)
    // if it cannot be generated (read only) generate it in cwd => dataFileName must be absolute
    path filepath(filename);
    if(filepath.extension().string() != ".xml") // filename is not xml, we need one
    {
        path xml_filepath(filename);
        xml_filepath.replace_extension(".xml");
        if(exists(xml_filepath)) // we found an xml, use it
        {
            m_xmlFileName = xml_filepath.string();
            std::cout << "Using existing xml file " << m_xmlFileName << std::endl;
        }
        else // no xml found, create one
        {
            m_xmlFileName = WriteXmlHeader(filename);
            m_isValid = !m_xmlFileName.empty();
            if(!m_isValid) std::cout << "Could not generate an xml file" << std::endl;
        }
    }
    else m_xmlFileName = filename; // filename is xml, use it

    // load the xml (either given as argument or generated)
    try
    {
        std::auto_ptr<cs::LidarDataType> ap(cs::lidarData(m_xmlFileName, xml_schema::Flags::dont_validate));
        m_xmlData  = boost::shared_ptr<cs::LidarDataType>(ap);
        m_isValid =  true;
    }

    catch( const std::exception &)
    {
        m_isValid =  false;
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

shared_ptr<cs::LidarDataType> LidarFile::createXMLStructure(
        const LidarDataContainer& lidarContainer,
        const std::string& xmlFileName,
        const LidarCenteringTransfo& transfo,
        const cs::DataFormatType format)
{
    //génération du fichier xml
    cs::LidarDataType::AttributesType attributes(lidarContainer.size(), format);

    //insertion des attributs dans le xml (avec leur nom et leur type)
    const AttributeMapType& attributeMap = lidarContainer.getAttributeMap();
    for(AttributeMapType::const_iterator it=attributeMap.begin(); it!=attributeMap.end(); ++it)
    {
        cs::AttributeType attrib_type(it->second.type, it->first);
        if(!it->second.dirty)
        {
            attrib_type.min(it->second.min);
            attrib_type.max(it->second.max);
        }
        attributes.attribute().push_back(attrib_type);
    }

    //cas de la transfo
    if(transfo.isSet())
    {
        attributes.centeringTransfo(cs::CenteringTransfoType(transfo.x(), transfo.y()));
    }

    shared_ptr<cs::LidarDataType> xmlStructure(new cs::LidarDataType(attributes));
    return xmlStructure;
}

void LidarFile::save(const LidarDataContainer& lidarContainer,
                     const std::string& xmlFileName,
                     const cs::LidarDataType& xmlStructure)
{
    //sauvegarde du xml
    xml_schema::NamespaceInfomap map;
    map[""].name = "cs";
    //map[""].schema = "/src/LidarFormat/models/xsd/format_me.xsd";
    std::ofstream ofs (xmlFileName.c_str());
    cs::lidarData (ofs, xmlStructure, map);

    //création du writer approprié au format grâce à la factory
    boost::shared_ptr<LidarFileIO> writer = LidarIOFactory::instance().createObject(xmlStructure.attributes().dataFormat());
    writer->save(lidarContainer, (path(xmlFileName).branch_path() / (basename(xmlFileName) + ".bin")).string());
}

void LidarFile::save(const LidarDataContainer& lidarContainer,
                     const std::string& xmlFileName,
                     const LidarCenteringTransfo& transfo,
                     const cs::DataFormatType format)
{
    shared_ptr<cs::LidarDataType> xmlStructure = createXMLStructure(lidarContainer, xmlFileName, transfo, format);

    save(lidarContainer, xmlFileName, *xmlStructure);
}

void LidarFile::save(const LidarDataContainer& lidarContainer,
                     const std::string& xmlFileName,
                     const cs::DataFormatType format)
{
    save(lidarContainer, xmlFileName, LidarCenteringTransfo(), format);
}


void LidarFile::saveInPlace(const LidarDataContainer& lidarContainer,
                            const std::string& xmlFileName)
{
    std::auto_ptr<cs::LidarDataType> xmlData(cs::lidarData(xmlFileName, xml_schema::Flags::dont_validate));

    const cs::DataFormatType format = xmlData->attributes().dataFormat();
    const std::string dataFileName = (path(xmlFileName).branch_path() / (basename(xmlFileName) + ".bin")).string();

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
        m_attributeMetaData.push_back( XMLAttributeMetaData(itAttribute->name(), itAttribute->dataType(), true,
                                                            !(itAttribute->min().present() && itAttribute->max().present()),
                                                            itAttribute->min().get(), itAttribute->max().get()));

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
            lidarContainer.addAttribute(it->name_, it->type_, it->dirty_, it->min_, it->max_);
        }
    }

}

LidarFile::~LidarFile()
{

}


} //namespace Lidar

