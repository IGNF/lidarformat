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

    Nicolas David, Olivier Tournaire, Bruno Vallet



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
#ifdef ENABLE_LAS
#include "file_formats/LAS/Las2Lf.h"
#endif // ENABLE_LAS

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
    m_xmlFileName(filename), m_isValid(false)
{
    path filepath(filename);
    std::string ext = filepath.extension().string();

    // BV: we want to tolerate loading directly other formats =>
    // there is now a MetaDataIOFactory to create meta data readers for all supported formats
    boost::shared_ptr<MetaDataIO> reader = MetaDataIOFactory::instance().createObject(ext);

    // load the xml (either given as argument or generated)
    try
    {
        m_xmlData = reader->load(filename);
        m_isValid = true;
    }
    catch( const std::exception &)
    {
        m_isValid = false;
    }
}

void LidarFile::loadData(LidarDataContainer& lidarContainer)
{
    if(!isValid())
        throw std::logic_error("Error : Lidar xml file is not valid !\n");

    //loadMetaDataFromXML(); // BV this is done by constructor
    lidarContainer.setMapsFromXML(m_xmlData);
    lidarContainer.resize(m_xmlData->attributes().dataSize());

    boost::shared_ptr<LidarFileIO> reader = LidarIOFactory::instance().createObject(m_xmlData->attributes().dataFormat());
    reader->loadData(lidarContainer, m_xmlFileName);
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
        const std::string& dataFileName,
        const LidarCenteringTransfo& transfo,
        const cs::DataFormatType format)
{
    // generate xml file
    cs::LidarDataType::AttributesType attributes(lidarContainer.size(), format);

    // insert attributes in the xml (with their names and types)
    const AttributeMapType& attributeMap = lidarContainer.getAttributeMap();
    for(AttributeMapType::const_iterator it=attributeMap.begin(); it!=attributeMap.end(); ++it)
    {
        attributes.attribute().push_back(it->second);
    }

    // add transfo
    if(transfo.isSet())
    {
        attributes.centeringTransfo(cs::CenteringTransfoType(transfo.x(), transfo.y()));
    }

    // add dataFilename
    attributes.dataFileName() = dataFileName;

    shared_ptr<cs::LidarDataType> xmlStructure(new cs::LidarDataType(attributes));
    return xmlStructure;
}

// BV: create extention based on format
void LidarFile::save(LidarDataContainer& lidarContainer,
                     const std::string& filename,
                     const LidarCenteringTransfo& transfo,
                     const cs::DataFormatType format)
{
    std::string ext;
    switch(format)
    {
    case cs::DataFormatType::binary: ext=".bin"; break;
    case cs::DataFormatType::ascii: ext=".txt"; break;
    case cs::DataFormatType::plyarchi: ext=".ply"; break;
    case cs::DataFormatType::las: ext=".las"; break;
    case cs::DataFormatType::terrabin: ext=".terrabin"; break;
    default: ext=".bin";
    }
    path dataFilePath(filename);
    dataFilePath.replace_extension(ext);

    lidarContainer.updateXMLStructure(dataFilePath.string(), format, transfo);
    boost::shared_ptr<LidarFileIO> writer = LidarIOFactory::instance().createObject(format);
    writer->save(lidarContainer, dataFilePath.string());
}

void LidarFile::save(LidarDataContainer& lidarContainer,
                     const std::string& xmlFileName,
                     const cs::DataFormatType format)
{
    double x=0., y=0.;
    lidarContainer.getCenteringTransfo(x,y);
    save(lidarContainer, xmlFileName, LidarCenteringTransfo(x,y), format);
}

// BV: guess format from given extention
void LidarFile::save(LidarDataContainer& lidarContainer,
                     const std::string& filename,
                     const LidarCenteringTransfo& transfo)
{
    // format should be inferred from dataFileName extension
    path dataFilePath(filename);
    std::string ext = dataFilePath.extension().string();
    cs::DataFormatType format = cs::DataFormatType::binary; // default (includes .xml and .bin)
    if(".txt" == ext) format = cs::DataFormatType::ascii;
    else if(".ply" == ext) format = cs::DataFormatType::plyarchi;
    else if(".asc" == ext) format = cs::DataFormatType::ascii;
    else if(".terrabin" == ext) format = cs::DataFormatType::terrabin;
    else if(".las" == ext) format = cs::DataFormatType::las;
    // if user gives .xml filename without specifying format, assume he wants binary
    else if(".xml" == ext) dataFilePath.replace_extension(".bin");

    // dataFilePath.filename() = relative path between .xml and data, only used by lidarformat formats
    lidarContainer.updateXMLStructure(dataFilePath.filename().string(), format, transfo);
    boost::shared_ptr<LidarFileIO> writer = LidarIOFactory::instance().createObject(format);
    writer->save(lidarContainer, filename);
}

void LidarFile::save(LidarDataContainer& lidarContainer,
                     const std::string& dataFileName)
{
    double x=0., y=0.;
    lidarContainer.getCenteringTransfo(x,y);
    save(lidarContainer, dataFileName, LidarCenteringTransfo(x,y));
}


void LidarFile::saveInPlace(LidarDataContainer& lidarContainer,
                            const std::string& xmlFileName)
{
    shared_ptr<cs::LidarDataType> xmlData(cs::lidarData(xmlFileName, xml_schema::Flags::dont_validate));

    const cs::DataFormatType format = xmlData->attributes().dataFormat();
    const std::string dataFileName = (path(xmlFileName).branch_path() / (basename(xmlFileName) + ".bin")).string();

    // create writer appropriate to format using the factory
    lidarContainer.updateXMLStructure(dataFileName, format, LidarCenteringTransfo());
    boost::shared_ptr<LidarFileIO> writer = LidarIOFactory::instance().createObject(format);
    writer->save(lidarContainer, xmlFileName);
}



void LidarFile::loadMetaDataFromXML()
{
    // BV: pas besoin de dupliquer une information qu'on a déj� , en plus on fige la structure donc on perd tout l'interet de xsd
    //parcours du fichier xml et récupération des métadonnées sur les attributs
    //	m_lidarMetaData.ptSize_ = 0;
    /*cs::LidarDataType::AttributesType attributes = m_xmlData->attributes();
    for (cs::LidarDataType::AttributesType::AttributeIterator itAttribute = attributes.attribute().begin(); itAttribute != attributes.attribute().end(); ++itAttribute)
    {
        //TODO adapter si on ne load pas tout
        m_attributeMetaData.push_back( XMLAttributeMetaData(itAttribute->name(), itAttribute->dataType(), true,
                                                            !(itAttribute->min().present() && itAttribute->max().present()),
                                                            itAttribute->min().get(), itAttribute->max().get()));

    }*/

    //general meta data
    //m_lidarMetaData.binaryDataFileName_ = getBinaryDataFileName();
    //m_lidarMetaData.nbPoints_ = (size_t)m_xmlData->attributes().dataSize(); //tailleFicOctets/m_lidarMetaData.ptSize_;

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
    lidarContainer.setMapsFromXML(m_xmlData);
}

LidarFile::~LidarFile()
{

}


} //namespace Lidar

