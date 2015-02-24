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
#include <fstream>
#include <algorithm>
#include <stdio.h>
using namespace std;

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <boost/noncopyable.hpp>

#include "LidarFormat/LidarDataFormatTypes.h"
#include "LidarFormat/LidarFile.h"
#include "LidarFormat/tools/AttributeBounds.h"
#include "apply.h"

#include "LidarDataContainer.h"




namespace Lidar
{

void LidarDataContainer::clear()
{
    lidarData_.clear();
}


std::ostream &LidarDataContainer::printHeader(std::ostream &os) const
{
    for (AttributeMapType::iterator it = attributeMap_->begin(); it != attributeMap_->end(); ++it)
    {
        os << it->first << "\t";
    }
    os << std::endl;
    return os;
}


void LidarDataContainer::getAttributeList(std::vector<std::string> &liste) const
{
    liste.clear();
    std::transform(attributeMap_->begin(), attributeMap_->end(), std::back_inserter(liste),
                   boost::bind( &AttributeMapType::value_type::first, _1 )
                   );
}

bool LidarDataContainer::getAttributeBounds(const std::string &attributeName, double & min, double & max) const
{
    AttributeMapType::iterator it = attributeMap_->find(attributeName);
    assert(it != attributeMap_->end());
    AttributesInfo & info = it->second;
    if(info.Dirty()) return false;
    min = info.min().get(); max = info.max().get();
    return true;
}

void LidarDataContainer::getAttributeCleanBounds(const std::string &attributeName, double & min, double & max, bool force_recompute)
{
    AttributeMapType::iterator it = attributeMap_->find(attributeName);
    assert(it != attributeMap_->end());
    AttributesInfo & info = it->second;
    if(force_recompute || info.Dirty())
    {
        FonctorMultiAbstractBound fmab;
        fmab.AddAttribute(it->first, info.decalage, info.dataType());
        fmab = std::for_each (begin(), end(), fmab);
        fmab.mvp_attrib[0]->Get(min, max);
        info.min(min); info.max(max);
    }
    min = info.min().get(); max = info.max().get();
}

void LidarDataContainer::recomputeBounds(bool force_recompute)
{
    FonctorMultiAbstractBound fmab;
    for(AttributeMapType::iterator it = attributeMap_->begin(); it != attributeMap_->end(); it++)
        if(force_recompute || it->second.Dirty())
        {
            fmab.AddAttribute(it->first, it->second.decalage, it->second.dataType());
        }
    fmab = std::for_each (begin(), end(), fmab);
    int i=0;
    for(AttributeMapType::iterator it = attributeMap_->begin(); it != attributeMap_->end(); it++)
        if(force_recompute || it->second.Dirty())
        {
            double min, max;
            fmab.mvp_attrib[i++]->Get(min, max);
            it->second.min(min); it->second.max(max); // sets optional attribute as present => not dirty
        }
}

bool LidarDataContainer::getCenteringTransfo(double & x, double & y) const
{
    if(!m_xmlData->attributes().centeringTransfo().present())
        return false;
    x = m_xmlData->attributes().centeringTransfo().get().tx();
    y = m_xmlData->attributes().centeringTransfo().get().ty();
    return true;
}

LidarDataContainer::LidarDataContainer():
    attributeMap_(new AttributeMapType),
    m_xmlData(new cs::LidarDataType(cs::LidarDataType::AttributesType(0, cs::DataFormatType::binary)))
{
}

LidarDataContainer::LidarDataContainer(shared_ptr<cs::LidarDataType> xmlData):
    attributeMap_(new AttributeMapType),
    m_xmlData(new cs::LidarDataType(cs::LidarDataType::AttributesType(0, cs::DataFormatType::binary)))
{
    setMapsFromXML(xmlData);
}

LidarDataContainer::LidarDataContainer(std::string dataFileName):
    attributeMap_(new AttributeMapType),
    m_xmlData(new cs::LidarDataType(cs::LidarDataType::AttributesType(0, cs::DataFormatType::binary)))
{
    load(dataFileName);
}

LidarDataContainer::LidarDataContainer(const LidarDataContainer& rhs):
    attributeMap_(new AttributeMapType),
    m_xmlData(new cs::LidarDataType(cs::LidarDataType::AttributesType(0, cs::DataFormatType::binary)))
{
    copy(rhs);
}

LidarDataContainer& LidarDataContainer::operator=(const LidarDataContainer& rhs)
{
    if(this!=&rhs)
        copy(rhs);

    return *this;
}

void LidarDataContainer::setMapsFromXML(shared_ptr<cs::LidarDataType> xmlData)
{
    *m_xmlData = *xmlData;
    cs::LidarDataType::AttributesType::AttributeIterator itAttribute;
    for (itAttribute = m_xmlData->attributes().attribute().begin();
         itAttribute != m_xmlData->attributes().attribute().end(); ++itAttribute)
    {
        //TODO adapt if we do not want to load everything
        addAttribute(itAttribute);
    }
}

void LidarDataContainer::updateXMLStructure(
        const std::string& dataFileName,
        const cs::DataFormatType format,
        const LidarCenteringTransfo& transfo)
{
    // todo: update instead of regenerating
    // generate xml file
    cs::LidarDataType::AttributesType attributes(size(), format);

    // insert attributes in the xml (with their names and types)
    for(AttributeMapType::const_iterator it=attributeMap_->begin(); it!=attributeMap_->end(); ++it)
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

    *m_xmlData = cs::LidarDataType(attributes);
}

void LidarDataContainer::load(std::string dataFileName)
{
    LidarFile file(dataFileName);
    file.loadData(*this);
}

void LidarDataContainer::save(std::string dataFileName)
{
    LidarFile::save(*this, dataFileName);
}

void LidarDataContainer::copy(const LidarDataContainer& rhs, bool copy_data)
{
    *attributeMap_ = *rhs.attributeMap_;
    *m_xmlData = *rhs.m_xmlData;

    pointSize_ = rhs.pointSize_;
    if(copy_data) lidarData_ = rhs.lidarData_;
}

void LidarDataContainer::append(const LidarDataContainer& rhs)
{
    //	assert(*rhs.attributeMap_ == *attributeMap_);

    lidarData_.insert(lidarData_.end(), rhs.lidarData_.begin(), rhs.lidarData_.end());
}

//struct FunctorAddAttributeParameters
//{
//	explicit FunctorAddAttributeParameters(const std::string& name, AttributeMapType& attributeMap):
//		name_(name), attributeMap_(attributeMap){}
//	const std::string& name_;
//	AttributeMapType& attributeMap_;
//};
//
//template<EnumLidarDataType TAttributeType>
//struct FunctorAddAttribute
//{
//	typedef typename Lidar::LidarEnumTypeTraits<TAttributeType>::type AttributeType;
//
//	unsigned int operator()(const FunctorAddAttributeParameters& p)
//	{
//		AttributesInfo infos;
//		infos.type = LidarTypeTraits<AttributeType>::enum_type;
//
//		if(p.attributeMap_.empty())
//			infos.decalage = 0;
//		else
//			infos.decalage = p.attributeMap_.back().second.decalage + sizeof(AttributeType);
//
//		p.attributeMap_.push_back(AttributeMapType::value_type(p.name_, infos));
//
//		return sizeof(AttributeType);
//	}
//
//};


template<EnumLidarDataType T>
struct PointSizeFunctor
{
    unsigned int operator()()
    {
        return sizeof( typename LidarEnumTypeTraits<T>::type );
    }
};


void LidarDataContainer::updateAttributeContent(const unsigned int oldPointSize)
{
    lidarData_.resize(lidarData_.size()/oldPointSize*pointSize_);

    /// update data, shift attributes and initialise new attribute
    const LidarDataContainerType::const_iterator itBegin = lidarData_.begin();
    LidarDataContainerType::const_iterator itOldEndElement = itBegin + (size()-1)*oldPointSize;

    LidarDataContainerType::iterator itNewEndElement = lidarData_.begin() + (size()-1)*pointSize_;

    while(itOldEndElement > itBegin)
    {
        std::copy(itOldEndElement, itOldEndElement + oldPointSize, itNewEndElement);
        itOldEndElement -= oldPointSize;
        itNewEndElement -= pointSize_;
    }
}

bool LidarDataContainer::addAttributeHelper(cs::LidarDataType::AttributesType::AttributeIterator it)
{
    //si l'attribut existe dÃ©jÃ , on sort et retourne false
    if(attributeMap_->find(it->name()) != attributeMap_->end())
        return false;

    //calcul du nouveau dÃ©calage
    unsigned int sizeLastAttribute=0;

    if(!attributeMap_->empty())
        sizeLastAttribute = apply<PointSizeFunctor, unsigned int>(attributeMap_->back().second.dataType());

    //ajout de l'attribut dans les maps
    AttributesInfo infos(*it);
    if(!attributeMap_->empty())
        infos.decalage = sizeLastAttribute + attributeMap_->back().second.decalage;
    else infos.decalage = sizeLastAttribute;

    attributeMap_->push_back(AttributeMapType::value_type(it->name(), infos));

    //Mise Ã  jour de la pointSize :
    unsigned int sizeLastInsertedAttribute;
    sizeLastInsertedAttribute = apply<PointSizeFunctor, unsigned int>(it->dataType());
    pointSize_ = infos.decalage + sizeLastInsertedAttribute;

    return true;
}


bool LidarDataContainer::addAttribute(cs::LidarDataType::AttributesType::AttributeIterator it)
{
    const unsigned int oldPointSize = pointSize_;

    bool attributeAdded = addAttributeHelper(it);

    if(!attributeAdded)
        return false;

    if(!empty())
        updateAttributeContent(oldPointSize);

    return true;
}

bool LidarDataContainer::addAttribute(std::string name, EnumLidarDataType type)
{
    cs::AttributeType attrib_type(type, name);
    m_xmlData->attributes().attribute().push_back(attrib_type);
    cs::LidarDataType::AttributesType::AttributeIterator it = m_xmlData->attributes().attribute().end(); it--;
    return addAttribute(it);
}


void LidarDataContainer::addAttributeList(const std::vector<std::pair<std::string, EnumLidarDataType> > attributes)
{
    const unsigned int oldPointSize = pointSize_;


    std::vector<std::pair<std::string, EnumLidarDataType> >::const_iterator itb = attributes.begin();
    const std::vector<std::pair<std::string, EnumLidarDataType> >::const_iterator ite = attributes.end();

    for( ; itb != ite; ++itb)
    {
        cs::AttributeType attrib_type(itb->second, itb->first);
        m_xmlData->attributes().attribute().push_back(attrib_type);
        cs::LidarDataType::AttributesType::AttributeIterator it = m_xmlData->attributes().attribute().end(); it--;
        addAttributeHelper(it);
    }
    if(!empty())
        updateAttributeContent(oldPointSize);
}

struct predicate_true
{
    template<typename T>
    bool operator()(T)
    {
        return true;
    }
};


bool LidarDataContainer::delAttribute(const std::string& attributeName)
{
    if (!checkAttributeIsPresent(attributeName))
        return false;

    // shift and size of attribute to delete
    const unsigned int strideAttribute = getDecalage(attributeName);
    const unsigned int sizeAttribute = apply<PointSizeFunctor, unsigned int>(getAttributeType(attributeName));

    // update pointSize :
    const unsigned int oldPointSize = pointSize_;
    const unsigned int nbPoints = size();
    pointSize_ -= sizeAttribute;
    //cout << "strideAttribute=" << strideAttribute << endl;
    //cout << "sizeAttribute=" << sizeAttribute << endl;
    //cout << "oldPointSize=" << oldPointSize << endl;
    //cout << "pointSize_=" << pointSize_ << endl;
    //cout << "afterAttribSize_=" << oldPointSize - strideAttribute - sizeAttribute << endl;
    //cout << "nbPoints=" << nbPoints << endl;

    /// update data, shift attributes
    LidarDataContainerType::iterator itOldAttributePosition = lidarData_.begin() + strideAttribute;
    LidarDataContainerType::iterator itNewAttributePosition = itOldAttributePosition;
    const LidarDataContainerType::iterator ite = lidarData_.end() - oldPointSize;

    for(; itOldAttributePosition < ite; itOldAttributePosition += oldPointSize, itNewAttributePosition += pointSize_)
        std::copy(itOldAttributePosition + sizeAttribute, itOldAttributePosition + oldPointSize, itNewAttributePosition);

    // BV: attributes after the deleted attribute are not copied for the last echo in the for() loop above
    std::copy(itOldAttributePosition + sizeAttribute, itOldAttributePosition + oldPointSize - strideAttribute, itNewAttributePosition);

    // resize the container to its new size nbPoints*pointSize_ < old size = nbPoints*oldPointSize
    lidarData_.erase(lidarData_.begin() + nbPoints*pointSize_, lidarData_.end());

    // delete attribute in the maps, update shifts after deleted attribute
    AttributeMapType::iterator itSuccessor = attributeMap_->erase(attributeMap_->find(attributeName));

    for(; itSuccessor != attributeMap_->end(); ++itSuccessor)
        itSuccessor->second.decalage -= sizeAttribute;

    return true;
}

void LidarDataContainer::delAttributeList(const std::vector<std::string>& attributeNames)
{
    for(std::vector<std::string>::const_iterator it = attributeNames.begin(); it != attributeNames.end(); ++it)
        delAttribute(*it);
}


} //namespace Lidar
