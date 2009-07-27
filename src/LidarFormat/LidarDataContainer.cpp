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
 * \file LidarDataContainer.cpp
 * \brief
 * \author Adrien Chauve
 * \date 15 sept. 2008
 */



#include <stdexcept>
#include <fstream>
#include <algorithm>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <boost/noncopyable.hpp>

#include "LidarFormat/LidarDataFormatTypes.h"
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


LidarDataContainer::LidarDataContainer():
	attributeMap_(new AttributeMapType)
{

}

LidarDataContainer::LidarDataContainer(const LidarDataContainer& rhs):
	attributeMap_(new AttributeMapType)
{
	copy(rhs);
}

LidarDataContainer& LidarDataContainer::operator=(const LidarDataContainer& rhs)
{
	if(this!=&rhs)
		copy(rhs);

	return *this;
}

void LidarDataContainer::copy(const LidarDataContainer& rhs)
{
	*attributeMap_ = *rhs.attributeMap_;

	lidarData_ = rhs.lidarData_;
	pointSize_ = rhs.pointSize_;
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


bool LidarDataContainer::addAttribute(const std::string& attributeName, const EnumLidarDataType type)
{
	//si l'attribut existe déjà, on sort et retourne false
	if(attributeMap_->find(attributeName)!=attributeMap_->end())
		return false;


	//calcul du nouveau décalage
	unsigned int sizeLastAttribute=0;

	if(!attributeMap_->empty())
		sizeLastAttribute = apply<PointSizeFunctor, unsigned int>(attributeMap_->back().second.type);

	//ajout de l'attribut dans les maps et mise à jour des décalages qui suivent l'attribut inséré
	AttributesInfo infos;
	infos.type = type;
	if(!attributeMap_->empty())
	{
		infos.decalage = sizeLastAttribute + attributeMap_->back().second.decalage;
	}
	else
		infos.decalage = sizeLastAttribute;


	attributeMap_->push_back(AttributeMapType::value_type(attributeName, infos));


	//Mise à jour de la pointSize :
	unsigned int sizeLastInsertedAttribute;
	sizeLastInsertedAttribute = apply<PointSizeFunctor, unsigned int>(type);
	pointSize_ = infos.decalage + sizeLastInsertedAttribute;


	//TODO
	///MAJ des données ! décalage des attributs + initialisation du nouvel attribut

	return true;
}


} //namespace Lidar
