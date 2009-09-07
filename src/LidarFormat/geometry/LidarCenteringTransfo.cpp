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
 * \file LidarCenteringTransfo.cpp
 * \brief
 * \author Adrien Chauve
 * \date 5 févr. 2009
 */

#include <cmath>


#include "LidarFormat/extern/matis/tpoint2d.h"
#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/apply.h"
#include "LidarFormat/tools/Orientation2D.h"

#include "LidarCenteringTransfo.h"

namespace Lidar
{

LidarCenteringTransfo::LidarCenteringTransfo():
	m_x(0), m_y(0)
{

}

void LidarCenteringTransfo::setTransfo(const double x, const double y)
{
	m_x = x;
	m_y = y;
}



void LidarCenteringTransfo::applyTransfoOrientation(Orientation2D& ori) const
{
	ori.OriginX(ori.OriginX() + m_x);
	ori.OriginY(ori.OriginY() + m_y);
}

void LidarCenteringTransfo::applyTransfoInverseOrientation(Orientation2D& ori) const
{
	ori.OriginX(ori.OriginX() - m_x);
	ori.OriginY(ori.OriginY() - m_y);
}


//struct FunctorCenterParameters
//{
//	explicit FunctorCenterParameters(const LidarDataContainer& lidarContainer, const std::string& name): lidarContainer_(lidarContainer), name_(name){}
//	const LidarDataContainer& lidarContainer_;
//	const std::string& name_;
//};

//template<EnumLidarDataType TAttributeType>
//struct FunctorCenter
//{
//	typedef typename LidarEnumTypeTraits<TAttributeType>::type AttributeType;
//
//	TPoint2D<double> operator()(LidarDataContainer& lidarContainer)
//	{
//
//		LidarIteratorXYZ<AttributeType> itb = lidarContainer.beginXYZ<AttributeType>();
//		const LidarIteratorXYZ<AttributeType> ite = lidarContainer.endXYZ<AttributeType>();
//
//		TPoint2D<double> center(0,0);
//
//		for(;itb!=ite; ++itb)
//		{
//			center.x += itb.x();
//			center.y += itb.y();
//		}
//
//		center /= lidarContainer.size();
//
//
//		itb = lidarContainer.beginXYZ<AttributeType>();
//		for(;itb!=ite; ++itb)
//		{
//			itb.x() -= center.x;
//			itb.y() -= center.y;
//		}
//
//		return center;
//	}
//
//};


template<EnumLidarDataType T>
struct ReadValueFunctor
{
	typedef typename LidarEnumTypeTraits<T>::type AttributeType;

	void operator()(LidarEcho &echo, const unsigned int decalage, LidarEcho &echoInitial, const unsigned int decalageInitial)
	{
		echo.value<AttributeType>(decalage) = echoInitial.value<AttributeType>(decalageInitial);
	}
};


template<EnumLidarDataType TAttributeType>
struct FunctorCenter
{
	typedef typename LidarEnumTypeTraits<TAttributeType>::type AttributeType;

	void operator()(const LidarDataContainer& lidarContainer, shared_ptr<LidarDataContainer>& centeredContainer, LidarCenteringTransfo& transfo, const char* const x, const char* const y, const char* const z)
	{
		LidarEcho echo = centeredContainer->createEcho();

		LidarConstIteratorEcho itb = lidarContainer.begin();
		const LidarConstIteratorEcho ite = lidarContainer.end();

		const unsigned int decalageX = centeredContainer->getDecalage(x);
		const unsigned int decalageY = centeredContainer->getDecalage(y);
		const unsigned int decalageZ = centeredContainer->getDecalage(z);

		const unsigned int decalageX_initial = lidarContainer.getDecalage(x);
		const unsigned int decalageY_initial = lidarContainer.getDecalage(y);
		const unsigned int decalageZ_initial = lidarContainer.getDecalage(z);


		//si la transfo vaut 0, elle est calculée automatiquement
		if(transfo.x()==0 && transfo.y()==0)
		{
			const float quotient = 1000.;
			LidarEcho echoInitial(*itb);
			double x = std::floor(echoInitial.value<AttributeType>(decalageX_initial)/quotient)*quotient;
			double y = std::floor(echoInitial.value<AttributeType>(decalageY_initial)/quotient)*quotient;
			transfo.setTransfo(x,y);
		}


		for(;itb!=ite; ++itb)
		{
			LidarEcho echoInitial(*itb);

			echo.value<float>(decalageX) = (float)( echoInitial.value<AttributeType>(decalageX_initial) - transfo.x() );
			echo.value<float>(decalageY) = (float)( echoInitial.value<AttributeType>(decalageY_initial) - transfo.y() );
			echo.value<float>(decalageZ) = (float)echoInitial.value<AttributeType>(decalageZ_initial);

			AttributeMapType::const_iterator itbAttribute = centeredContainer->getAttributeMap().begin();
			const AttributeMapType::const_iterator iteAttribute = centeredContainer->getAttributeMap().end();

			AttributeMapType::const_iterator itbAttributeInitial = lidarContainer.getAttributeMap().begin();

			for(; itbAttribute != iteAttribute; ++itbAttribute, ++itbAttributeInitial)
			{
				if(itbAttribute->first!=x && itbAttribute->first!=y && itbAttribute->first!=z)
				{
					apply<ReadValueFunctor, void, LidarEcho&, const unsigned int, LidarEcho&, const unsigned int>(itbAttribute->second.type, echo, itbAttribute->second.decalage, echoInitial, itbAttributeInitial->second.decalage);
				}
			}


			centeredContainer->push_back(echo);
		}

	}

};




//void PointCloud::minmax(const std::string &attributeName, double& mini, double &maxi) const
//{
//	FunctorMinMaxParameters p(*m_lidarContainer, attributeName);
//	std::pair<double, double> minmax = Apply<FunctorMinMax, FunctorMinMaxParameters, std::pair<double, double> >()(m_lidarContainer->getAttributeType(attributeName), p);
//	mini=minmax.first;
//	maxi=minmax.second;
//}

shared_ptr<LidarDataContainer> LidarCenteringTransfo::centerLidarDataContainer(const LidarDataContainer& lidarContainer, const char* const x, const char* const y, const char* const z)
{
	//Creation du nouveau container et ajout des attributs
	shared_ptr<LidarDataContainer> centeredContainer(new LidarDataContainer);
	const AttributeMapType& attributeMap = lidarContainer.getAttributeMap();

	for(AttributeMapType::const_iterator it = attributeMap.begin(); it != attributeMap.end(); ++it)
	{
		EnumLidarDataType type;
		if(it->first == x || it->first == y || it->first == z)
			type = LidarDataType::float32;
		else
			type = it->second.type;

		centeredContainer->addAttribute(it->first, type);
	}

	centeredContainer->reserve(lidarContainer.size());

	apply<FunctorCenter, void, const LidarDataContainer&, shared_ptr<LidarDataContainer>&, LidarCenteringTransfo&, const char* const, const char* const, const char* const>(lidarContainer.getAttributeType(x), lidarContainer, centeredContainer, *this, x, y, z);

	return centeredContainer;
}

} //namespace Lidar

