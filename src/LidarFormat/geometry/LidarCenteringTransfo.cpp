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

