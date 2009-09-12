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


#include <iostream>

#include "LidarEcho.h"

#include "apply.h"

namespace Lidar
{


template<Lidar::EnumLidarDataType T>
struct PrintFunctor
{
	void operator()(std::ostream &os, const LidarEcho &echo, const std::string &name)
	{
		os << echo.value<typename Lidar::LidarEnumTypeTraits<T>::type>(name) << "\t" ;
	}
};

template<>
struct PrintFunctor<LidarDataType::int8>
{
	void operator()(std::ostream &os, const LidarEcho &echo, const std::string &name)
	{
		os << (int) echo.value<int8>(name) << "\t" ;
	}
};

template<>
struct PrintFunctor<LidarDataType::uint8>
{
	void operator()(std::ostream &os, const LidarEcho &echo, const std::string &name)
	{
		os << (unsigned int) echo.value<uint8>(name) << "\t" ;
	}
};

std::ostream& operator<<( std::ostream& os, const LidarEcho& echo )
{

	for(AttributeMapType::const_iterator it = echo.attributeMap_->begin(); it != echo.attributeMap_->end(); ++it)
	{
		EnumLidarDataType type = it->second.type;
		apply<PrintFunctor, void, std::ostream &, const LidarEcho &, const std::string &>(type, os, echo, it->first);

	}

	return os;
}


template<EnumLidarDataType T>
struct PointSizeFunctor
{
	unsigned int operator()()
	{
		return sizeof( typename LidarEnumTypeTraits<T>::type );
	}
};



} //namespace Lidar



