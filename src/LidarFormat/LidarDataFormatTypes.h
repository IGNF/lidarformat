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


#ifndef LIDARDATAFORMATTYPES_H_
#define LIDARDATAFORMATTYPES_H_


#include <boost/cstdint.hpp>

#include "LidarFormat/models/format_me.hxx"


namespace Lidar
{

typedef boost::int8_t int8;
typedef boost::int16_t int16;
typedef boost::int32_t int32;
typedef boost::uint8_t uint8;
typedef boost::uint16_t uint16;
typedef boost::uint32_t uint32;
typedef boost::int64_t int64;
typedef boost::uint64_t uint64;
typedef float float32;
typedef double float64;


typedef cs::AttributeDataType LidarDataType;
typedef LidarDataType::Value EnumLidarDataType;



template <typename ScalarType>
struct LidarTypeTraits;

#ifdef LIDAR_TYPE_TRAITS
#  error
#endif

#define LIDAR_TYPE_TRAITS(TYPE, ENUM, NAME, OLD_NAME)\
template <>\
struct LidarTypeTraits<TYPE>\
{\
  typedef TYPE type;\
  static const EnumLidarDataType enum_type = ENUM;\
  static const char* name() { return NAME; }\
  static const char* old_name() { return OLD_NAME; }\
};

LIDAR_TYPE_TRAITS(int8, LidarDataType::int8, "int8", "char")
LIDAR_TYPE_TRAITS(int16, LidarDataType::int16, "int16", "short")
LIDAR_TYPE_TRAITS(int32, LidarDataType::int32, "int32", "int")
LIDAR_TYPE_TRAITS(int64, LidarDataType::int64, "int64", "long")
LIDAR_TYPE_TRAITS(uint8, LidarDataType::uint8, "uint8", "uchar")
LIDAR_TYPE_TRAITS(uint16, LidarDataType::uint16, "uint16", "ushort")
LIDAR_TYPE_TRAITS(uint32, LidarDataType::uint32, "uint32", "uint")
LIDAR_TYPE_TRAITS(uint64, LidarDataType::uint64, "uint64", "ulong")
LIDAR_TYPE_TRAITS(float32, LidarDataType::float32, "float32", "float")
LIDAR_TYPE_TRAITS(float64, LidarDataType::float64, "float64", "double")

#undef LIDAR_TYPE_TRAITS


/////////////////ENUM

template <EnumLidarDataType>
struct LidarEnumTypeTraits;

#ifdef LIDAR_ENUM_TYPE_TRAITS
#  error
#endif

#define LIDAR_ENUM_TYPE_TRAITS(TYPE, ENUM, NAME, OLD_NAME)\
template <>\
struct LidarEnumTypeTraits<ENUM>\
{\
  typedef TYPE type;\
  static const cs::AttributeDataType::Value enum_type = ENUM;\
  static const char* name() { return NAME; }\
  static const char* old_name() { return OLD_NAME; }\
};

LIDAR_ENUM_TYPE_TRAITS(int8, LidarDataType::int8, "int8", "char")
LIDAR_ENUM_TYPE_TRAITS(int16, LidarDataType::int16, "int16", "short")
LIDAR_ENUM_TYPE_TRAITS(int32, LidarDataType::int32, "int32", "int")
LIDAR_ENUM_TYPE_TRAITS(int64, LidarDataType::int64, "int64", "long")
LIDAR_ENUM_TYPE_TRAITS(uint8, LidarDataType::uint8, "uint8", "uchar")
LIDAR_ENUM_TYPE_TRAITS(uint16, LidarDataType::uint16, "uint16", "ushort")
LIDAR_ENUM_TYPE_TRAITS(uint32, LidarDataType::uint32, "uint32", "uint")
LIDAR_ENUM_TYPE_TRAITS(uint64, LidarDataType::uint64, "uint64", "ulong")
LIDAR_ENUM_TYPE_TRAITS(float32, LidarDataType::float32, "float32", "float")
LIDAR_ENUM_TYPE_TRAITS(float64, LidarDataType::float64, "float64", "double")

#undef LIDAR_ENUM_TYPE_TRAITS


}

#endif /* LIDARDATAFORMATTYPES_H_ */
