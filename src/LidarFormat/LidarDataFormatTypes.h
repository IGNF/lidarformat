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
 * \file LidarDataFormatTypes.h
 * \brief
 * \author Adrien Chauve
 * \date 16 sept. 2008
 */

#ifndef LIDARDATAFORMATTYPES_H_
#define LIDARDATAFORMATTYPES_H_


#include <boost/cstdint.hpp>

#include "LidarFormat/models/format_me.hxx"

/**
* @namespace Namespace qui contient les codes de format Lidar.
*
* @author Adrien Chauve
*
*/

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
