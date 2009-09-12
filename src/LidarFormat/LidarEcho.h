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


#ifndef LIDARECHO_H_
#define LIDARECHO_H_

#include <cassert>
#include <string>
#include <map>
#include <iosfwd>

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#include "LidarFormat/AttributesInfo.h"
#include "LidarFormat/LidarDataFormatTypes.h"

namespace Lidar
{

/**
* @brief Classe d'écho lidar.
*
* @author Adrien Chauve
*
*/

using boost::shared_ptr;


class LidarEcho
{
	public:
		explicit LidarEcho(const unsigned int size, const char *data, const shared_ptr<AttributeMapType> &attributeMap):
			echoPtr_(new char[size]), size_(size), attributeMap_(attributeMap)
		{
			update(data);
		}

		LidarEcho(const LidarEcho &rhs)
		{
			copy(rhs);
		}

//		LidarEcho(const LidarEchoRef& echo);

		LidarEcho &operator=(const LidarEcho &rhs)
		{
			copy(rhs);
			return *this;
		}

		inline friend bool operator== (const LidarEcho &lhs, const LidarEcho &rhs)
		{
			assert(lhs.size_ == rhs.size_);
			return memcmp(lhs.echoPtr_.get(), rhs.echoPtr_.get(), lhs.size_)==0;
		}

		template<typename TAttributeType>
		const TAttributeType value(const std::string &attributeName) const
		{
			char *ptr = echoPtr_.get() + getDecalage(attributeName);
			return *reinterpret_cast<TAttributeType*>(ptr);
		}

		template<typename TAttributeType>
		TAttributeType& value(const std::string &attributeName)
		{
			char *ptr = echoPtr_.get() + getDecalage(attributeName);
			return *reinterpret_cast<TAttributeType*>(ptr);
		}

		template<typename TAttributeType>
		const TAttributeType value(const unsigned int decalage) const
		{
			char *ptr = echoPtr_.get() + decalage;
			return *reinterpret_cast<TAttributeType*>(ptr);
		}

		template<typename TAttributeType>
		TAttributeType& value(const unsigned int decalage)
		{
			char *ptr = echoPtr_.get() + decalage;
			return *reinterpret_cast<TAttributeType*>(ptr);
		}



		Lidar::EnumLidarDataType getAttributeType(const std::string &attributeName) const
		{
			return attributeMap_->find(attributeName)->second.type;
		}

		unsigned int size() const
		{
			return size_;
		}

		unsigned int getDecalage(const std::string &attributeName) const
		{
			return attributeMap_->find(attributeName)->second.decalage;
		}


		friend std::istream& operator>>(std::istream& os, LidarEcho& echo);
		friend std::ostream& operator<<(std::ostream& os, const LidarEcho& echo);

		///Attention !!
		const char* getRawData() const
		{
			return echoPtr_.get();
		}
	protected:
		boost::shared_array<char> echoPtr_; //donnees d'un echo
		unsigned int size_; //taille d'un echo

		shared_ptr<AttributeMapType> attributeMap_;

		void update(const char * data)
		{
			memcpy(echoPtr_.get(), data, size_);
		}

		void copy(const LidarEcho& rhs)
		{
			size_ = rhs.size_;
			echoPtr_ = boost::shared_array<char>(new char[size_]);
			update(rhs.echoPtr_.get());
			attributeMap_ = rhs.attributeMap_;
		}



};


} //namespace Lidar

#endif /* LIDARECHO_H_ */
