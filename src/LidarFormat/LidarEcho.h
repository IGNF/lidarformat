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
 * \file LidarEcho.h
 * \brief
 * \author Adrien Chauve
 * \date 18 sept. 2008
 */

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
