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


#ifndef LIDARITERATORECHO_H_
#define LIDARITERATORECHO_H_

#include <cassert>
#include <iterator>

#include <boost/shared_ptr.hpp>

#include "LidarFormat/LidarEcho.h"


namespace Lidar
{

using boost::shared_ptr;

namespace detail
{
	struct _LidarEchoProxy
	{
		_LidarEchoProxy(char *dataPtr, const std::size_t increment, const shared_ptr<AttributeMapType>& attributeMap):
			m_dataPtr(dataPtr), m_increment(increment), m_attributeMap(attributeMap)
		{
		}

		_LidarEchoProxy():
			m_dataPtr(0), m_increment(0), m_attributeMap(new AttributeMapType)
		{
		}

		_LidarEchoProxy& operator=(const _LidarEchoProxy& rhs)
		{
			memcpy(m_dataPtr, rhs.m_dataPtr, m_increment);
			return *this;
		}

		_LidarEchoProxy& operator=(const LidarEcho& rhs)
		{
			memcpy(m_dataPtr, rhs.getRawData(), m_increment);
			return *this;
		}

	    bool
	    operator==(const _LidarEchoProxy& rhs) const
	    {
	    	return LidarEcho(*this) == LidarEcho(rhs);
	    }

		operator LidarEcho() const
		{
			return LidarEcho(m_increment, m_dataPtr, m_attributeMap);
		}

		private:
			char *m_dataPtr;
			std::size_t m_increment;
			shared_ptr<AttributeMapType> m_attributeMap; //infos sur les attributs
	};

	struct _LidarIteratorEchoBase : public std::iterator<std::random_access_iterator_tag, LidarEcho>
	{
		_LidarIteratorEchoBase(char *dataPtr, const std::size_t increment, const shared_ptr<AttributeMapType>& attributeMap):
			m_dataPtr(dataPtr), m_increment(increment), m_attributeMap(attributeMap)
		{
		}

		_LidarIteratorEchoBase():
			m_dataPtr(0), m_increment(0), m_attributeMap(new AttributeMapType)
		{
		}

		typedef _LidarIteratorEchoBase Self;

	    bool
	    operator==(const Self& rhs) const
	    {
	    	return m_dataPtr == rhs.m_dataPtr && m_increment == rhs.m_increment;
	    }

	    bool
	    operator<(const Self& rhs) const
	    {
	    	return m_dataPtr < rhs.m_dataPtr;
	    }

	    bool
	    operator!=(const Self& rhs) const
	    { return !(*this == rhs); }

	    bool
	    operator>(const Self& rhs) const
	    { return rhs < *this; }

	    bool
	    operator<=(const Self& rhs) const
	    { return !(rhs < *this); }

	    bool
	    operator>=(const Self& rhs) const
	    { return !(*this < rhs); }


		unsigned int getDecalage(const std::string &attributeName) const
		{
			//TODO ajouter assert presence de l'attribut
			return m_attributeMap->find(attributeName)->second.decalage;
		}


		protected:
			void incremente()
			{
				m_dataPtr += m_increment;
			}

			void incremente(const difference_type i)
			{
				m_dataPtr += m_increment * i;
			}

			void decremente()
			{
				m_dataPtr -= m_increment;
			}

			char *m_dataPtr;
			std::size_t m_increment;
			shared_ptr<AttributeMapType> m_attributeMap; //infos sur les attributs
	};
}


struct LidarIteratorEcho : public detail::_LidarIteratorEchoBase
{

		typedef LidarIteratorEcho Self;

	    typedef detail::_LidarEchoProxy  reference;
	    typedef shared_ptr<detail::_LidarEchoProxy> pointer;

	    LidarIteratorEcho(){}

		LidarIteratorEcho(char *dataPtr, const std::size_t increment, const shared_ptr<AttributeMapType>& attributeMap):
			detail::_LidarIteratorEchoBase(dataPtr, increment, attributeMap) {}



		reference operator*() const
		{
			return reference(m_dataPtr, m_increment, m_attributeMap);
		}

		pointer operator->() const
		{
			return pointer(new reference(**this));
		}

		const Self operator--(int)
		{
			Self newSelf(*this);
			--(*this);
			return newSelf;
		}

		const Self operator++(int)
		{
			Self newSelf(*this);
			++(*this);
			return newSelf;
		}


		Self& operator++()
		{
			incremente();
			return *this;
		}

		Self& operator--()
		{
			decremente();
			return *this;
		}

		Self& operator+= (const difference_type i)
		{
			incremente(i);
			return *this;
		}

		Self& operator-= (const difference_type i)
		{
			incremente(-i);
			return *this;
		}

	    reference operator[](const difference_type i) const
	    {
	    	return *(*this + i);
	    }

		inline friend const Self operator+(const Self &lhs, const difference_type index)
		{
			return Self(lhs) += index;
		}

		inline friend const Self operator-(const Self &lhs, const difference_type index)
		{
			return Self(lhs) -= index;
		}

		inline friend const difference_type operator- (const Self &lhs, const Self &rhs)
		{
			assert(lhs.m_increment == rhs.m_increment);
			return static_cast<difference_type>( (lhs.m_dataPtr - rhs.m_dataPtr) / lhs.m_increment);
		}

		inline friend const Self operator+(const difference_type n, const Self& rhs)
		{
			return rhs + n;
		}


//		const char* getData() const
//		{
//			return m_dataPtr;
//		}

		///Fonctions reprises de LidarEcho, mais permettent d'optimiser les accès
		template<typename TAttributeType>
		TAttributeType& value(const std::string &attributeName) const
		{
			return *reinterpret_cast<TAttributeType*>(m_dataPtr + getDecalage(attributeName));
		}


		template<typename TAttributeType>
		TAttributeType& value(const unsigned int decalage) const
		{
			return *reinterpret_cast<TAttributeType*>(m_dataPtr + decalage);
		}

		friend struct LidarConstIteratorEcho;
};




struct LidarConstIteratorEcho : public detail::_LidarIteratorEchoBase
{

		typedef LidarConstIteratorEcho Self;

	    typedef const LidarEcho  reference;
	    typedef shared_ptr<const LidarEcho> pointer;

	    LidarConstIteratorEcho(){}

	    LidarConstIteratorEcho(char *dataPtr, const std::size_t increment, const shared_ptr<AttributeMapType>& attributeMap):
			detail::_LidarIteratorEchoBase(dataPtr, increment, attributeMap) {}

	    LidarConstIteratorEcho(const LidarIteratorEcho& rhs):
			detail::_LidarIteratorEchoBase(rhs.m_dataPtr, rhs.m_increment, rhs.m_attributeMap) {}


	    reference operator*() const
		{
			return reference(m_increment, m_dataPtr, m_attributeMap);
		}

		pointer operator->() const
		{
			return pointer(new reference(**this));
		}

		const Self operator--(int)
		{
			Self newSelf(*this);
			--(*this);
			return newSelf;
		}

		const Self operator++(int)
		{
			Self newSelf(*this);
			++(*this);
			return newSelf;
		}


		Self& operator++()
		{
			incremente();
			return *this;
		}

		Self& operator--()
		{
			decremente();
			return *this;
		}

		Self& operator+= (const difference_type i)
		{
			incremente(i);
			return *this;
		}

		Self& operator-= (const difference_type i)
		{
			incremente(-i);
			return *this;
		}

	    reference operator[](const difference_type i) const
	    {
	    	return *(*this + i);
	    }

		inline friend const Self operator+(const Self &lhs, const difference_type index)
		{
			return Self(lhs) += index;
		}

		inline friend const Self operator-(const Self &lhs, const difference_type index)
		{
			return Self(lhs) -= index;
		}

		inline friend const difference_type operator- (const Self &lhs, const Self &rhs)
		{
			assert(lhs.m_increment == rhs.m_increment);
			return static_cast<difference_type>( (lhs.m_dataPtr - rhs.m_dataPtr) / lhs.m_increment);
		}

		inline friend const Self operator+(const difference_type n, const Self& rhs)
		{
			return rhs + n;
		}

		template<typename TAttributeType>
		const TAttributeType value(const std::string &attributeName) const
		{
			return *reinterpret_cast<TAttributeType*>(m_dataPtr + getDecalage(attributeName));
		}

		template<typename TAttributeType>
		const TAttributeType value(const unsigned int decalage) const
		{
			return *reinterpret_cast<TAttributeType*>(m_dataPtr + decalage);
		}



};


} //namespace Lidar


void swap(Lidar::detail::_LidarEchoProxy p1, Lidar::detail::_LidarEchoProxy p2);


#endif /* LIDARITERATORECHO_H_ */
