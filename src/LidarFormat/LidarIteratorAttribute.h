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


#ifndef LIDARITERATORATTRIBUTE_H_
#define LIDARITERATORATTRIBUTE_H_

#include <string>
#include <cassert>
#include <iterator>

namespace Lidar
{

namespace detail
{
	template<typename T>
	struct _LidarIteratorAttributeBase : public std::iterator<std::random_access_iterator_tag, T>
	{
		_LidarIteratorAttributeBase(char *dataPtr, const std::size_t increment):
			m_dataPtr(dataPtr), m_increment(increment)
		{
		}

		_LidarIteratorAttributeBase():
			m_dataPtr(0), m_increment(0)
		{
		}

		typedef _LidarIteratorAttributeBase Self;

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


		protected:
			void incremente()
			{
				m_dataPtr += m_increment;
			}

			void incremente(const typename std::iterator<std::random_access_iterator_tag, T>::difference_type i)
			{
				m_dataPtr += m_increment * i;
			}

			void decremente()
			{
				m_dataPtr -= m_increment;
			}

			char *m_dataPtr;
			std::size_t m_increment;
	};
}


template<typename T>
class LidarIteratorAttribute : public detail::_LidarIteratorAttributeBase<T>
{
	public:
		LidarIteratorAttribute(char *dataPtr, const unsigned int increment): detail::_LidarIteratorAttributeBase<T>(dataPtr, increment) {}

		LidarIteratorAttribute(){}

		typedef LidarIteratorAttribute<T> Self;
		typedef detail::_LidarIteratorAttributeBase<T> Super;

		typedef typename Super::reference reference;
		typedef typename Super::pointer pointer;
		typedef typename Super::difference_type difference_type;

		template<class U>
		friend class LidarConstIteratorAttribute;

	    reference operator*() const
		{
			return *reinterpret_cast<T*>(Super::m_dataPtr);
		}

	    pointer operator->() const
		{
			return reinterpret_cast<T*>(Super::m_dataPtr);
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
			Super::incremente();
			return *this;
		}

		Self& operator--()
		{
			Super::decremente();
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

		inline friend const Self operator+(const Self &lhs, const std::size_t index)
		{
			return Self(lhs) += index;
		}

		inline friend const Self operator-(const Self &lhs, const std::size_t index)
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
};


template<typename T>
class LidarConstIteratorAttribute : public detail::_LidarIteratorAttributeBase<T>
{
	public:
		LidarConstIteratorAttribute(char *dataPtr, const unsigned int increment): detail::_LidarIteratorAttributeBase<T>(dataPtr, increment) {}

		LidarConstIteratorAttribute(){}

		LidarConstIteratorAttribute(const LidarIteratorAttribute<T>& rhs):
			detail::_LidarIteratorAttributeBase<T>(rhs.m_dataPtr, rhs.m_increment) {}

		typedef LidarConstIteratorAttribute<T> Self;
		typedef detail::_LidarIteratorAttributeBase<T> Super;

		typedef typename Super::reference reference;
		typedef typename Super::pointer pointer;
		typedef typename Super::difference_type difference_type;


	    reference operator*() const
		{
			return *reinterpret_cast<T*>(Super::m_dataPtr);
		}

		pointer operator->() const
		{
			return reinterpret_cast<T*>(Super::m_dataPtr);
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
			Super::incremente();
			return *this;
		}

		Self& operator--()
		{
			Super::decremente();
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

		inline friend const Self operator+(const Self &lhs, const std::size_t index)
		{
			return Self(lhs) += index;
		}

		inline friend const Self operator-(const Self &lhs, const std::size_t index)
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
};

} //namespace Lidar


#endif /* LIDARITERATORATTRIBUTE_H_ */
