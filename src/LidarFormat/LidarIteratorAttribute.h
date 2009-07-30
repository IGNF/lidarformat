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
 * \file LidarIteratorAttribute.h
 * \brief
 * \author Adrien Chauve
 * \date 9 oct. 2008
 */

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
