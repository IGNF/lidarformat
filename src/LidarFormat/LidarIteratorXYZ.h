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


#ifndef LIDARITERATORXYZ_H_
#define LIDARITERATORXYZ_H_

#include <string>
#include <cassert>
#include <iterator>


#include "extern/matis/tpoint3d.h"
#include "extern/matis/tpoint2d.h"



namespace Lidar
{

	/**
	 * ATTENTION : - cette classe n'est à utiliser que si le conteneur lidar a bien trois attributs appelés "x", "y" et "z" de MEME type T
	 */

namespace detail
{
	template<typename T>
	struct _XYZProxy
	{
		_XYZProxy(char *dataPtr, const std::size_t stride):
			m_dataPtr(dataPtr), m_stride(stride)
		{
		}

		_XYZProxy():
			m_dataPtr(0), m_stride(0)
		{
		}

		typedef TPoint3D<T> PointType;

		typedef _XYZProxy<T> Self;

		Self& operator=(const Self& rhs)
		{
			memcpy(m_dataPtr, rhs.m_dataPtr, m_stride);
			return *this;
		}

		Self& operator=(const PointType& rhs)
		{
			*reinterpret_cast<T*>(m_dataPtr) = rhs.x;
			*reinterpret_cast<T*>(m_dataPtr + m_stride) = rhs.y;
			*reinterpret_cast<T*>(m_dataPtr + 2*m_stride) = rhs.z;
			return *this;
		}

	    bool
	    operator==(const Self& rhs) const
	    {
	    	return PointType(*this) == PointType(rhs);
	    }

		operator PointType() const
		{
			return PointType(*reinterpret_cast<T*>(m_dataPtr), *reinterpret_cast<T*>(m_dataPtr + m_stride), *reinterpret_cast<T*>(m_dataPtr + 2*m_stride));
		}

		friend void swap(Self& p1, Self& p2)
		{
			PointType temp(p1);
			p1 = p2;
			p2 = temp;
		}

		private:
			char *m_dataPtr;
			std::size_t m_stride;
	};

	template<typename T>
	struct _LidarIteratorXYZBase : public std::iterator<std::random_access_iterator_tag, const TPoint3D<T> >
	{
		_LidarIteratorXYZBase(char *dataPtr, const std::size_t increment):
			m_dataPtr(dataPtr), m_increment(increment)
		{
		}

		_LidarIteratorXYZBase():
			m_dataPtr(0), m_increment(0)
		{
		}

		typedef _LidarIteratorXYZBase Self;

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
class LidarIteratorXYZ : public detail::_LidarIteratorXYZBase<T>
{
	public:
		LidarIteratorXYZ(char *dataPtr, const unsigned int increment): detail::_LidarIteratorXYZBase<T>(dataPtr, increment) {}

		typedef LidarIteratorXYZ<T> Self;
		typedef detail::_LidarIteratorXYZBase<T> Super;

	    typedef detail::_XYZProxy<T> reference;
	    typedef shared_ptr<detail::_XYZProxy<T> > pointer;
		typedef typename Super::difference_type difference_type;

		T& x() const { return *reinterpret_cast<T*>(Super::m_dataPtr); }
		T& y() const { return *reinterpret_cast<T*>(Super::m_dataPtr + sizeof(T)); }
		T& z() const { return *reinterpret_cast<T*>(Super::m_dataPtr + 2*sizeof(T)); }

		const TPoint2D<T> xy() const { return TPoint2D<T>(x(), y()); }

		template<class U>
		friend class LidarConstIteratorXYZ;

	    reference operator*() const
		{
			return reference(Super::m_dataPtr, sizeof(T));
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
class LidarConstIteratorXYZ : public detail::_LidarIteratorXYZBase<T>
{
	public:
		LidarConstIteratorXYZ(char *dataPtr, const unsigned int increment): detail::_LidarIteratorXYZBase<T>(dataPtr, increment) {}

		LidarConstIteratorXYZ(const LidarIteratorXYZ<T>& rhs):
			detail::_LidarIteratorXYZBase<T>(rhs.m_dataPtr, rhs.m_increment) {}


		typedef LidarConstIteratorXYZ<T> Self;
		typedef detail::_LidarIteratorXYZBase<T> Super;

		typedef const TPoint3D<T> reference;
		typedef const TPoint3D<T>* pointer;
		typedef typename Super::difference_type difference_type;

		const T x() const { return *reinterpret_cast<T*>(Super::m_dataPtr); }
		const T y() const { return *reinterpret_cast<T*>(Super::m_dataPtr + sizeof(T)); }
		const T z() const { return *reinterpret_cast<T*>(Super::m_dataPtr + 2*sizeof(T)); }

		const TPoint2D<T> xy() const { return TPoint2D<T>(x(), y()); }


	    reference operator*() const
		{
			return reference(x(), y(), z());
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

#endif /* LIDARITERATORXYZ_H_ */
