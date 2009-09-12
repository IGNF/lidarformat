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


#ifndef UNSORTEDASSOCMAP_H_
#define UNSORTEDASSOCMAP_H_

#include <vector>
#include <utility>


template<typename T1, typename T2>
class UnsortedAssocMap
{
	public:

		typedef T1 first_type;
		typedef T2 second_type;

		typedef std::pair<first_type, second_type> value_type;
		typedef std::vector<value_type> container_type;

		typedef typename container_type::const_iterator const_iterator;
		typedef typename container_type::iterator iterator;

		typedef UnsortedAssocMap<T1, T2> Self;


		const_iterator find(const first_type& value) const
		{
			return _find(value);
		}

		iterator find(const first_type& value)
		{
			return begin() + (_find(value) - begin());
		}

		void push_back(const value_type& value)
		{
			container_.push_back(value);
		}

		const value_type& back() const
		{
			return container_.back();
		}

		value_type& back()
		{
			return container_.back();
		}

		void reserve(const std::size_t s)
		{
			container_.reserve(s);
		}

		std::size_t size() const
		{
			return container_.size();
		}

		bool empty() const
		{
			return container_.empty();
		}

		iterator begin()
		{
			return container_.begin();
		}

		const_iterator begin() const
		{
			return container_.begin();
		}

		iterator end()
		{
			return container_.end();
		}

		const_iterator end() const
		{
			return container_.end();
		}

	private:
		container_type container_;


		const_iterator _find(const first_type& value) const
		{
			typename container_type::const_iterator it=container_.begin();

			while(it!=container_.end())
			{
				if(it->first==value)
					return it;
				++it;
			}

			return it;
		}
};

#endif /* UNSORTEDASSOCMAP_H_ */
