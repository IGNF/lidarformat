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
	
Contributors:

	Nicolas David, Olivier Tournaire



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
 * \file UnsortedAssocMap.h
 * \brief
 * \author Adrien Chauve
 * \date 13 janv. 2009
 */

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
