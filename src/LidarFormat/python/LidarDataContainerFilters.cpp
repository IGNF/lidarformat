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

#include <boost/python.hpp>

#include <algorithm>


#include "LidarFormat/LidarEcho.h"



#include "LidarDataContainerFilters.h"

namespace python_bindings
{


///algo copy_if, non implémenté dans la STL
///version tirée de "Effective STL"
template< typename InputIterator, typename OutputIterator, typename Predicate>
OutputIterator copy_if( InputIterator begin, InputIterator end, OutputIterator destBegin, Predicate p)
{
	while(begin!=end)
	{
		if(p(*begin)) *destBegin++ = *begin;
		++begin;
	}
	return destBegin;
}



///double for_each
template< typename InputIterator1, typename InputIterator2, typename Function>
Function for_each_double( InputIterator1 begin, InputIterator1 end, InputIterator2 begin2, Function f)
{
	while(begin!=end)
	{
		f(*begin++, *begin2++);
	}
	return f;
}

struct PythonCallable
{

	PythonCallable(PyObject* callable): m_callable(callable){}

	bool operator()(const LidarEcho& echo)
	{
		return boost::python::call<bool>(m_callable, echo);
	}

	PyObject* m_callable;
};


void print_if_echo(LidarDataContainer& container, PyObject* callable_predicate)
{
	using namespace std;
	cout.precision(12);
	ostream_iterator<LidarEcho> it_outputEcho( cout, "\n" );
	copy_if(container.begin(), container.end(), it_outputEcho, PythonCallable(callable_predicate));
}


void for_each_xyz(LidarDataContainer& container, PyObject* callable_modifier)
{
//	std::for_each(container.beginXYZ<double>(), container.endXYZ<double>(), PythonCallable(callable_modifier));
}





}
