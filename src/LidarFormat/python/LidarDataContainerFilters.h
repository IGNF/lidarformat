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

#ifndef LIDARDATACONTAINERFILTERS_H_
#define LIDARDATACONTAINERFILTERS_H_

#include <boost/python/detail/wrap_python.hpp>

#include "LidarFormat/LidarDataContainer.h"

namespace python_bindings
{
	using namespace Lidar;


	void print_if_echo(LidarDataContainer& container, PyObject* callable_predicate);

	void for_each_echo(LidarDataContainer& container, PyObject* callable_modifier);

	void for_each_xyz(LidarDataContainer& container, PyObject* callable_modifier);

	void filter(LidarDataContainer& container, PyObject* callable_filter);



}

#endif /* LIDARDATACONTAINERFILTERS_H_ */
