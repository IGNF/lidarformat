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

#include <iostream>
#include <sstream>

#include <boost/noncopyable.hpp>
#include <boost/python.hpp>
//#include <boost/bind.hpp>

using namespace boost::python;


#include <boost/gil/extension/io/tiff_io.hpp>

namespace gil = boost::gil;

#include "LidarDataContainerFilters.h"
#include "LidarFormat/geometry/LidarCenteringTransfo.h"
#include "LidarFormat/LidarFile.h"
#include "LidarFormat/tools/Orientation2D.h"


using namespace Lidar;
using namespace python_bindings;


void saveSharedToBinary(const shared_ptr<LidarDataContainer>& container, const std::string& xmlFileName, const LidarCenteringTransfo& transfo)
{
	LidarFile::save(*container, xmlFileName, transfo);
}

void saveContainerToBinary(const LidarDataContainer& container, const std::string& xmlFileName, const LidarCenteringTransfo& transfo)
{
	LidarFile::save(container, xmlFileName, transfo);
}


std::string print_echo(const LidarEcho& e)
{
	std::ostringstream oss;
	oss << e;
	return oss.str();
}

std::string header(const LidarDataContainer& c)
{
	std::ostringstream oss;
	c.printHeader(oss);
	return oss.str();
}


BOOST_PYTHON_MODULE(libLidarFormat)
{


    class_<LidarFile>("LidarFile", init<const std::string &>())
        .def("get_meta_data", &LidarFile::getMetaData)
		.def("load_data", &LidarFile::loadData)
		.def("load_transfo", &LidarFile::loadTransfo)
	;

    def("save", saveSharedToBinary);
    def("save", saveContainerToBinary);



	typedef const double (LidarEcho::*ConstValueFuncType)(const std::string&) const;

	ConstValueFuncType constValueFunc = &LidarEcho::value<double>;


	class_<LidarEcho>("LidarEcho", no_init)
		.def("value_double", constValueFunc)
//		.def(str(self))
		.def("__str__", print_echo)
	;


	class_<LidarCenteringTransfo>("LidarCenteringTransfo")
		.def("set_transfo", static_cast<void (LidarCenteringTransfo::*)(const double, const double)>(&LidarCenteringTransfo::setTransfo))
		.def("center_lidar_container", &LidarCenteringTransfo::centerLidarDataContainer)
	;

//	boost::function<LidarIteratorAttribute<double>() > beginAttribute = boost::bind(static_cast< LidarIteratorAttribute<double> (LidarDataContainer::*)(const std::string&) >(&LidarDataContainer::beginAttribute<double>), "x", _1);

	class_<LidarDataContainer, boost::noncopyable>("LidarDataContainer")
		.def("__len__", &LidarDataContainer::size)
		.def("add_attributes", &LidarDataContainer::addAttributeList)
		.def("del_attribute", &LidarDataContainer::delAttribute)
		.def("__iter__", iterator<LidarDataContainer >())
		.def("header", header)
	;

	register_ptr_to_python< boost::shared_ptr<LidarDataContainer> >();


	def("print_if_echo", print_if_echo, "Classe qui gere les echos");






	class_<Orientation2D>("Ori")
		.def("read_from_image_file", &Orientation2D::ReadOriFromImageFile)
		.def("read_from_ori_file", &Orientation2D::ReadOriFromOriFile)
		.def("save", &Orientation2D::SaveOriToFile)
		.def("origin_x", static_cast<double (Orientation2D::*)() const>(&Orientation2D::OriginX))
		.def("origin_y", static_cast<double (Orientation2D::*)() const>(&Orientation2D::OriginY))
		.def("step", static_cast<double (Orientation2D::*)() const>(&Orientation2D::Step))
		.def("size_x", static_cast<unsigned int (Orientation2D::*)() const>(&Orientation2D::SizeX))
		.def("size_y", static_cast<unsigned int (Orientation2D::*)() const>(&Orientation2D::SizeY))
	;


}


