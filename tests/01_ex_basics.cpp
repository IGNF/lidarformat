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
#include <boost/filesystem.hpp>

#include "config_data_test.h"

#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/LidarFile.h"

#ifdef _WINDOWS
#include "LidarFormat/file_formats/StaticRegisterFormats.cpp"
#endif


int main()
{
	using namespace Lidar;
	using namespace std;


	/* Call this function before loading any data from a file if the library is compiled as a .lib or .a (static) */
#ifdef _WINDOWS
	registerAllFileFormats();
#endif


	/**** Load data in a container ****/

	//data test filename (trunk/data/testAscii.xml)
	const string lidarFileName(string(PATH_LIDAR_TEST_DATA) + "/testAscii.xml");

	//create a LidarFile which reads the XML file and contains informations about the data
	//(size, number of attributes, types of attributes, path to the data file)
	LidarFile file(lidarFileName);

	//create an empty LidarDataContainer, the object to contain point clouds
	LidarDataContainer lidarContainer;

	//Load data from the file to the container
	file.loadData(lidarContainer);



	/**** Print data in the console ****/

	std::cout << "\n\nContainer content:\n";

	//Print the list of attributes
	lidarContainer.printHeader(cout);

	//Print the content of the container
	ostream_iterator<LidarEcho> echoOutputIterator( cout, "\n" );
	copy(lidarContainer.begin(), lidarContainer.end(), echoOutputIterator);



	/**** File formats ****/

	//Standard formats are ASCII and BINARY formats (for read/write operations)
	//Other formats (LAS, TerraBin, PlyArchi) are only supported for reading

	//save the container in binary format
	using namespace boost::filesystem;
	const std::string lidarFileNameBinary = path(lidarFileName).branch_path().string() + "/" + basename(lidarFileName) + "-binary.xml";
	LidarFile::save(lidarContainer, lidarFileNameBinary); //default format is binary

	//save the container in new ascii file
	const std::string lidarFileNameAsciiNew = path(lidarFileName).branch_path().string() + "/" + basename(lidarFileName) + "-new.xml";
	LidarFile::save(lidarContainer, lidarFileNameAsciiNew, cs::DataFormatType::ascii); //default format is binary



	/**** Basic operations on container ****/

	std::cout << "\n\nBasic operations on container:\n";

	//copy
	LidarDataContainer anotherLidarContainer(lidarContainer);

	std::cout << "size:" << anotherLidarContainer.size();

	anotherLidarContainer.clear();


	/**** Walk through container using iterators ****/
	{
		std::cout << "\n\nContainer content using iterators:\n";

		LidarDataContainer::const_iterator itb = lidarContainer.begin();
		const LidarDataContainer::const_iterator ite = lidarContainer.end();

		//Print content using iterators
		for(; itb != ite; ++itb)
			std::cout << *itb << "\n";

		//NB: if you use non-const iterators, you need to use:
		//	cout << LidarEcho(*itb) << "\n";
		//because in this case the operator* returns a proxy to a LidarEcho

	}


	/**** Change values using (random access) iterators ****/
	{
		//first, check that the points in the containers have x,y attributes (and the type of x and y is float64)
		if( !lidarContainer.checkAttributeIsPresentAndType("x", LidarDataType::float64)
				&& !lidarContainer.checkAttributeIsPresentAndType("y", LidarDataType::float64) )
		{
			std::cerr << "Points don't have x attribute or its type is not float64" << std::endl;
			return 1;
		}


		LidarDataContainer::iterator itb = lidarContainer.begin();
		const LidarDataContainer::iterator ite = lidarContainer.end();

		//first possibility using the name (string) of the attribute
		for(; itb != ite; ++itb)
			itb.value<float64>("x") = 5.;

		//second possibility, mre efficient: store the shift for one attribute
		const unsigned int shiftY = lidarContainer.getDecalage("y");
		for(; itb != ite; ++itb)
			itb.value<float64>(shiftY) = 3.;

	}


	/**** Create new container ****/
	{
		LidarDataContainer newContainer;

		//add attributes ...
		newContainer.addAttribute("x", LidarDataType::float64);
		newContainer.addAttribute("y", LidarDataType::float64);
		newContainer.addAttribute("z", LidarDataType::float64);

		//... then resize the container ...
		newContainer.resize(10);


		LidarDataContainer::iterator itb = newContainer.begin();
		const LidarDataContainer::iterator ite = newContainer.end();

		const unsigned int shiftX = lidarContainer.getDecalage("x");
		const unsigned int shiftY = lidarContainer.getDecalage("y");
		const unsigned int shiftZ = lidarContainer.getDecalage("z");

		//... and set values
		int i = 0;
		for(; itb != ite; ++itb, ++i)
		{
			itb.value<float64>(shiftX) = i;
			itb.value<float64>(shiftY) = 2.*i;
			itb.value<float64>(shiftZ) = 3.*i;
		}

	}


	/**** Other vector-like methods ****/

	//erase, empty, capacity, reserve, push_back
	//TODO


	return 0;
}
