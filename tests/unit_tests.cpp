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


#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE LidarFormatUnitTests
#include <boost/test/unit_test.hpp>

#include "config_data_test.h"

#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/LidarFile.h"

using namespace Lidar;
using namespace std;

#ifdef _WINDOWS
#include "LidarFormat/file_formats/StaticRegisterFormats.cpp"
#endif



BOOST_AUTO_TEST_SUITE(LidarDataContainerTests)


//fichier xml de test
const string lidarFileName(string(PATH_LIDAR_TEST_DATA) + "/testAscii.xml");

const double firstX = 919351.96, firstY = 1914105.38, firstZ = 1075.35;
const double lastX = 919360.56, lastY = 1914108.38, lastZ = 1079.2;



BOOST_AUTO_TEST_CASE( LidarIteratorEcho_tests )
{
	/* Call this function before loading any data from a file if the library is compiled as a .lib or .a (static) */
#ifdef _WINDOWS
	registerAllFileFormats();
#endif

	LidarFile file(lidarFileName);
	LidarDataContainer lidarContainer;
	file.loadData(lidarContainer);

	const unsigned int decalageX = lidarContainer.getDecalage("x");
	const unsigned int decalageY = lidarContainer.getDecalage("y");
	const unsigned int decalageZ = lidarContainer.getDecalage("z");


	const LidarDataContainer::const_iterator itBeginLidarContainer = lidarContainer.begin();

	BOOST_CHECK_EQUAL(lidarContainer.size(), 10);

	BOOST_CHECK_EQUAL(itBeginLidarContainer.value<double>(decalageX), firstX);
	BOOST_CHECK_EQUAL(itBeginLidarContainer.value<double>(decalageY), firstY);
	BOOST_CHECK_EQUAL(itBeginLidarContainer.value<double>(decalageZ), firstZ);

	const LidarDataContainer::const_iterator itBeforeEndLidarContainer = lidarContainer.end()-1;

	BOOST_CHECK_EQUAL(itBeforeEndLidarContainer.value<double>(decalageX), lastX);
	BOOST_CHECK_EQUAL(itBeforeEndLidarContainer.value<double>(decalageY), lastY);
	BOOST_CHECK_EQUAL(itBeforeEndLidarContainer.value<double>(decalageZ), lastZ);
}


BOOST_AUTO_TEST_CASE( LidarIteratorXYZ_tests )
{
	LidarFile file(lidarFileName);
	LidarDataContainer lidarContainer;
	file.loadData(lidarContainer);

//	const unsigned int decalageX = lidarContainer.getDecalage("x");
//	const unsigned int decalageY = lidarContainer.getDecalage("y");
//	const unsigned int decalageZ = lidarContainer.getDecalage("z");


	const LidarConstIteratorXYZ<double> itBeginLidarContainer = lidarContainer.beginXYZ<double>();

	BOOST_CHECK_EQUAL(*itBeginLidarContainer, TPoint3D<double>(firstX, firstY, firstZ));

	const LidarConstIteratorXYZ<double> itBeforeEndLidarContainer = lidarContainer.endXYZ<double>()-1;

	BOOST_CHECK_EQUAL(*itBeforeEndLidarContainer, TPoint3D<double>(lastX, lastY, lastZ));
}


BOOST_AUTO_TEST_CASE( LidarIteratorAttribute_tests )
{
	LidarFile file(lidarFileName);
	LidarDataContainer lidarContainer;
	file.loadData(lidarContainer);

//	const unsigned int decalageX = lidarContainer.getDecalage("x");
//	const unsigned int decalageY = lidarContainer.getDecalage("y");
//	const unsigned int decalageZ = lidarContainer.getDecalage("z");


	const LidarConstIteratorAttribute<double> itBeginLidarContainerX = lidarContainer.beginAttribute<double>("x");
	const LidarConstIteratorAttribute<double> itBeginLidarContainerY = lidarContainer.beginAttribute<double>("y");
	const LidarConstIteratorAttribute<double> itBeginLidarContainerZ = lidarContainer.beginAttribute<double>("z");

	BOOST_CHECK_EQUAL(*itBeginLidarContainerX, firstX);
	BOOST_CHECK_EQUAL(*itBeginLidarContainerY, firstY);
	BOOST_CHECK_EQUAL(*itBeginLidarContainerZ, firstZ);

	const LidarConstIteratorAttribute<double> itBeforeEndLidarContainerX = lidarContainer.endAttribute<double>("x")-1;
	const LidarConstIteratorAttribute<double> itBeforeEndLidarContainerY = lidarContainer.endAttribute<double>("y")-1;
	const LidarConstIteratorAttribute<double> itBeforeEndLidarContainerZ = lidarContainer.endAttribute<double>("z")-1;

	BOOST_CHECK_EQUAL(*itBeforeEndLidarContainerX, lastX);
	BOOST_CHECK_EQUAL(*itBeforeEndLidarContainerY, lastY);
	BOOST_CHECK_EQUAL(*itBeforeEndLidarContainerZ, lastZ);}



BOOST_AUTO_TEST_SUITE_END()
