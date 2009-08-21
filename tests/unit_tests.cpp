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
 * \file unit_tests.cpp
 * \brief
 * \author Adrien Chauve
 * \date 1 avr. 2009
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE LidarFormatUnitTests
#include <boost/test/unit_test.hpp>

#include "config_data_test.h"

#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/LidarFile.h"

using namespace Lidar;
using namespace std;



BOOST_AUTO_TEST_SUITE(LidarDataContainerTests)


//fichier xml de test
const string lidarFileName(string(PATH_LIDAR_TEST_DATA) + "/testAscii.xml");

const double firstX = 919351.96, firstY = 1914105.38, firstZ = 1075.35;
const double lastX = 919360.56, lastY = 1914108.38, lastZ = 1079.2;



BOOST_AUTO_TEST_CASE( LidarIteratorEcho_tests )
{
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
