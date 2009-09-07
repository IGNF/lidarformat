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
 * \file test1.cpp
 * \brief
 * \author Adrien Chauve
 * \date 15 févr. 2009
 */

#include <boost/filesystem.hpp>

#include <algorithm>

#include "config_data_test.h"

#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/LidarFile.h"


using namespace Lidar;
using namespace std;

struct FoncteurFiltre
{
	FoncteurFiltre(const unsigned int decalage, const double seuilMin, const double seuilMax):
		decalage_(decalage), seuilMin_(seuilMin), seuilMax_(seuilMax)
		{
		}

	bool operator()(const LidarEcho& echo) const
	{
		return echo.value<double>(decalage_)>=seuilMin_ && echo.value<double>(decalage_)<=seuilMax_;
	}

	unsigned int decalage_;
	double seuilMin_, seuilMax_;

};


struct LessEcho
{
	LessEcho(const unsigned int decalage):
		decalage_(decalage){}

	bool operator()(const LidarEcho& e1, const LidarEcho& e2) const
	{
		return e1.value<double>(decalage_) < e2.value<double>(decalage_);
	}

	unsigned int decalage_;
};


int main()
{

	LidarDataContainer lidarContainer;
	lidarContainer.addAttribute("x", LidarDataType::float64);
	lidarContainer.addAttribute("y", LidarDataType::float64);
	lidarContainer.addAttribute("z", LidarDataType::float64);
	LidarEcho echo = lidarContainer.createEcho();

	for(unsigned int i=0; i<15; ++i)
	{
		echo.value<double>("x") = 0;
		echo.value<double>("y") = 0;
		echo.value<double>("z") = i;
		lidarContainer.push_back(echo);
	}



	cout << "\n\tFiltering test" << endl;
	cout << "\n\t\tInitial data: " << endl;

	ostream_iterator<LidarEcho> it_outputEcho( cout, "\n" );
	copy(lidarContainer.begin(), lidarContainer.end(), it_outputEcho);

	cout << "\n\t\tShuffled data: " << endl;

	random_shuffle(lidarContainer.begin(), lidarContainer.end());

	copy(lidarContainer.begin(), lidarContainer.end(), it_outputEcho);


	LidarIteratorEcho end_remove = remove_if(lidarContainer.begin(), lidarContainer.end(), FoncteurFiltre(lidarContainer.getDecalage("z"), 3, 7));

	cout << "\n\t\tFiltered data: " << endl;
	lidarContainer.erase( end_remove, lidarContainer.end());

	copy(lidarContainer.begin(), lidarContainer.end(), it_outputEcho);

	cout << "\n\t\tSorted data: " << endl;
	sort(lidarContainer.begin(), lidarContainer.end(), LessEcho(lidarContainer.getDecalage("z")));
	copy(lidarContainer.begin(), lidarContainer.end(), it_outputEcho);

	return 0;
}



