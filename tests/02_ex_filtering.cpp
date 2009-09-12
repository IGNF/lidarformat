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



