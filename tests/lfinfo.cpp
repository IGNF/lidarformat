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

        Nicolas David, Olivier Tournaire, Bruno Vallet
	
	

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

/** \example lfinfo.cpp by Bruno Vallet
 * This displays infos on the lidarformat file
 */


#include <boost/filesystem.hpp>

#include <algorithm>

#include "config_data_test.h"

#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/LidarFile.h"
#include "LidarFormat/geometry/LidarCenteringTransfo.h"

using namespace Lidar;
using namespace std;

template<typename T> struct FonctorBound
{
    FonctorBound<T>(const unsigned int decalage):
            m_decalage(decalage),
            m_min(std::numeric_limits<T>::max()),
            m_max(std::numeric_limits<T>::min())
    {
        if(!std::numeric_limits<T>::is_integer) m_max=-m_min;
    }

    void operator()(const Lidar::LidarEcho& echo)
    {
        double val = echo.value<T>(m_decalage);
        if(val < m_min) m_min = val;
        if(val > m_max) m_max = val;
    }
    unsigned int m_decalage;
    T m_min, m_max;

};

template<typename T> void TGetMinMax(const LidarDataContainer & ldc, const std::string & attrib)
{
    FonctorBound<T> fb(ldc.getDecalage(attrib));
    fb = for_each (ldc.begin(), ldc.end(), fb);
    cout << "-" << attrib << " (" << LidarTypeTraits<T>::name() << "=" <<
            LidarTypeTraits<T>::old_name() << ") " << fb.m_min << " to " << fb.m_max << endl;
}

int main(int argc, char** argv)
{
    if(argc<2)
    {
        cout << "Usage: " << argv[0] << " lidarfile.xml" << std::endl;
    }
    LidarFile file(argv[1]);
    LidarDataContainer ldc;
    file.loadData(ldc);
    LidarCenteringTransfo transfo;
    file.loadTransfo(transfo);
    cout << "transfo: " << transfo.x() << ", " << transfo.y() << endl;

    std::vector<std::string> attrib_list;
    ldc.getAttributeList(attrib_list);
    cout << "Attribute (type=old_type) min to max:" << endl;

    for(std::vector<std::string>::iterator it = attrib_list.begin(); it != attrib_list.end(); it++)
    {
        switch(ldc.getAttributeType(*it))
        {
        case LidarDataType::float32: TGetMinMax<float32>(ldc,*it); break;
        case LidarDataType::float64: TGetMinMax<float64>(ldc,*it); break;
        case LidarDataType::int8: TGetMinMax<int8>(ldc,*it); break;
        case LidarDataType::int16: TGetMinMax<int16>(ldc,*it); break;
        case LidarDataType::int32: TGetMinMax<int32>(ldc,*it); break;
        case LidarDataType::int64: TGetMinMax<int64>(ldc,*it); break;
        case LidarDataType::uint8: TGetMinMax<uint8>(ldc,*it); break;
        case LidarDataType::uint16: TGetMinMax<uint16>(ldc,*it); break;
        case LidarDataType::uint32: TGetMinMax<uint32>(ldc,*it); break;
        case LidarDataType::uint64: TGetMinMax<uint64>(ldc,*it); break;
        }
    }

    return 0;
}



