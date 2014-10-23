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

/** \example histo.cpp by Bruno Vallet
 * This displays the (sparse) histogram of an attribute on the lidarformat file
 */
#include <time.h>

#include "config_data_test.h"

#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/LidarFile.h"
#include "LidarFormat/geometry/LidarCenteringTransfo.h"
#include "LidarFormat/tools/AttributeBounds.h"

using namespace Lidar;
using namespace std;

// histo for float classes
template <class T> void Histo(LidarDataContainer & ldc, unsigned int shift, int bins=100)
{
    vector<unsigned int> histo;
    T min=numeric_limits<T>::max(), max=numeric_limits<T>::min();
    for(LidarDataContainer::iterator it=ldc.begin(); it!=ldc.end(); it++)
    {
        T val = it.value<T>(shift);
        if(val<min) min=val;
        if(val>max) max=val;
    }
    T dv = (max-min)/bins;
    for(LidarDataContainer::iterator it=ldc.begin(); it!=ldc.end(); it++)
    {
        T val = it.value<T>(shift);
        int i = (val-min)/dv;
        if(i<0) i=0;
        if(!(i<histo.size())) i = histo.size();
        histo[i]++;
    }
    for(int i=0; i<histo.size(); i++)
    {
        cout << min+i*dv << "-" << min+(i+1)*dv << ":" << histo[i] << endl;
    }
}

// sparse histo for integer classes
template <class T> void SparseHisto(LidarDataContainer & ldc, unsigned int shift)
{
    map<T, unsigned int> sparse_histo;
    for(LidarDataContainer::iterator it=ldc.begin(); it!=ldc.end(); it++)
    {
        T val = it.value<T>(shift);
        typename map<T, unsigned int>::iterator key = sparse_histo.find(val);
        if(key == sparse_histo.end()) sparse_histo[val] = 1;
        else key->second++;
    }
    for(typename map<T, unsigned int>::iterator key = sparse_histo.begin(); key != sparse_histo.end(); key++)
    {
        cout << (int)key->first << ":\t" << key->second << endl;
    }
}

int main(int argc, char** argv)
{
    if(argc<3)
    {
        cout << "Usage: " << argv[0] << " lidarfile.xml attrib_name [n_bins=100]" << std::endl;
        cout << "Interger attribs: sparse histogram" << std::endl;
        cout << "Real attribs: n_bins bins between min and max" << std::endl;
        return 0;
    }
    string lidar_filename(argv[1]);
    string attrib_name(argv[2]);
    int n_bins=100;
    if(argc>3) n_bins = atoi(argv[3]);

    time_t timer = clock();
    LidarFile file(lidar_filename);
    cout << file.getMetaData();
    LidarDataContainer ldc;
    file.loadData(ldc);

    if(!ldc.checkAttributeIsPresent(attrib_name))
    {
        cout << "No attribute " << attrib_name << " in " << lidar_filename << endl;
        return 1;
    }
    int shift = ldc.getDecalage(attrib_name);

    switch(ldc.getAttributeType(attrib_name))
    {
    case LidarDataType::float32: Histo<float32>(ldc, shift, n_bins); break;
    case LidarDataType::float64: Histo<float64>(ldc, shift, n_bins); break;
    case LidarDataType::int8: SparseHisto<int8>(ldc, shift); break;
    case LidarDataType::int16: SparseHisto<int16>(ldc, shift); break;
    case LidarDataType::int32: SparseHisto<int32>(ldc, shift); break;
    case LidarDataType::int64: SparseHisto<int64>(ldc, shift); break;
    case LidarDataType::uint8: SparseHisto<uint8>(ldc, shift); break;
    case LidarDataType::uint16: SparseHisto<uint16>(ldc, shift); break;
    case LidarDataType::uint32: SparseHisto<uint32>(ldc, shift); break;
    case LidarDataType::uint64: SparseHisto<uint64>(ldc, shift); break;
    }

    timer = clock()-timer;
    std::cout << "Time: " << ( double ) timer/CLOCKS_PER_SEC << " s" << std::endl;
    return 0;
}



