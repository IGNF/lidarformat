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
#include <time.h>

#include "config_data_test.h"

#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/LidarFile.h"
#include "LidarFormat/geometry/LidarCenteringTransfo.h"
#include "LidarFormat/tools/AttributeBounds.h"

using namespace Lidar;
using namespace std;

class AbstractAttribute
{
public:
    AbstractAttribute(const EnumLidarDataType type):m_type(type){}
    virtual string String(const Lidar::LidarEcho& echo)=0;

    string m_name;
    unsigned int m_decalage;
    EnumLidarDataType m_type;
};

template<typename T> class TAttribute : public AbstractAttribute
{
public:
    TAttribute(const EnumLidarDataType type):
        AbstractAttribute(type) {}

    string String(const Lidar::LidarEcho& echo)
    {
        ostringstream oss;
        if(std::numeric_limits<T>::is_integer)
            oss << (int)echo.value<T>(m_decalage);
        else oss << echo.value<T>(m_decalage);
        return oss.str();
    }
};

// Factory
AbstractAttribute * CreateAttribute(const std::string & attrib_name,
                                    const unsigned int decalage,
                                    const EnumLidarDataType type)
{
    AbstractAttribute* p_attrib=NULL;
    switch(type)
    {
    case LidarDataType::float32: p_attrib = new TAttribute<float32>(type); break;
    case LidarDataType::float64: p_attrib = new TAttribute<float64>(type); break;
    case LidarDataType::int8: p_attrib = new TAttribute<int8>(type); break;
    case LidarDataType::int16: p_attrib = new TAttribute<int16>(type); break;
    case LidarDataType::int32: p_attrib = new TAttribute<int32>(type); break;
    case LidarDataType::int64: p_attrib = new TAttribute<int64>(type); break;
    case LidarDataType::uint8: p_attrib = new TAttribute<uint8>(type); break;
    case LidarDataType::uint16: p_attrib = new TAttribute<uint16>(type); break;
    case LidarDataType::uint32: p_attrib = new TAttribute<uint32>(type); break;
    case LidarDataType::uint64: p_attrib = new TAttribute<uint64>(type); break;
    }
    if(p_attrib)
    {
        p_attrib->m_name = attrib_name;
        p_attrib->m_decalage = decalage;
    }
    return p_attrib;
}

int main(int argc, char** argv)
{
    if(argc<2)
    {
        cout << "Usage: " << argv[0] << " lidarfile [n_echo]" << endl;
        cout << "lidarfile: lidar file to analyse in a lidarformat supported format (.xml, .bin, .ply,...)" << endl;
        cout << "n_echo=0: number of echo attributes to dispay in a tabular format" << endl;
        return 0;
    }
    int n_echo = 0;
    if(argc>2) n_echo = atoi(argv[2]);
    time_t timer = clock();
    LidarFile file(argv[1]);
    cout << file.getMetaData();
    LidarDataContainer ldc;
    file.loadData(ldc);
    LidarCenteringTransfo transfo;
    file.loadTransfo(transfo);
    cout << "transfo: " << transfo.x() << ", " << transfo.y() << endl;

    vector<string> attrib_list;
    ldc.getAttributeList(attrib_list);
    FonctorMultiAbstractBound fmab;
    vector<AbstractAttribute*> v_attrib;
    for(vector<string>::iterator it = attrib_list.begin(); it != attrib_list.end(); it++)
    {
        cout << *it << " decal " << ldc.getDecalage(*it) << " type " << ldc.getAttributeType(*it);
        double min=0., max=0.;
        if(ldc.getAttributeBounds(*it, min, max))
        {
            cout << " in [" << min << "," << max << "]" << endl;
        } else cout << " no bounds in metainfo" << endl;
        fmab.AddAttribute(*it, ldc.getDecalage(*it), ldc.getAttributeType(*it));
        v_attrib.push_back(CreateAttribute(*it, ldc.getDecalage(*it), ldc.getAttributeType(*it)));
    }
    fmab = for_each (ldc.begin(), ldc.end(), fmab);
    fmab.Print();
    timer = clock()-timer;
    std::cout << "Time: " << ( double ) timer/CLOCKS_PER_SEC << " s" << std::endl;

    if(n_echo==0) return 0;
    for(vector<string>::iterator it = attrib_list.begin(); it != attrib_list.end(); it++)
        cout << *it << "\t";
    cout << endl;
    for(LidarDataContainer::iterator it=ldc.begin(); it<ldc.begin()+n_echo; it++)
    {
        for(unsigned int i=0; i<v_attrib.size(); i++)
            cout << v_attrib[i]->String(*it) << "\t";
        cout << endl;
    }
    return 0;
}
