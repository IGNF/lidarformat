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
#include <time.h>

#include "config_data_test.h"

#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/LidarFile.h"
#include "LidarFormat/geometry/LidarCenteringTransfo.h"

using namespace Lidar;
using namespace std;

// new way
class AbstractBound
{
public:
    AbstractBound(const EnumLidarDataType type):m_type(type){}
    virtual void Add(const Lidar::LidarEcho& echo) = 0;
    virtual void Print() = 0;

    std::string m_name;
    unsigned int m_decalage;
    EnumLidarDataType m_type;
};

template<typename T> class TBound : public AbstractBound
{
public:
    TBound(const EnumLidarDataType type):
        AbstractBound(type),
        m_min(std::numeric_limits<T>::max()),
        m_max(std::numeric_limits<T>::min())
    {
        if(!std::numeric_limits<T>::is_integer) m_max=-m_min;
    }

    void Add(const Lidar::LidarEcho& echo)
    {
        double val = echo.value<T>(m_decalage);
        if(val < m_min) m_min = val;
        if(val > m_max) m_max = val;
    }

    void Print()
    {
        cout << m_name << " (" << LidarTypeTraits<T>::name() << "/" <<
                LidarTypeTraits<T>::old_name() << ") ";
        // convert (unsigned) char to int
        if(std::numeric_limits<T>::is_integer)
            cout << (int)m_min << " to " << (int)m_max << " = " << (int)m_max - (int)m_min + 1 << endl;
        else cout << m_min << " to " << m_max << " = " << m_max-m_min << endl;
    }

    T m_min, m_max;
};

struct FonctorMultiAbstractBound
{
    FonctorMultiAbstractBound(){}
    void AddAttribute(const std::string & name, const unsigned int decalage, const EnumLidarDataType type)
    {
        AbstractBound* p_bound=NULL;
        switch(type)
        {
        case LidarDataType::float32: p_bound = new TBound<float32>(type); break;
        case LidarDataType::float64: p_bound = new TBound<float64>(type); break;
        case LidarDataType::int8: p_bound = new TBound<int8>(type); break;
        case LidarDataType::int16: p_bound = new TBound<int16>(type); break;
        case LidarDataType::int32: p_bound = new TBound<int32>(type); break;
        case LidarDataType::int64: p_bound = new TBound<int64>(type); break;
        case LidarDataType::uint8: p_bound = new TBound<uint8>(type); break;
        case LidarDataType::uint16: p_bound = new TBound<uint16>(type); break;
        case LidarDataType::uint32: p_bound = new TBound<uint32>(type); break;
        case LidarDataType::uint64: p_bound = new TBound<uint64>(type); break;
        }
        if(p_bound)
        {
            p_bound->m_name = name;
            p_bound->m_decalage = decalage;
            mvp_attrib.push_back(p_bound);
        }
    }

    void operator()(const Lidar::LidarEcho& echo)
    {
        for(std::vector<AbstractBound*>::iterator it=mvp_attrib.begin(); it!=mvp_attrib.end(); it++)
            (*it)->Add(echo);
    }
    void Print()
    {
        for(std::vector<AbstractBound*>::iterator it=mvp_attrib.begin(); it!=mvp_attrib.end(); it++)
            (*it)->Print();
    }
    std::vector<AbstractBound*> mvp_attrib;
};

// main

int main(int argc, char** argv)
{
    if(argc<2)
    {
        cout << "Usage: " << argv[0] << " lidarfile.xml" << std::endl;
        return 0;
    }
    for(int i=1; i<argc; i++)
    {
        time_t timer = clock();
        LidarFile file(argv[i]);
        LidarDataContainer ldc;
        file.loadData(ldc);
        LidarCenteringTransfo transfo;
        file.loadTransfo(transfo);
        cout << "transfo: " << transfo.x() << ", " << transfo.y() << endl;

        std::vector<std::string> attrib_list;
        ldc.getAttributeList(attrib_list);
        cout << "Attribute (type/old_type) min to max = max-min:" << endl;

        FonctorMultiAbstractBound fmb;
        for(std::vector<std::string>::iterator it = attrib_list.begin(); it != attrib_list.end(); it++)
            fmb.AddAttribute(*it, ldc.getDecalage(*it), ldc.getAttributeType(*it));
        fmb = for_each (ldc.begin(), ldc.end(), fmb);
        fmb.Print();
        timer = clock()-timer;
        std::cout << "Time: " << ( double ) timer/CLOCKS_PER_SEC << " s" << std::endl;
    }
    return 0;
}



