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

#ifndef ATTRIBUTEBOUNDS_H_
#define ATTRIBUTEBOUNDS_H_

/**
 * \class AttributeBounds
 * \brief Classe de gestion des intervalles pour chaque attribut
 *
 */

#include <iostream>
#include "LidarFormat/LidarEcho.h"

namespace Lidar
{

class AbstractBound
{
public:
    AbstractBound(const EnumLidarDataType type);
    virtual void Add(const Lidar::LidarEcho& echo) = 0;
    virtual void Print() = 0;
    virtual void Get(double & min, double & max) = 0;

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
        std::cout << m_name << " (" << LidarTypeTraits<T>::name() << "/" <<
                LidarTypeTraits<T>::old_name() << ") ";
        // convert (unsigned) char to int
        if(std::numeric_limits<T>::is_integer)
            std::cout << "[" << (int)m_min << "," << (int)m_max << "]=" << (int)m_max - (int)m_min + 1 << std::endl;
        else std::cout << "[" << m_min << "," << m_max << "]=" << m_max-m_min << std::endl;
    }

    void Get(double & min, double & max)
    {
        min = m_min; max = m_max;
    }

    T m_min, m_max;
};

struct FonctorMultiAbstractBound
{
    FonctorMultiAbstractBound();
    void AddAttribute(const std::string & attrib_name, const unsigned int decalage, const EnumLidarDataType type);
    void operator()(const Lidar::LidarEcho& echo);
    void Print();
    std::vector<AbstractBound*> mvp_attrib;
};

} //namespace Lidar

#endif /*ATTRIBUTEBOUNDS_H_*/
