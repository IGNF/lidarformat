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

#include "LidarFormat/tools/AttributeBounds.h"

namespace Lidar
{

AbstractBound::AbstractBound(const EnumLidarDataType type):m_type(type){}

FonctorMultiAbstractBound::FonctorMultiAbstractBound(){}
void FonctorMultiAbstractBound::AddAttribute(const std::string & attrib_name, const unsigned int decalage, const EnumLidarDataType type)
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
        p_bound->m_name = attrib_name;
        p_bound->m_decalage = decalage;
        mvp_attrib.push_back(p_bound);
    }
}

void FonctorMultiAbstractBound::operator()(const Lidar::LidarEcho& echo)
{
    for(std::vector<AbstractBound*>::iterator it=mvp_attrib.begin(); it!=mvp_attrib.end(); it++)
        (*it)->Add(echo);
}

void FonctorMultiAbstractBound::Print()
{
    std::cout << "Attribute (type/old_type) [min,max]=range:" << std::endl;
    for(std::vector<AbstractBound*>::iterator it=mvp_attrib.begin(); it!=mvp_attrib.end(); it++)
        (*it)->Print();
}

} //namespace Lidar

