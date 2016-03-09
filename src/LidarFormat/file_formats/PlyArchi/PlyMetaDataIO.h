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

#pragma once

#include "LidarFormat/LidarFileIO.h"

namespace Lidar
{
class PlyMetaDataIO : public MetaDataIO
{
public:
    virtual ~PlyMetaDataIO(){}

    virtual boost::shared_ptr<cs::LidarDataType> load(const std::string& filename);

    static bool Register();
    friend boost::shared_ptr<PlyMetaDataIO> createPlyMetaDataReader();

private:
    PlyMetaDataIO(){}

    static bool m_isRegistered;
};

}
