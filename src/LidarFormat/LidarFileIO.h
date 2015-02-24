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


#ifndef LIDARFILEIO_H_
#define LIDARFILEIO_H_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "LidarFormat/LidarDataFormatTypes.h"

namespace Lidar
{

class LidarDataContainer;

/// BV: refactored with 2 objectives:
/// 1) to have an identical api for lidarformat (.xml/.(bin|txt)) and non-lidarformat (.ply, .las,...) file formats
/// filename should be the .xml in the first case and .ply/.las in the second
/// 2) keeping the possibility to load meta-data and data separately
/// As in OpenSceneGraph, all loaders are attempted (in the order they were declared) to load meta data
/// thus a loader should simply return false if anything is wrong (starting with wrong extention
class MetaDataIO
{
public:
    virtual ~MetaDataIO();

    /// load meta data from the specific file format
    /// throws on error
    virtual boost::shared_ptr<cs::LidarDataType> load(const std::string& filename)=0;

protected:
    MetaDataIO();
};

class LidarFileIO
{
public:
    virtual ~LidarFileIO();

    /// BV: all meta data assumed loaded in the container and memory allocated
    /// for lidarformat formats, filename is the xml and it is used to make the binary filename (from metadata) absolute
    /// throws on error
    virtual void loadData(LidarDataContainer& lidarContainer, std::string filename)=0;

    /// data filename is in the container's xml structure.
    /// for lidarformat, accompanying xml filename is inferred from data filename by replacing ext by .xml
    virtual void save(const LidarDataContainer& lidarContainer, std::string filename)=0;

    /// DEPRECATED, use loadMetaData() instead
    void setXMLData(const boost::shared_ptr<cs::LidarDataType>& xmlData);

protected:
    LidarFileIO();

    boost::shared_ptr<cs::LidarDataType> m_xmlData;

};

} //namespace Lidar

#endif /* LIDARFILEIO_H_ */
