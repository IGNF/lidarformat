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


#ifndef LIDARFILE_H_
#define LIDARFILE_H_

#include <string>
#include <boost/shared_ptr.hpp>

#include "LidarFormat/LidarDataFormatTypes.h"
#include "LidarFormat/LidarFileIO.h"

/**
* @brief Base Class for lidar file handling
*
*
*/

using boost::shared_ptr;

namespace cs
{
class LidarDataType;
}

namespace Lidar
{

class LidarDataContainer;
class LidarCenteringTransfo;


class LidarFile
{
public:
    explicit LidarFile(const std::string &xmlFileName);
    virtual ~LidarFile();

    /// tests if xml file is valid (according to xsd model)
    virtual bool isValid() const { return m_isValid; }
    /// get main metadata in a string
    virtual std::string getMetaData() const;
    /// get data format (ascii, binary, ...)
    virtual std::string getFormat() const;
    /// get BinaryDataFileName (deprecated, depends on format)
    virtual std::string getBinaryDataFileName() const;
    /// get number  of points in the file
    virtual unsigned int getNbPoints() const;


    /// load centering transfo, (0,0) si none
    void loadTransfo(LidarCenteringTransfo& transfo) const;

    /// load data from file to a lidar container
    void loadData(LidarDataContainer& lidarContainer);

    /// Save container data in a file
    static void save(LidarDataContainer& lidarContainer,
                     const std::string& xmlFileName,
                     const LidarCenteringTransfo& transfo,
                     const cs::DataFormatType format);
    static void save(LidarDataContainer& lidarContainer,
                     const std::string& xmlFileName,
                     const cs::DataFormatType format);
    static void save(LidarDataContainer& lidarContainer,
                     const std::string& dataFileName,
                     const LidarCenteringTransfo& transfo);
    static void save(LidarDataContainer& lidarContainer,
                     const std::string& dataFileName);

    /// Save container data in the same file (in place)
    static void saveInPlace(LidarDataContainer& lidarContainer, const std::string& xmlFileName);

    /// Create xml structure from lidar container
    static shared_ptr<cs::LidarDataType> createXMLStructure(
            const LidarDataContainer& lidarContainer,
            const std::string& dataFileName,
            const LidarCenteringTransfo& transfo,
            const cs::DataFormatType format=cs::DataFormatType::binary);

protected:
    /// xml file with infos on dataset
    std::string m_xmlFileName;
    /// xml data : xml file loaded in memory
    boost::shared_ptr<cs::LidarDataType> m_xmlData;
    /// is the xml structure valid ?
    bool m_isValid;

    /// file meta data
    //XMLLidarMetaData m_lidarMetaData;
    //XMLAttributeMetaDataContainerType m_attributeMetaData;

    /// useful methods
    /// load meta data from xml DEPRECATED, does nothing (now done by the MetaReaders for each format)
    void loadMetaDataFromXML();

    void setMapsFromXML(LidarDataContainer& lidarContainer) const;

};

} //namespace Lidar

#endif /* LIDARFILE_H_ */
