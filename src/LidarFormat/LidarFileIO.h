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


#ifndef LIDARFILEIO_H_
#define LIDARFILEIO_H_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "LidarFormat/LidarDataFormatTypes.h"

namespace Lidar
{

class LidarDataContainer;



struct XMLLidarMetaData
{
	XMLLidarMetaData() : nbPoints_(0), xmlFileName_(""), binaryDataFileName_("") {}
    std::size_t nbPoints_;
    std::string xmlFileName_;
    std::string binaryDataFileName_;
};

struct XMLAttributeMetaData
{
    XMLAttributeMetaData(): name_(""), type_(LidarDataType::int8), loaded_(false), min_(0.), max_(0.) {}
    explicit XMLAttributeMetaData(const std::string &name, const EnumLidarDataType type, const bool loaded,
                                  const bool dirty, const double min=0., const double max=0.):
        name_(name), type_(type), loaded_(loaded), dirty_(dirty), min_(min), max_(max) {}
    std::string name_; //attribute name
	EnumLidarDataType type_; //attribute type
    bool loaded_, dirty_; // attribute is loaded, attribute bounds are dirty (need recompute)
    double min_, max_; // min/max attribute value
};
typedef std::vector<XMLAttributeMetaData> XMLAttributeMetaDataContainerType;


class LidarFileIO
{
	public:
		virtual ~LidarFileIO();

		virtual void loadData(LidarDataContainer& lidarContainer, const XMLLidarMetaData& lidarMetaData, const XMLAttributeMetaDataContainerType& attributesDescritpion)=0;
        // BV: in order to export standard filetypes (ply, las,...) save should have metadata
        virtual void save(const LidarDataContainer& lidarContainer, const cs::LidarDataType& xmlStructure, const std::string& dataFileName)=0;


		void setXMLData(const boost::shared_ptr<cs::LidarDataType>& xmlData);

	protected:
		LidarFileIO();

		boost::shared_ptr<cs::LidarDataType> m_xmlData;

};

} //namespace Lidar

#endif /* LIDARFILEIO_H_ */
