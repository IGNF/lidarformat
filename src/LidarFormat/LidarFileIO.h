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
//	unsigned int ptSize_; //taille d'un point avec tous ses attributs chargés UNIQUEMENT
	std::size_t nbPoints_; //nombre de points du fichier
	std::string xmlFileName_; //fichier xml de données
	std::string binaryDataFileName_; //fichier binaire associé
};

struct XMLAttributeMetaData
{
	XMLAttributeMetaData(): name_(""), type_(LidarDataType::int8), loaded_(false) {}
	explicit XMLAttributeMetaData(const std::string &name, const EnumLidarDataType type, const bool loaded):
		name_(name), type_(type), loaded_(loaded) {}
	std::string name_; //atribute name
	EnumLidarDataType type_; //attribute type
//	unsigned int size_; //taille d'un attribut
	bool loaded_;
};
typedef std::vector<XMLAttributeMetaData> XMLAttributeMetaDataContainerType;


class LidarFileIO
{
	public:
		virtual ~LidarFileIO();

		virtual void loadData(LidarDataContainer& lidarContainer, const XMLLidarMetaData& lidarMetaData, const XMLAttributeMetaDataContainerType& attributesDescritpion)=0;
		virtual void save(const LidarDataContainer& lidarContainer, const std::string& binaryDataFileName)=0;


		void setXMLData(const boost::shared_ptr<cs::LidarDataType>& xmlData);

	protected:
		LidarFileIO();

		boost::shared_ptr<cs::LidarDataType> m_xmlData;

};

} //namespace Lidar

#endif /* LIDARFILEIO_H_ */
