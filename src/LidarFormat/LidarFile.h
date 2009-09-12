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


#ifndef LIDARFILE_H_
#define LIDARFILE_H_

#include <string>
#include <boost/shared_ptr.hpp>

#include "LidarFormat/LidarDataFormatTypes.h"
#include "LidarFormat/LidarFileIO.h"

/**
* @brief Classe de base de gestion des fichiers lidar.
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

		///Teste si le fichier xml est valide (par rapport au modèle xsd)
		virtual bool isValid() const { return m_isValid; }
		///Récupère les méta-données principales dans une string
		virtual std::string getMetaData() const;
		///Récupère le format des données (ascii, binaire, ...)
		virtual std::string getFormat() const;
		///Récupère le nom du fichier de données
		virtual std::string getBinaryDataFileName() const;
		///Récupère le nb de points du nuage
		virtual unsigned int getNbPoints() const;



		///Recupere la transfo de centrage, (0,0) si pas de transfo
		void loadTransfo(LidarCenteringTransfo& transfo) const;

		///Charge les données du fichier dans un conteneur lidar
		void loadData(LidarDataContainer& lidarContainer);

		///Save container data in a file
		static void save(const LidarDataContainer& lidarContainer, const std::string& xmlFileName, const LidarCenteringTransfo& transfo, const cs::DataFormatType format=cs::DataFormatType::binary);
		static void save(const LidarDataContainer& lidarContainer, const std::string& xmlFileName, const cs::DataFormatType format=cs::DataFormatType::binary);
		static void save(const LidarDataContainer& lidarContainer, const std::string& xmlFileName, const cs::LidarDataType& xmlStructure);

		///Save container data in the same file (in place)
		static void saveInPlace(const LidarDataContainer& lidarContainer, const std::string& xmlFileName);

		///Create xml structure from lidar container
		static shared_ptr<cs::LidarDataType> createXMLStructure(const LidarDataContainer& lidarContainer, const std::string& xmlFileName, const LidarCenteringTransfo& transfo, const cs::DataFormatType format=cs::DataFormatType::binary);

	protected:
		///fichier xml qui regroupe toutes les infos sur les données
		std::string m_xmlFileName;
		///données xml : fichier xml chargé en mémoire
		boost::shared_ptr<cs::LidarDataType> m_xmlData;
		///fichier valide (structure xml) ?
		bool m_isValid;






		///Méta-données du fichier
		XMLLidarMetaData m_lidarMetaData;
		XMLAttributeMetaDataContainerType m_attributeMetaData;

		///fonctions utiles
		///Chargement des méta-données à partir du xml
		void loadMetaDataFromXML();

		void setMapsFromXML(LidarDataContainer& lidarContainer) const;

};

} //namespace Lidar

#endif /* LIDARFILE_H_ */
