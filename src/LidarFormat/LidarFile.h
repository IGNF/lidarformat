/***********************************************************************

This file is part of the LidarFormat project source files.

LidarFormat is an open source library for efficiently handling 3D point 
clouds with a variable number of attributes at runtime. LidarFormat is 
distributed under the CeCILL-B licence. See Licence_CeCILL-B_V1-en.txt 
or http://www.cecill.info for more details.


Homepage: 

	https://fullanalyze.ign.fr/trac/LidarFormat
	
Copyright:
	
	Institut Geographique National & CEMAGREF (2009)

Author: 

	Adrien Chauve
	
Contributors:

	Nicolas David, Olivier Tournaire



This software is governed by the CeCILL-B license under French law and
abiding by the rules of distribution of free software.  You can  use, 
modify and/ or redistribute the software under the terms of the CeCILL-B
license as circulated by CEA, CNRS and INRIA at the following URL
"http://www.cecill.info". 

As a counterpart to the access to the source code and  rights to copy,
modify and redistribute granted by the license, users are provided only
with a limited warranty  and the software's author,  the holder of the
economic rights,  and the successive licensors  have only  limited
liability. 

In this respect, the user's attention is drawn to the risks associated
with loading,  using,  modifying and/or developing or reproducing the
software by the user in light of its specific status of free software,
that may mean  that it is complicated to manipulate,  and  that  also
therefore means  that it is reserved for developers  and  experienced
professionals having in-depth computer knowledge. Users are therefore
encouraged to load and test the software's suitability as regards their
requirements in conditions enabling the security of their systems and/or 
data to be ensured and,  more generally, to use and operate it in the 
same conditions as regards security. 

The fact that you are presently reading this means that you have had
knowledge of the CeCILL-B license and that you accept its terms.
 
***********************************************************************/

/*!
 * \file LidarFile.h
 * \brief
 * \author Adrien Chauve
 * \date 29 oct. 2008
 */

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
* @author Adrien Chauve
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
