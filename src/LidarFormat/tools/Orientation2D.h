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

#ifndef ORIENTATION2D_H_
#define ORIENTATION2D_H_

/**
 * \class Orientation2D
 * \brief Classe de gestion des orientations 2D en géométrie ortho
 * \author A. Chauve
 * \version 0.15
 * \date 03/2008
 *
 * Classe qui lit et écrit les fichiers ORI, et passe de l'indice de pixel aux coordonnées X,Y.
 *
 */

#include <string>
#include <cmath>

namespace Lidar
{

class Orientation2D
{
public:
	Orientation2D();
	Orientation2D(const double origineX, const double origineY, const double step,const unsigned int zoneCarto, const unsigned int tailleX, const unsigned int tailleY);

	///Accesseurs/Setteurs
	double OriginX() const { return m_originX; }
	void OriginX( const double x) { m_originX = x; }
	double OriginY() const { return m_originY; }
	void OriginY( const double y) { m_originY = y; }

	double Step() const { return m_step; }
	void Step( const double s) { m_step = s; }

	unsigned int ZoneCarto() const { return m_zoneCarto; }
	void ZoneCarto( const unsigned int zone) { m_zoneCarto = zone; }

	unsigned int SizeX() const { return m_sizeX; }
	void SizeX( const unsigned int size) { m_sizeX = size; }
	unsigned int SizeY() const { return m_sizeY; }
	void SizeY( const unsigned int size) { m_sizeY = size; }

	///Passage de pixel a image et inversement
	inline void ImageToMap(const int col, const int lig, float &x, float &y) const;
	inline void MapToImage(const float x, const float y, int &col, int &lig) const;

	///IO : renvoit des exceptions si mauvais format ou pbs en lecture
	void ReadOriFromOriFile(const std::string &filename);
	void ReadOriFromTFWFile(const std::string &filename);
	void ReadOriFromImageFile(const std::string &filename);

	void SaveOriToFile(const std::string &filename);

	std::string Affiche() const;

private:

	///Origine en X et Y de la couche 2D
	double m_originX, m_originY;
	///Resolution en X et Y de la couche 2D
	double m_step;

	unsigned int m_zoneCarto;
	unsigned int m_sizeX, m_sizeY;

};

inline void Orientation2D::ImageToMap(const int col, const int lig, float &x, float &y) const
{
	x = m_originX + col * m_step;
	y = m_originY - lig * m_step;
}

inline void Orientation2D::MapToImage(const float x, const float y, int &col, int &lig) const
{
	col = static_cast<int>( std::floor((x - m_originX ) / m_step + 0.5) ); //+0.5 pour faire un ROUND
	lig = -static_cast<int>( std::floor((y - m_originY ) / m_step + 0.5) );
}

} //namespace Lidar

#endif /*ORIENTATION2D_H_*/
