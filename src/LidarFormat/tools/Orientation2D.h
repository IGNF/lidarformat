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

#ifndef ORIENTATION2D_H_
#define ORIENTATION2D_H_

/**
 * \class Orientation2D
 * \brief Classe de gestion des orientations 2D en géométrie ortho
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
	x = (float)(m_originX + col * m_step);
	y = (float)(m_originY - lig * m_step);
}

inline void Orientation2D::MapToImage(const float x, const float y, int &col, int &lig) const
{
	col = static_cast<int>( std::floor((x - m_originX ) / m_step + 0.5) ); //+0.5 pour faire un ROUND
	lig = -static_cast<int>( std::floor((y - m_originY ) / m_step + 0.5) );
}

} //namespace Lidar

#endif /*ORIENTATION2D_H_*/
