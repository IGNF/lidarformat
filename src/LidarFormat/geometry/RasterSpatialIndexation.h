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


#ifndef RASTERSPATIALINDEXATION_H_
#define RASTERSPATIALINDEXATION_H_

#include <string>
#include <vector>
//#include <list>

#include <boost/function.hpp>

//#include "outils/stl_tools.h"
//
#include "LidarFormat/tools/Orientation2D.h"
//#include "itk/Image.h"
//
//#include "outils/OutilsMaths.h"

#include "LidarFormat/extern/matis/ttableau2d.h"
#include "LidarFormat/extern/matis/tpoint2d.h"
#include "LidarFormat/extern/matis/tpoint3d.h"


namespace Lidar
{

/*!
 *
 * @brief Classe d'indexation spatiale au format raster.
 *
 * Classe d'indexation spatiale au format raster : créé un tableau 2D en géométrie ortho autour de la BBOX des points à la résolution donnée et indexe dans chaque pixel
 * la pile de points correspondants.
 *
 *
 */
class RasterSpatialIndexation
{
	public:
		typedef std::vector<unsigned int> NeighborhoodListeType;
		typedef TTableau2D<std::vector<unsigned int> > GriddedDataType;


		typedef boost::function<bool(const float, const float, const float)> NeighborhoodFunctionType;

		RasterSpatialIndexation();
		virtual ~RasterSpatialIndexation();

		void setResolution(const float resolution);
		void setBBox(const TPoint2D<float> &bboxMin, const TPoint2D<float> &bboxMax);

		///Fonction de calcul de voisinage par défaut; renvoit true à l'intérieur du voisinage
		static bool defaultIsInside(const float, const float, const float)
		{ return true; }

		///Fonction de calcul de voisinage par défaut; renvoit true à l'intérieur du voisinage
		static bool allPointsFilter(const unsigned int)
		{ return true; }



		///Renvoie un voisinage rectangulaire (à partir des points p1,p2)
		///Le voisinage contient au moins le rectangle (p1,p2), plus une bande autour de largeur max _resolution
		void getApproximateRectangularNeighborhood(NeighborhoodListeType &list, const TPoint2D<float> &p1, const TPoint2D<float> &p2) const;

		///Doit être redéfinie dans les classes filles car pas de méthode générale pour accéder au données et vérifier qu'elles sont dans le voisinage
		virtual void GetCenteredNeighborhood(NeighborhoodListeType &list, const TPoint2D<float> &centre, const float approxNeighborhoodSize, const NeighborhoodFunctionType IsInside = defaultIsInside) const=0;




		const GriddedDataType& getSpatialIndexation() const { return m_griddedData; }
		const Orientation2D getOri() const { return m_ori; }

		const TPoint2D<float> getBBoxMin() const { return m_bboxMin; }
		const TPoint2D<float> getBBoxMax() const { return m_bboxMax; }


		/// Fonction qui lance l'indexation spatiale
		void indexData();

		static unsigned int m_nbPointsParM2; //maxi 10 points/m2 en aeroporté, à tuner en terrestre...

	protected:
		///Fonctions propres à dériver
		virtual void findBBox() = 0; //parcourt les données et récupère la BBox si elle n'a pas été "settée" à la main
		virtual void fillData() = 0; //remplit la grille d'indexation


		///Fonctions propres
		void allocateData(); //alloue la mémoire pour la grille d'indexation


		///Data
		//grille d'indexation
		GriddedDataType m_griddedData;

		float m_resolution;

		//BBox
		TPoint2D<float> m_bboxMin;
		TPoint2D<float> m_bboxMax;

		Orientation2D m_ori;

};


namespace Neighborhoods
{

	template<typename T> inline
	T _sqr(const T x)
	{
		return x*x;
	}

	struct SphericalNeighborhood
	{
			SphericalNeighborhood(const TPoint3D<float> &centre, const float rayon) :
				centre_(centre), rayonCarre_(rayon*rayon)
			{
			}

			bool operator()(const float x, const float y, const float z) const
			{
				return _sqr(x - centre_.x) + _sqr(y - centre_.y) + _sqr(z - centre_.z) <= rayonCarre_;
			}

		private:
			const TPoint3D<float> centre_;
			const float rayonCarre_;
	};

	struct CylindricalNeighborhood
	{
			CylindricalNeighborhood(const TPoint2D< float > &centre, const float rayon) :
				centre_(centre), rayonCarre_(rayon*rayon)
			{
			}

			bool operator()(const float x, const float y, const float z) const
			{
				return _sqr(x - centre_.x) + _sqr(y - centre_.y) <= rayonCarre_;
			}

		private:
			const TPoint2D<float> centre_;
			const float rayonCarre_;
	};

	struct CubicNeighborhood
	{
		CubicNeighborhood(const TPoint3D< float > &centre, const float a) :
			centre_(centre), a_(a)
			{
			}

			bool operator()(const float x, const float y, const float z) const
			{
				return ( std::fabs(x - centre_.x) <= a_ ) && ( std::fabs(y - centre_.y) <= a_ ) && ( std::fabs(z - centre_.z) <= a_ );
			}

		private:
			const TPoint3D<float> centre_;
			const float a_;
	};

}

}//namespace Lidar

#endif /* RASTERSPATIALINDEXATION_H_ */
