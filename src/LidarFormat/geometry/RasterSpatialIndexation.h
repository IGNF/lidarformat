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

/*!
 * \file RasterSpatialIndexation.h
 * \brief
 * \author Adrien Chauve
 * \date 13 nov. 2008
 */

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

#include "extern/matis/ttableau2d.h"
#include "extern/matis/tpoint2d.h"
#include "extern/matis/tpoint3d.h"


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
