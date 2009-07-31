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
 * \file RegionOfInterest2D.h
 * \brief
 * \author Adrien Chauve
 * \date 27 janv. 2009
 */

#ifndef REGIONOFINTEREST2D_H_
#define REGIONOFINTEREST2D_H_

#include <boost/shared_ptr.hpp>

#include "LidarFormat/extern/matis/tpoint2d.h"

#include "LidarFormat/geometry/LidarCenteringTransfo.h"
#include "LidarFormat/geometry/RasterSpatialIndexation.h"

using boost::shared_ptr;

namespace Lidar
{
	class LidarDataContainer;
	class LidarSpatialIndexation2D;
}

class RegionOfInterest2D
{
	public:
		RegionOfInterest2D();
		virtual ~RegionOfInterest2D();

		virtual void getListNeighborhood(Lidar::RasterSpatialIndexation::NeighborhoodListeType& listeVoisins, const Lidar::RasterSpatialIndexation& spatialIndexation, const Lidar::LidarCenteringTransfo& transfo = Lidar::LidarCenteringTransfo()) const=0;
		virtual shared_ptr<Lidar::LidarDataContainer> cropLidarData(const Lidar::LidarDataContainer& lidarContainer, const Lidar::LidarSpatialIndexation2D& spatialIndexation, const Lidar::LidarCenteringTransfo& transfo = Lidar::LidarCenteringTransfo()) const;
};



class PointRegionOfInterest2D : public RegionOfInterest2D
{
	public:
		PointRegionOfInterest2D(const TPoint2D<double>& pt);
		virtual ~PointRegionOfInterest2D();

		virtual void getListNeighborhood(Lidar::RasterSpatialIndexation::NeighborhoodListeType& listeVoisins, const Lidar::RasterSpatialIndexation& spatialIndexation, const Lidar::LidarCenteringTransfo& transfo = Lidar::LidarCenteringTransfo()) const;

	private:
		TPoint2D<double> m_pt;
};

class CircularRegionOfInterest2D : public RegionOfInterest2D
{
	public:
		CircularRegionOfInterest2D(const TPoint2D<double>& centre, const float radius);
		virtual ~CircularRegionOfInterest2D();

		virtual void getListNeighborhood(Lidar::RasterSpatialIndexation::NeighborhoodListeType& listeVoisins, const Lidar::RasterSpatialIndexation& spatialIndexation, const Lidar::LidarCenteringTransfo& transfo = Lidar::LidarCenteringTransfo()) const;

	private:
		TPoint2D<double> m_ptCentre;
		float m_radius;
};


class RectangularRegionOfInterest2D : public RegionOfInterest2D
{
	public:
		RectangularRegionOfInterest2D(const TPoint2D<double>& pt1, const TPoint2D<double>& pt2);
		virtual ~RectangularRegionOfInterest2D();

		virtual void getListNeighborhood(Lidar::RasterSpatialIndexation::NeighborhoodListeType& listeVoisins, const Lidar::RasterSpatialIndexation& spatialIndexation, const Lidar::LidarCenteringTransfo& transfo = Lidar::LidarCenteringTransfo()) const;

	private:
		TPoint2D<double> m_pt1, m_pt2;
};

#endif /* REGIONOFINTEREST2D_H_ */
