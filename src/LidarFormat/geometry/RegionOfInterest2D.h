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
