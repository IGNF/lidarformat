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



#ifndef LIDARSPATIALINDEXATION2D_H_
#define LIDARSPATIALINDEXATION2D_H_


#include "LidarFormat/geometry/RasterSpatialIndexation.h"

namespace Lidar
{

class LidarDataContainer;
template<class T> class LidarIteratorXYZ;

/**
 * ATTENTION : implémentée que pour des float !
 */

class LidarSpatialIndexation2D : public RasterSpatialIndexation
{
	public:
		LidarSpatialIndexation2D(const LidarDataContainer& lidarContainer);
		virtual ~LidarSpatialIndexation2D();

		virtual void GetCenteredNeighborhood(NeighborhoodListeType &list, const TPoint2D<float> &centre, const float approxNeighborhoodSize, const NeighborhoodFunctionType IsInside = defaultIsInside) const;

	protected:
		virtual void findBBox();
		virtual void fillData();

		//reference data
		const LidarDataContainer& m_lidarContainer;


};


} //namespace Lidar


#endif /* LIDARSPATIALINDEXATION2D_H_ */
