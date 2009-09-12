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


#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/geometry/LidarSpatialIndexation2D.h"

#include "RegionOfInterest2D.h"

using namespace Lidar;


shared_ptr<LidarDataContainer> RegionOfInterest2D::cropLidarData(const LidarDataContainer& lidarContainer, const LidarSpatialIndexation2D& spatialIndexation, const LidarCenteringTransfo& transfo) const
{
	//Creation du nouveau container et ajout des attributs
	shared_ptr<LidarDataContainer> resultContainer = shared_ptr<LidarDataContainer>(new LidarDataContainer);
	const AttributeMapType& attributeMap = lidarContainer.getAttributeMap();

	for(AttributeMapType::const_iterator it = attributeMap.begin(); it != attributeMap.end(); ++it)
	{
		resultContainer->addAttribute(it->first, it->second.type);
	}

//	std::cout << "\tNouveau container OK..." << std::endl;


	//Recuperation des indices des points contenus dans la region croppee
	LidarSpatialIndexation2D::NeighborhoodListeType listeIndices;
	getListNeighborhood(listeIndices, spatialIndexation, transfo);

//	std::cout << "\tRécupération de la région d'intérêt OK..." << std::endl;

	//Recopie des points d'interet dans le nouveau container
	for(RasterSpatialIndexation::NeighborhoodListeType::iterator it = listeIndices.begin(); it != listeIndices.end(); ++it)
	{
		resultContainer->push_back(lidarContainer.rawData(*it));
	}

//	std::cout << "\tNouveau container rempli taille=" << resultContainer->size() << "  OK..." << std::endl;

	return resultContainer;
}


RegionOfInterest2D::RegionOfInterest2D()
{
}

RegionOfInterest2D::~RegionOfInterest2D()
{
}



PointRegionOfInterest2D::PointRegionOfInterest2D(const TPoint2D<double>& pt):
	m_pt(pt)
{

}

PointRegionOfInterest2D::~PointRegionOfInterest2D()
{
}

void PointRegionOfInterest2D::getListNeighborhood(RasterSpatialIndexation::NeighborhoodListeType& listeVoisins, const RasterSpatialIndexation& spatialIndexation, const Lidar::LidarCenteringTransfo& transfo) const
{
	spatialIndexation.getApproximateRectangularNeighborhood(listeVoisins, transfo.applyTransfoInverse(m_pt), transfo.applyTransfoInverse(m_pt));
}







CircularRegionOfInterest2D::CircularRegionOfInterest2D(const TPoint2D<double>& centre, const float radius):
	m_ptCentre(centre), m_radius(radius)
{

}

CircularRegionOfInterest2D::~CircularRegionOfInterest2D()
{
}

void CircularRegionOfInterest2D::getListNeighborhood(RasterSpatialIndexation::NeighborhoodListeType& listeVoisins, const RasterSpatialIndexation& spatialIndexation, const Lidar::LidarCenteringTransfo& transfo) const
{
	spatialIndexation.GetCenteredNeighborhood(listeVoisins, transfo.applyTransfoInverse(m_ptCentre), m_radius, Neighborhoods::CylindricalNeighborhood(transfo.applyTransfoInverse(m_ptCentre), m_radius));
}





RectangularRegionOfInterest2D::RectangularRegionOfInterest2D(const TPoint2D<double>& pt1, const TPoint2D<double>& pt2):
	m_pt1(pt1), m_pt2(pt2)
{

}

RectangularRegionOfInterest2D::~RectangularRegionOfInterest2D()
{
}

void RectangularRegionOfInterest2D::getListNeighborhood(RasterSpatialIndexation::NeighborhoodListeType& listeVoisins, const RasterSpatialIndexation& spatialIndexation, const Lidar::LidarCenteringTransfo& transfo) const
{
	spatialIndexation.getApproximateRectangularNeighborhood(listeVoisins, transfo.applyTransfoInverse(m_pt1), transfo.applyTransfoInverse(m_pt2));
}


