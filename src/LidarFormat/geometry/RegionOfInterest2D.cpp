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
 * \file RegionOfInterest2D.cpp
 * \brief
 * \author Adrien Chauve
 * \date 27 janv. 2009
 */

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


