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
 * \file RasterSpatialIndexation.cpp
 * \brief
 * \author Adrien Chauve
 * \date 13 nov. 2008
 */

#include <iostream>
#include <stdexcept>

#include "RasterSpatialIndexation.h"

namespace Lidar
{

unsigned int RasterSpatialIndexation::m_nbPointsParM2 = 10;

void RasterSpatialIndexation::getApproximateRectangularNeighborhood( NeighborhoodListeType &list, const TPoint2D<float> &p1, const TPoint2D<float> &p2) const
{
	list.clear();

	//Récupération des pixels à visiter
	int colonne1, ligne1, colonne2, ligne2;
	m_ori.MapToImage( p1.x, p1.y, colonne1, ligne1 );
	m_ori.MapToImage( p2.x, p2.y, colonne2, ligne2 );


	const int colMin = std::max( 0, std::min(colonne1, colonne2) );
	const int colMax = std::min( m_griddedData.GetTaille().x - 1, std::max(colonne1, colonne2) );
	const int ligMin = std::max( 0, std::min(ligne1, ligne2) );
	const int ligMax = std::min( m_griddedData.GetTaille().y - 1, std::max(ligne1, ligne2) );

	const unsigned int evalNbPoints = (unsigned int)( (colMax-colMin+1)*(ligMax-ligMin+1)*m_resolution*m_nbPointsParM2 );
	list.reserve(evalNbPoints);

	for (int col = colMin; col <= colMax; ++col)
	{
		for (int lig = ligMin; lig <= ligMax; ++lig)
		{
			std::copy( m_griddedData( col,lig).begin(), m_griddedData( col,lig ).end(), std::back_inserter(list));
		}
	}
}

void RasterSpatialIndexation::indexData()
{
	if(m_resolution == 0)
		throw std::logic_error("Erreur dans RasterSpatialIndexation::indexData : la résolution n'a pas été choisie !\n");

	if(m_bboxMin == TPoint2D<float>(0,0) &&  m_bboxMax == TPoint2D<float>(0,0))
		findBBox();

	allocateData();

	fillData();
}

void RasterSpatialIndexation::allocateData()
{
	std::cout << "RasterSpatialIndexation : allocateData : \n";
	std::cout << "bboxMin=" << m_bboxMin << " , bboxMax=" << m_bboxMax << std::endl;

	//boundingBox de l'image :
	float x0min = m_resolution * std::floor( m_bboxMin.x / m_resolution );
	float y0min = m_resolution * std::floor( m_bboxMin.y / m_resolution );
	float x0max = m_resolution * std::ceil( m_bboxMax.x / m_resolution );
	float y0max = m_resolution * std::ceil( m_bboxMax.y / m_resolution );

	//origine de la géométrie ortho dallée = (x0min, y0max)
	// x positif vers l'est
	// y positif vers le nord != géométrie image

	//dimensions:


	int tailleX = static_cast< int > ( (x0max - x0min) / m_resolution);
	int tailleY = static_cast< int > ( (y0max - y0min) / m_resolution);

	//allocation du tableau :
	m_griddedData = GriddedDataType( tailleX, tailleY );

	//Ori de la géométrie :
	std::cout << "x0min=" << x0min << " , y0max=" << y0max << " , tailleX=" << tailleX << " , tailleY=" << tailleY << std::endl;
	m_ori = Orientation2D( x0min, y0max, m_resolution, 0, tailleX, tailleY );
}

void RasterSpatialIndexation::setResolution(const float resolution)
{
	m_resolution = resolution;
}

void RasterSpatialIndexation::setBBox(const TPoint2D<float> &bboxMin, const TPoint2D<float> &bboxMax)
{
	m_bboxMin = bboxMin;
	m_bboxMax = bboxMax;
}


RasterSpatialIndexation::RasterSpatialIndexation(): //const XYZFunctionType& funcX, const XYZFunctionType& funcY, const XYZFunctionType& funcZ):
	m_resolution(0), m_bboxMin(0,0), m_bboxMax(0,0)
//	m_funcX(funcX), m_funcY(funcY), m_funcZ(funcZ)
{

}

RasterSpatialIndexation::~RasterSpatialIndexation()
{
}


}//namespace Lidar
