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
 * \file LidarCenteringTransfo.h
 * \brief
 * \author Adrien Chauve
 * \date 5 févr. 2009
 */

#ifndef LIDARCENTERINGTRANSFO_H_
#define LIDARCENTERINGTRANSFO_H_

#include <boost/shared_ptr.hpp>

#include "LidarFormat/extern/matis/tpoint2d.h"
#include "LidarFormat/extern/matis/tpoint3d.h"


namespace Lidar
{

using boost::shared_ptr;

class LidarDataContainer;
class Orientation2D;

class LidarCenteringTransfo
{
	public:
		LidarCenteringTransfo();

		void setTransfo(const double x, const double y);
		void setTransfo(const TPoint2D<double>& transfo) { m_x = transfo.x; m_y = transfo.y; }
		const double x() const { return m_x; }
		const double y() const { return m_y; }
		const TPoint2D<double> getTransfo() const { return TPoint2D<double>(m_x, m_y); }

		shared_ptr<LidarDataContainer> centerLidarDataContainer(const LidarDataContainer& lidarContainer, const char* const x="x", const char* const y="y", const char* const z="z");

		bool isSet() const { return m_x!=0 && m_y!=0; }


		//application de la transfo
		void applyTransfoOrientation(Orientation2D& ori) const;
		void applyTransfoInverseOrientation(Orientation2D& ori) const;

		template<typename T>
		const TPoint2D<double> applyTransfo(const TPoint2D<T>& pt) const
		{
			return TPoint2D<double>(pt.x + m_x, pt.y + m_y);
		}

		template<typename T>
		const TPoint2D<double> applyTransfoInverse(const TPoint2D<T>& pt) const
		{
			return TPoint2D<double>(pt.x - m_x, pt.y - m_y);
		}

		template<typename T>
		const TPoint3D<double> applyTransfo(const TPoint3D<T>& pt) const
		{
			return TPoint3D<double>(pt.x + m_x, pt.y + m_y, pt.z);
		}

		template<typename T>
		const TPoint3D<double> applyTransfoInverse(const TPoint3D<T>& pt) const
		{
			return TPoint3D<double>(pt.x - m_x, pt.y - m_y, pt.z);
		}

	private:
		double m_x, m_y;
};

} //namespace Lidar

#endif /* LIDARCENTERINGTRANSFO_H_ */
