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
        LidarCenteringTransfo(const double x, const double y):m_x(x),m_y(y){}
        LidarCenteringTransfo(const TPoint2D<double>& transfo):m_x(transfo.x),m_y(transfo.y){}

		void setTransfo(const double x, const double y);
		void setTransfo(const TPoint2D<double>& transfo) { m_x = transfo.x; m_y = transfo.y; }
		const double x() const { return m_x; }
		const double y() const { return m_y; }
		const TPoint2D<double> getTransfo() const { return TPoint2D<double>(m_x, m_y); }

        shared_ptr<LidarDataContainer> centerLidarDataContainer(const LidarDataContainer& lidarContainer,
                                                                const char* const x="x", const char* const y="y", const char* const z="z");

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
