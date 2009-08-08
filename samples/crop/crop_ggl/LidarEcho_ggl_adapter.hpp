/*
 * LidarEcho_ggl_adapter.hpp
 *
 *  Created on: 24 juil. 2009
 *      Author: ndavid
 */

#ifndef LIDARECHO_GGL_ADAPTER_HPP_
#define LIDARECHO_GGL_ADAPTER_HPP_

#include "extern/matis/tpoint3d.h"
#include <boost/shared_ptr.hpp>
#include "ggl/ggl.hpp"
#include "ggl/geometries/cartesian2d.hpp"
#include "ggl/algorithms/within.hpp"
#include "LidarFormat/LidarDataContainer.h"

using namespace Lidar;
using namespace std;

namespace ggl
{
        namespace traits
        {
                template <int I, typename T> struct accessor;

                template < typename T> struct accessor<0, T>
                {
						typedef TPoint3D<T> PointType;
                        inline static double get(const PointType& p) { return p.x; }
                        inline static void set(PointType& p, const T& value) { p.x = value; }
                };

                template < typename T> struct accessor<1, T>
                {
						typedef TPoint3D<T> PointType;
                        inline static double get(const PointType& p) { return p.y; }
                        inline static void set(PointType& p, const T& value) { p.y = value; }
                };

                template < typename T> struct accessor<2, T>
                {
                	typedef TPoint3D<T> PointType;
                       inline static double get(const PointType& p) { return p.z; }
                       inline static void set(PointType& p, const T& value) { p.z = value; }
                };
                // For legacy points, define the necessary structs coordinate (with typedef),
                // dimension (with value) and access (with get function).
                // Be sure to define them within the namespace geometry::traits
                template <typename T> struct tag<TPoint3D<T> > { typedef point_tag type; };
                template <typename T> struct coordinate_type<TPoint3D<T> > { typedef  T type; };
                template <typename T> struct coordinate_system<TPoint3D<T> > { typedef cs::cartesian type; };
                template <typename T> struct dimension<TPoint3D<T> >: boost::mpl::int_<3> {};
                template <typename T> struct access<TPoint3D<T> >
                {
						typedef TPoint3D<T> PointType;
                        template <int I>
                        static double get(const PointType& p)
                        { return accessor<I,T>::get(p); }

                        template <int I>
                        static void set(PointType& p, const T& value)
                        { accessor<I,T>::set(p, value); }
                };
        }
}

using namespace ggl;

template<typename T>
void add_crop(polygon_2d poly, boost::shared_ptr<LidarDataContainer> m_lidarInput,boost::shared_ptr<LidarDataContainer> m_lidarOutput)
{
	typedef TPoint3D<T> PointType;
	typedef LidarIteratorXYZ<T> ite_xyz;
	ite_xyz ite_input= m_lidarInput->beginXYZ<T>();
	LidarIteratorEcho ite_echo_input=m_lidarInput->begin();

	for(ite_input ; ite_input!=m_lidarInput->endXYZ<T>(); ite_input++ )
	{
		//PointType pt_tmp2=*ite_input;
		PointType pt_tmp(ite_input.x(),ite_input.y(),ite_input.z());
		//pt_tmp=*ite_input;
		//std::cout<<" test point x,y,z "<<pt_tmp<<" ref "<<pt_tmp2<<std::endl;
		if( ggl::within(pt_tmp, poly) )
		{
			//std::cout<<" OK"<<std::endl;
			m_lidarOutput->push_back(*ite_echo_input);
		}
		else
		 {
			//std::cout<<" FALSE"<<std::endl;
		}
		ite_echo_input++;
	}
}

#endif /* LIDARECHO_GGL_ADAPTER_HPP_ */
