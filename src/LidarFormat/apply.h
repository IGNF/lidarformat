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

#if !BOOST_PP_IS_ITERATING

#ifndef APPLY_H_
#define APPLY_H_

#include <boost/preprocessor/repetition/enum_trailing_params.hpp>
#include <boost/preprocessor/repetition/enum_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/repetition/enum_shifted_params.hpp>
#include <boost/preprocessor/comma_if.hpp>
#include <boost/preprocessor/inc.hpp>

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <boost/preprocessor/seq/for_each.hpp>

#include "LidarFormat/models/format_me.hxx"

#include <boost/preprocessor/iteration/iterate.hpp>

#define BOOST_PP_ITERATION_PARAMS_1 (3, (0, 9, "LidarFormat/apply.h"))
#include BOOST_PP_ITERATE()

#endif /* APPLY_H_ */


#elif BOOST_PP_ITERATION_DEPTH() == 1

	namespace Lidar
	{

		#define N BOOST_PP_ITERATION()
		#define TYPES (int8)(uint8)(int16)(uint16)(int32)(uint32)(int64)(uint64)(float32)(float64)


		#define SWITCH_GENERATION(r, data, TYPE)\
			case LidarDataType::TYPE:\
				result = boost::bind(&TFunctor<LidarDataType::TYPE>::operator(), TFunctor<LidarDataType::TYPE>() BOOST_PP_COMMA_IF(N) BOOST_PP_ENUM_SHIFTED_PARAMS_Z(1, BOOST_PP_INC(N), _) );\
				break;


		template
		<
			template <EnumLidarDataType> class TFunctor,
			typename R
			BOOST_PP_ENUM_TRAILING_PARAMS_Z(1, N, typename A)
		>
		R apply(const EnumLidarDataType switchedVariable BOOST_PP_COMMA_IF(N) BOOST_PP_ENUM_BINARY_PARAMS_Z(1, N, A, a))
		{

			boost::function<R( BOOST_PP_ENUM_PARAMS_Z(1, N, A) )> result;
			switch(switchedVariable)
			{
				BOOST_PP_SEQ_FOR_EACH(SWITCH_GENERATION, N, TYPES)
			}

			return result(BOOST_PP_ENUM_PARAMS_Z(1, N, a));

		}


		#undef TYPES
		#undef N
		#undef SWITCH_GENERATION_NEW


	}



#endif


