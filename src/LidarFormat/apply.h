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
 * \file apply.h
 * \brief
 * \author Adrien Chauve
 * \date 10 févr. 2009
 */

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


