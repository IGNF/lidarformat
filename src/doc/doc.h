/*! \mainpage LidarFormat Documentation
 *
 *
 * \par
 *
 *
 * LidarFormat is an open source library for efficiently handling 3D point
 * clouds with a variable number of attributes at runtime. LidarFormat is
 * distributed under the CeCILL-B licence. See Licence_CeCILL-B_V1-en.txt
 * or http://www.cecill.info for more details.
 *
 * \section intro_sec Introduction
 *
 * Homepage: https://fullanalyze.ign.fr/trac/LidarFormat
 *
 * Copyright: Institut Geographique National & CEMAGREF (2009)
 *
 * Author: Adrien Chauve
 *
 *
 *
 * \section install_sec Installation instructions
 *
 * \subsection dependencies Dependencies
 *
 * 		NB: On linux plateforms, you should use the official distribution packages.
 *
 *		Install the following tools or libraries:
 *			-CMake
 *			-Boost (>= 1.36)
 *			-Xerces
 *			-Code Synthesis XSD (can be downloaded at http://www.codesynthesis.com/download)
 *
 * \subsection compilation	Compilation:
 *
 *		On Linux platforms, enter in a terminal (in the LidarFormat/trunk directory):
 *
 *			\code
 *			ccmake .
 *			--> follow the CMake instructions, in particular:
 *			--> launch the configure step with 'c'
 *			--> set CMAKE_BUILD_TYPE to Release within CMake
 *			--> generate the configuration with 'g'
 *			make
 *			make test
 *			--> check all tests are passed!
 *			\endcode
 *
 *		** Make sure you compiled LidarFormat as a shared library, otherwise you will need
 *		to register the file formats in your code before using the library.
 *
 */



/** \defgroup group_iterators Iterators
 *
 *  There are 3 types of iterators in LidarFormat:
 *
 *  - LidarIteratorEcho / LidarConstIteratorEcho
 *  - LidarIteratorAttribute / LidarConstIteratorAttribute
 *  - LidarIteratorXYZ / LidarConstIteratorXYZ
 *
 */

