CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
##################
### SEULEMENT LINUX POUR L'INSTANT..###
##################

#Nom du projet
PROJECT(CROP_GGL CXX C)
# SET classical path for the MATIS library
#
IF(UNIX)
	SET(CMAKE_PREFIX_PATH "/usr/local" ${CMAKE_PREFIX_PATH}  )
ENDIF(UNIX)
IF(WIN32)
	SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "C:/Program Files/MATIS" )
ENDIF(WIN32)



FIND_PACKAGE(LidarFormat REQUIRED)
IF(LidarFormat_FOUND)
	MESSAGE(STATUS "LidarFormat_FOUND found !")
	include_directories(${LidarFormat_INCLUDE_DIRS})
ELSE(LidarFormat_FOUND)
	# For convenience. inform the user
	MESSAGE(SEND_ERROR "LidarFormat not found ! check if you have still install this library
	and/or set the correct LidarFormatConfig path ")
ENDIF(LidarFormat_FOUND)

# Find BOOST
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON)
SET(Boost_ADDITIONAL_VERSIONS "1.37.0" "1.37" "1.38" "1.38.0" "1.39" "1.39.0")
FIND_PACKAGE( Boost 1.36 COMPONENTS filesystem system thread program_options)
IF( Boost_FOUND )
    INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
    #MESSAGE("Boost Include Path : " ${Boost_INCLUDE_DIR} )
ELSE()
	MESSAGE( FATAL_ERROR "Boost not found ! Please set Boost path ..." )
ENDIF()

SET(GGL_FOUND FALSE)
FIND_PATH(GGL_INCLUDE_DIR ggl.hpp PATH ${Boost_INCLUDE_DIR}/boost/ggl)
IF(GGL_INCLUDE_DIR)
    INCLUDE_DIRECTORIES( ${GGL_INCLUDE_DIR}/../ )
    SET(GGL_FOUND TRUE)
ELSE(GGL_INCLUDE_DIR)
    MESSAGE( FATAL_ERROR "path of ggl.hpp not found ..." )
ENDIF(GGL_INCLUDE_DIR)

#################################################
### Executable								 ###
#################################################
SET(crop_ggl_SRC_DIR ${CMAKE_SOURCE_DIR})

IF(UNIX)
	ADD_EXECUTABLE(crop_ggl  ${crop_ggl_SRC_DIR}/crop_ggl.cpp ${crop_ggl_SRC_DIR}/LidarEcho_ggl_adapter.hpp)
ENDIF(UNIX)
IF(WIN32)
	ADD_DEFINITIONS(-DWIN32)
	ADD_EXECUTABLE(crop_ggl   ${crop_ggl_SRC_DIR}/crop_ggl.cpp ${crop_ggl_SRC_DIR}/LidarEcho_ggl_adapter.hpp)
ENDIF(WIN32)

TARGET_LINK_LIBRARIES(crop_ggl  LidarFormat  ${Boost_LIBRARIES})
