# Find BOOST
# CMake does not include boost version 1.39
FIND_PACKAGE( Boost 1.36 COMPONENTS python REQUIRED)
if( Boost_FOUND )
	INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
	LINK_DIRECTORIES( ${Boost_LIBRARY_DIRS} )
	# Autolink under Windows platforms
	if( NOT WIN32 )
		SET(LidarFormat_LIBRAIRIES ${LidarFormat_LIBRAIRIES} ${Boost_PYTHON_LIBRARY})
	endif()
else()
	message( FATAL_ERROR "Boost python not found ! Please set Boost path ..." )
endif()

FIND_PACKAGE(PythonLibs REQUIRED)
IF( PYTHONLIBS_FOUND )
	INCLUDE_DIRECTORIES( ${PYTHON_INCLUDE_PATH} )
	SET(LidarFormat_LIBRAIRIES ${LidarFormat_LIBRAIRIES} ${PYTHON_LIBRARIES})
ELSE()
	MESSAGE( FATAL_ERROR "PythonLibs not found ! Please set PythonLibs path ..." )
ENDIF()


AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat/python  SRC_PYTHON)