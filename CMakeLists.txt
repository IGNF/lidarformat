CMAKE_MINIMUM_REQUIRED( VERSION 2.6 )

#Nom du projet
PROJECT(LidarFormat)

# Useful for Eclipse code STL completion
if(UNIX)
	exec_program( gcc ARGS -dumpversion OUTPUT_VARIABLE GCC_VERSION )
	
	string( SUBSTRING ${GCC_VERSION} 0 3 GOOD_GCC_VERSION )
    include_directories( /usr/include/c++/${GOOD_GCC_VERSION} )
endif()

####
#### On recherche tous les fichiers ".h", ".hpp", ".hxx" et "*.inl"
####
FILE( GLOB ALL_LIDAR_FORMAT_HEADER_FILES src/LidarFormat/*.h )
FILE( GLOB ALL_GEOMETRY_HEADER_FILES src/LidarFormat/geometry/*.h )
FILE( GLOB ALL_TOOLS_HEADER_FILES src/LidarFormat/tools/*.h src/LidarFormat/tools/*.hpp)
FILE( GLOB ALL_MODELS_HEADER_FILES src/LidarFormat/models/*.h src/LidarFormat/models/*.hxx)
FILE( GLOB ALL_EXTERN_HEADER_FILES src/LidarFormat/extern/matis/*.h)


#Génération du code de parse des fichiers xml (gestion du format lidar)
add_custom_command(
    OUTPUT  ${CMAKE_SOURCE_DIR}/src/LidarFormat/models/format_me.cxx ${CMAKE_SOURCE_DIR}/src/LidarFormat/models/format_me.hxx ${CMAKE_SOURCE_DIR}/src/LidarFormat/models/format_me-fwd.hxx
    COMMAND xsd cxx-tree --function-naming lcc --type-naming ucc --generate-polymorphic --generate-comparison --generate-doxygen --generate-serialization --generate-forward --output-dir ${CMAKE_SOURCE_DIR}/src/LidarFormat/models ${CMAKE_SOURCE_DIR}/src/LidarFormat/models/xsd/format_me.xsd
	DEPENDS ${CMAKE_SOURCE_DIR}/src/LidarFormat/models/xsd/format_me.xsd
)

#### Definition des dossiers contenant les sources
SET(SRC_DIR src/)
INCLUDE_DIRECTORIES(${SRC_DIR})

AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat SRC_LIDAR_FORMAT)
AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat/geometry  SRC_GEOMETRY)
AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat/tools  SRC_TOOLS)
AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat/file_formats  SRC_FILE_FORMATS)
AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat/file_formats/standard  SRC_FILE_FORMATS_STANDARD)
AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat/extern/matis  SRC_EXTERN_MATIS)
AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat/extern/terrabin  SRC_EXTERN_TERRABIN)

SET(SRC_MODELS_DIR ${SRC_DIR}/LidarFormat/models)
SET(SRC_MODELS  ${SRC_MODELS_DIR}/format_me.cxx)

# Une variable regroupant toutes les sources ...
SET( ALL_SOURCES 
        ${SRC_LIDAR_FORMAT} 
        ${SRC_MODELS} 
        ${SRC_GEOMETRY} 
        ${SRC_TOOLS} 
        ${SRC_FILE_FORMATS}
        ${SRC_FILE_FORMATS_STANDARD}
        ${SRC_EXTERN_MATIS} 
        ${SRC_EXTERN_TERRABIN}
   )

SET( ALL_HEADERS ${ALL_LIDAR_FORMAT_HEADER_FILES} ${ALL_MODELS_HEADER_FILES} ${ALL_GEOMETRY_HEADER_FILES})

####
#### Recherche des libs
####	
#Pour utiliser le find xerces et xsd:
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmakeContrib)
FIND_PACKAGE(XercesC)
IF( XERCESC_FOUND )
	INCLUDE_DIRECTORIES( ${XERCESC_INCLUDE_DIR} )
	SET(LidarFormat_LIBRAIRIES ${LidarFormat_LIBRAIRIES} ${XERCESC_LIBRARIES})
ELSE()
	MESSAGE( FATAL_ERROR "XercesC not found ! Please set XercesC path ..." )
ENDIF()

FIND_PACKAGE(Xsd)
IF( XSD_FOUND )
	INCLUDE_DIRECTORIES( ${XSD_INCLUDE_DIR} )
	#MESSAGE("XSD Include Path : " ${XSD_INCLUDE_DIR} )
ELSE()
	MESSAGE( FATAL_ERROR "XSD not found ! Please set XSD path ..." )
ENDIF()

# Find BOOST
# CMake does not include boost version 1.39
set(Boost_ADDITIONAL_VERSIONS "1.39.0" "1.39")
find_package( Boost 1.36 COMPONENTS filesystem system unit_test_framework)
if( Boost_FOUND )
	include_directories( ${Boost_INCLUDE_DIR} )
	link_directories( ${Boost_LIBRARY_DIRS} )
	# Autolink under Windows platforms
	if( NOT WIN32 )
		set(LidarFormat_LIBRAIRIES ${LidarFormat_LIBRAIRIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
	endif()
else()
	message( FATAL_ERROR "Boost not found ! Please set Boost path ..." )
endif()


####
#### Use LAS format
####
OPTION( ENABLE_LAS "Enable LAS" OFF )
if(ENABLE_LAS)
    AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat/file_formats/LAS  SRC_LAS)
    SET( ALL_SOURCES ${ALL_SOURCES} ${SRC_LAS})
    
    SET( LAS_DIRECTORY "" CACHE STRING "LibLAS include directory" )
    SET( LAS_LIBRARY "" CACHE STRING "LibLAS library" )
    INCLUDE_DIRECTORIES(${LAS_DIRECTORY})
    
    SET(LidarFormat_LIBRAIRIES ${LidarFormat_LIBRAIRIES} ${LAS_LIBRARY})
endif(ENABLE_LAS)


####
#### Use TerraBin format
####
OPTION( ENABLE_TERRABIN "Enable TerraBin" OFF )
if(ENABLE_TERRABIN)
    AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat/file_formats/TerraBin  SRC_TERRABIN)
    SET( ALL_SOURCES ${ALL_SOURCES} ${SRC_TERRABIN})
endif(ENABLE_TERRABIN)


####
#### Use PlyArchi format
####
OPTION( ENABLE_PLYARCHI "Enable PlyArchi" OFF )
if(ENABLE_PLYARCHI)
    AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat/file_formats/PlyArchi  SRC_PLYARCHI)
    SET( ALL_SOURCES ${ALL_SOURCES} ${SRC_PLYARCHI})
endif(ENABLE_PLYARCHI)


####
#### Construction de la librarie
####
# Options pour construire une librarie (et la choisir en .a ou .so)
SET( LIBRARY_TYPE STATIC )
# Verification du type de bibliotheque a construire
OPTION( BUILD_LIBRARY_SHARED "Build LidarFormat as a SHARED library." ON )
IF( BUILD_LIBRARY_SHARED )
	SET( LIBRARY_TYPE SHARED )
ELSE()
	SET( LIBRARY_TYPE STATIC )
ENDIF()

if(UNIX)
	ADD_DEFINITIONS(-Wall)
else()
	ADD_DEFINITIONS( -W3 )
	# Visual warnings on checked iterators
	ADD_DEFINITIONS( -D_SCL_SECURE_NO_WARNINGS )
	ADD_DEFINITIONS( -D_CRT_SECURE_NO_WARNINGS )
endif()

ADD_LIBRARY( LidarFormat  ${LIBRARY_TYPE} ${ALL_SOURCES} ${ALL_HEADERS}  )


TARGET_LINK_LIBRARIES( LidarFormat ${LidarFormat_LIBRAIRIES} )

####
#### Construction du sample
####
#INCLUDE( LidarFormat_build_sample.cmake )
		

####
#### Tests
####
OPTION( BUILD_TESTS "Build tests" ON )
if(BUILD_TESTS)
    INCLUDE( LidarFormat_tests.cmake )
endif(BUILD_TESTS)

####
#### Install
####
OPTION( BUILD_INSTALL "Generate install target" ON)
IF(BUILD_INSTALL)
	INCLUDE(LidarFormat_install.cmake )
ENDIF(BUILD_INSTALL)
