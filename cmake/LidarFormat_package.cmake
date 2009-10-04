#################################################
###           Package 						  ###
#################################################

IF(UNIX)
	INSTALL (FILES ${ALL_LIDAR_FORMAT_HEADER_FILES} DESTINATION include/LidarFormat COMPONENT headers)
	INSTALL( FILES ${ALL_MODELS_HEADER_FILES} DESTINATION include/LidarFormat/models COMPONENT headers)
	INSTALL( FILES ${ALL_GEOMETRY_HEADER_FILES} DESTINATION include/LidarFormat/geometry COMPONENT headers)
	INSTALL( FILES ${ALL_TOOLS_HEADER_FILES} DESTINATION include/LidarFormat/tools COMPONENT headers)
	INSTALL( FILES ${ALL_EXTERN_HEADER_FILES} DESTINATION include/LidarFormat/extern/matis COMPONENT headers)
	
	INSTALL ( TARGETS LidarFormat DESTINATION lib COMPONENT library )
	
	 SET(CPACK_GENERATOR "DEB")
	 
	 #dpkg-shlibdeps libLidarFormat.so
	 SET(CPACK_DEBIAN_PACKAGE_DEPENDS
	         "libboost-filesystem1.37.0 (>= 1.37.0-1), libboost-system1.37.0 (>= 1.37.0-1), libc6 (>= 2.4), libgcc1 (>= 1:4.1.1), libstdc++6 (>= 4.2.1), libxerces-c28"
	     )	 
	 
ENDIF(UNIX)



IF(WIN32)
	SET(LidarFormat_INSTALL_DIR "LidarFormat")
	SET(INSTALL_PREFIX "C:/Program Files/LidarFormat" CACHE PATH " install path")
	INSTALL (FILES ${ALL_LIDAR_FORMAT_HEADER_FILES} DESTINATION include/LidarFormat COMPONENT headers)
	INSTALL( FILES ${ALL_MODELS_HEADER_FILES} DESTINATION include/LidarFormat/models COMPONENT headers)
	INSTALL( FILES ${ALL_GEOMETRY_HEADER_FILES} DESTINATION include/LidarFormat/geometry COMPONENT headers)
	INSTALL( FILES ${ALL_TOOLS_HEADER_FILES} DESTINATION include/LidarFormat/tools COMPONENT headers)
	INSTALL( FILES ${ALL_EXTERN_HEADER_FILES} DESTINATION include/LidarFormat/extern/matis COMPONENT headers)
	
	INSTALL ( TARGETS LidarFormat DESTINATION lib COMPONENT library)	
	
	 SET(CPACK_GENERATOR "NSIS")

ENDIF(WIN32)



set(CPACK_PACKAGE_NAME "LidarFormat")
set(CPACK_PACKAGE_VENDOR "IGN/CEMAGREF")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "LidarFormat is an open source library for efficiently handling 3D point clouds with a variable number of attributes at runtime.")
set(CPACK_PACKAGE_VERSION "0.1")
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "1")
set(CPACK_PACKAGE_VERSION_PATCH "0")

set(CPACK_PACKAGE_CONTACT "LidarFormat Dev <lidardev@lists.launchpad.net>")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "LidarFormat")
 
set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_SOURCE_DIR}/LICENSE )
set(CPACK_RESOURCE_FILE_README ${CMAKE_SOURCE_DIR}/README )
 
# Descriptive components names
set( CPACK_COMPONENT_LIBRARY_DISPLAY_NAME "Library" )
set( CPACK_COMPONENT_HEADERS_DISPLAY_NAME "C++ Headers" )
 
# Descriptive components texts
set( CPACK_COMPONENT_LIBRARY_DESCRIPTION "Library used to build applications with LidarFormat" )
set( CPACK_COMPONENT_HEADERS_DESCRIPTION "C++ headers files for use with LidarFormat" )
 
# Groups
set( CPACK_COMPONENT_LIBRARY_GROUP "Development" )
set( CPACK_COMPONENT_HEADERS_GROUP "Development" )

set(CPACK_COMPONENTS_ALL library headers)
include(CPack)
