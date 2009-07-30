#################################################
###           INSTALL 						  ###
#################################################
#get_filename_component(LIDARFORMAT_CMAKE_SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

#IF(UNIX)
#	SET(INSTALL_PREFIX "/usr/local" CACHE PATH " install path" )
#ENDIF(UNIX)
#IF(WIN32)
#	SET(LidarFormat_INSTALL_DIR "LidarFormat")
#	SET(INSTALL_PREFIX "C:/Program Files/MATIS" CACHE PATH " install path")
#ENDIF(WIN32)
#SET(CMAKE_INSTALL_PREFIX ${INSTALL_PREFIX} CACHE INTERNAL  " real install path" FORCE)

IF(UNIX)
    #SET(CMAKE_INSTALL_SO_NO_EXE "0")
	INSTALL (FILES ${ALL_LIDAR_FORMAT_HEADER_FILES} DESTINATION include/LidarFormat COMPONENT headers)
	INSTALL( FILES ${ALL_MODELS_HEADER_FILES} DESTINATION include/LidarFormat/models COMPONENT headers)
	INSTALL( FILES ${ALL_GEOMETRY_HEADER_FILES} DESTINATION include/LidarFormat/geometry COMPONENT headers)
	INSTALL( FILES ${ALL_TOOLS_HEADER_FILES} DESTINATION include/LidarFormat/tools COMPONENT headers)
	INSTALL( FILES ${ALL_EXTERN_HEADER_FILES} DESTINATION include/LidarFormat/extern/matis COMPONENT headers)
	#SET(RELATIVE_INCLUDE_PATH "../../include/LidarFormat")
	#SET(RELATIVE_INCLUDE_EXTERN_PATH "../../include/extern")
	#CONFIGURE_FILE(${LIDARFORMAT_CMAKE_SELF_DIR}/LidarFormatConfig.cmake.in ${CMAKE_CURRENT_SOURCE_DIR}/LidarFormatConfig.cmake @ONLY)
	
	INSTALL ( TARGETS LidarFormat DESTINATION lib COMPONENT library )
			 
	#INSTALL (	EXPORT LidarFormat-targets 
	#			DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/LidarFormat
	#			
	#		)
	#INSTALL (FILES LidarFormatConfig.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/LidarFormat)
	
	
	 set(CPACK_PACKAGE_NAME "LidarFormat")
	 set(CPACK_PACKAGE_VENDOR "IGN/CEMAGREF")
	 set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "LidarFormat is an open source library for efficiently handling 3D point clouds with a variable number of attributes at runtime.")
	 set(CPACK_PACKAGE_VERSION "0.1")
	 set(CPACK_PACKAGE_VERSION_MAJOR "0")
	 set(CPACK_PACKAGE_VERSION_MINOR "1")
	 set(CPACK_PACKAGE_VERSION_PATCH "0")
	 set(CPACK_GENERATOR "DEB")
	 set(CPACK_PACKAGE_CONTACT "adrien.chauve@ign.fr")
	 set(CPACK_PACKAGE_INSTALL_DIRECTORY "CPack Component Example")
	 set( CPACK_COMPONENTS_ALL headers library )
	 
	 
	 include(CPack)
	 
	
ENDIF(UNIX)

