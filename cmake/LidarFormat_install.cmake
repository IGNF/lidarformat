#################################################
###           INSTALL 						  ###
#################################################
get_filename_component(LIDARFORMAT_CMAKE_SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

IF(UNIX)
	SET(INSTALL_PREFIX "/usr/local" CACHE PATH " install path" )
ENDIF(UNIX)
IF(WIN32)
	SET(LidarFormat_INSTALL_DIR "LidarFormat")
	SET(INSTALL_PREFIX "C:/Program Files/MATIS" CACHE PATH " install path")
ENDIF(WIN32)
SET(CMAKE_INSTALL_PREFIX ${INSTALL_PREFIX} CACHE INTERNAL  " real install path" FORCE)

IF(UNIX)
	#generate config file
	SET(RELATIVE_INCLUDE_PATH "../../include/LidarFormat")
	SET(RELATIVE_INCLUDE_EXTERN_PATH "../../include/extern")
	CONFIGURE_FILE(${LIDARFORMAT_CMAKE_SELF_DIR}/LidarFormatConfig.cmake.in ${CMAKE_CURRENT_SOURCE_DIR}/LidarFormatConfig.cmake @ONLY)
	#install include file
	INSTALL (FILES ${ALL_LIDAR_FORMAT_HEADER_FILES} DESTINATION include/LidarFormat COMPONENT headers)
	INSTALL (FILES ${ALL_FILE_FORMATS_HEADER_FILES} DESTINATION include/LidarFormat/file_formats COMPONENT headers)
	INSTALL( FILES ${ALL_MODELS_HEADER_FILES} DESTINATION include/LidarFormat/models COMPONENT headers)
	INSTALL( FILES ${ALL_GEOMETRY_HEADER_FILES} DESTINATION include/LidarFormat/geometry COMPONENT headers)
	INSTALL( FILES ${ALL_TOOLS_HEADER_FILES} DESTINATION include/LidarFormat/tools COMPONENT headers)
	INSTALL( FILES ${ALL_EXTERN_HEADER_FILES} DESTINATION include/LidarFormat/extern/matis COMPONENT headers)
	#install lib and target config file
	INSTALL ( TARGETS LidarFormat EXPORT LidarFormat-targets DESTINATION lib COMPONENT library ) 
	INSTALL ( EXPORT LidarFormat-targets 
				DESTINATION lib/LidarFormat
			)
	INSTALL (FILES LidarFormatConfig.cmake DESTINATION lib/LidarFormat)
ENDIF(UNIX)



IF(WIN32)
	SET(RELATIVE_INCLUDE_PATH "../include/LidarFormat")
	SET(RELATIVE_INCLUDE_EXTERN_PATH "../include/extern")
	CONFIGURE_FILE(${LIDARFORMAT_CMAKE_SELF_DIR}/LidarFormatConfig.cmake.in ${CMAKE_CURRENT_SOURCE_DIR}/LidarFormatConfig.cmake @ONLY)
	
	INSTALL (FILES ${ALL_LIDAR_FORMAT_HEADER_FILES} DESTINATION include/LidarFormat COMPONENT headers)
	INSTALL( FILES ${ALL_MODELS_HEADER_FILES} DESTINATION include/LidarFormat/models COMPONENT headers)
	INSTALL (FILES ${ALL_FILE_FORMATS_HEADER_FILES} DESTINATION include/LidarFormat/file_formats COMPONENT headers)
	INSTALL( FILES ${ALL_GEOMETRY_HEADER_FILES} DESTINATION include/LidarFormat/geometry COMPONENT headers)
	INSTALL( FILES ${ALL_TOOLS_HEADER_FILES} DESTINATION include/LidarFormat/tools COMPONENT headers)
	INSTALL( FILES ${ALL_EXTERN_HEADER_FILES} DESTINATION include/LidarFormat/extern/matis COMPONENT headers)
	
	INSTALL ( TARGETS LidarFormat EXPORT LidarFormat-targets DESTINATION ${CMAKE_INSTALL_PREFIX}/lib COMPONENT library)
	INSTALL (EXPORT LidarFormat-targets 
				DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake)
	INSTALL (FILES LidarFormatConfig.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake)
ENDIF(WIN32)

