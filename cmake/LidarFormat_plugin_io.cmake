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
    ADD_DEFINITIONS(-DENABLE_PLYARCHI)
    AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat/file_formats/PlyArchi  SRC_PLYARCHI)
    SET( ALL_SOURCES ${ALL_SOURCES} ${SRC_PLYARCHI})
endif(ENABLE_PLYARCHI)
