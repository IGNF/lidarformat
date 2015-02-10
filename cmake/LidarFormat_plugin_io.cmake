####
#### Use LAS format
####
OPTION( ENABLE_LAS "Enable LAS" OFF )
if(ENABLE_LAS)
    ADD_DEFINITIONS(-DENABLE_LAS)
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
    ADD_DEFINITIONS(-DENABLE_TERRABIN)
    set(SRC_TERRABIN
        ${SRC_DIR}/LidarFormat/file_formats/TerraBin/TerraBINLidarFileIO.cpp
        ${SRC_DIR}/LidarFormat/file_formats/TerraBin/TerraBINLidarFileIO.h
        ${SRC_DIR}/LidarFormat/extern/terrabin/TerraBin.cpp
        ${SRC_DIR}/LidarFormat/extern/terrabin/TerraBin.h
)
    SET( ALL_SOURCES ${ALL_SOURCES} ${SRC_TERRABIN})
endif(ENABLE_TERRABIN)


####
#### Use PlyArchi format
####
# BV: always enable because it is now used in LidarFile and it does not add any dependency
ADD_DEFINITIONS(-DENABLE_PLYARCHI)
AUX_SOURCE_DIRECTORY(${SRC_DIR}/LidarFormat/file_formats/PlyArchi SRC_PLYARCHI)
SET( ALL_SOURCES ${ALL_SOURCES} ${SRC_PLYARCHI})

FILE( GLOB PLYARCHI_HEADERS src/LidarFormat/file_formats/PlyArchi/*.h )
SET(ALL_FILE_FORMATS_HEADER_FILES ${ALL_FILE_FORMATS_HEADER_FILES} ${PLYARCHI_HEADERS})
