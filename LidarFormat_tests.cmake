ENABLE_TESTING()

#########
##OPTION Pour indiquer le chemin des données lidar pour effectuer les tests
SET( PATH_LIDAR_TEST_DATA "data" CACHE STRING "Path to lidar test data" )
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/tests/config_data_test.h.cmake.in ${CMAKE_CURRENT_SOURCE_DIR}/tests/config_data_test.h @only immediate)


SET( all_tests  
        concepts 
        unit_tests
        01_ex_basics
        02_ex_filtering
    )

FOREACH( one_test ${all_tests} )
    ADD_EXECUTABLE(${one_test} tests/${one_test}.cpp)
    TARGET_LINK_LIBRARIES(${one_test} LidarFormat ${LidarFormat_LIBRAIRIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
    ADD_DEPENDENCIES(${one_test} LidarFormat)

    ADD_TEST(LidarFormat_${one_test} ${one_test})
ENDFOREACH( one_test ${all_tests} )
