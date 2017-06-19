cmake_minimum_required(VERSION 2.8.3)
project(bw_generator)

## gnu++11 feature
set(CMAKE_CXX_FLAGS "-std=gnu++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
set(catkin_deps
    roscpp
    roslib
    nodelet
    std_msgs
    )

find_package(catkin REQUIRED ${catkin_deps})

###################################
## catkin specific configuration ##
###################################
catkin_package(INCLUDE_DIRS   include
               CATKIN_DEPENDS ${catkin_deps}
               DEPENDS        Boost
               LIBRARIES      ${PROJECT_NAME}_nodelet
)

###########
## Build ##
###########
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)
## Declare a C++ library
add_library(${PROJECT_NAME}_nodelet
            src/bw_generator_nodelet.cpp
)
## Link the catkin libraries
target_link_libraries(${PROJECT_NAME}_nodelet
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
)
## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/bw_generator_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
                      ${catkin_LIBRARIES}
                      ${PROJECT_NAME}_nodelet
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_nodelet ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Need to explicitly list nodelet.xml for installation
install(FILES
  nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
