cmake_minimum_required(VERSION 3.0.2)
project(farmbeast_simulation)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES farmbeast_simulation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include 
 ${catkin_INCLUDE_DIRS}
)
add_executable(cmd_vel_control src/control.cpp)
target_link_libraries(cmd_vel_control ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


catkin_install_python(PROGRAMS 
src/test.py
src/GUI.py
src/nevem.py
src/SkupniUI.py
src/NimamPojmaPartTwoElectricBoogaloo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

