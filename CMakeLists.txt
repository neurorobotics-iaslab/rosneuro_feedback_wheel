cmake_minimum_required(VERSION 3.0)
project(rosneuro_feedback_wheel)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 rosneuro_msgs
			 )

find_library(NEURODRAW_LIBRARIES neurodraw REQUIRED)
find_path(NEURODRAW_INCLUDE_DIRS neurodraw/Shape.h REQUIRED)

find_library(NEUROCHRONO_LIBRARIES neurochrono REQUIRED)
find_path(NEUROCHRONO_INCLUDE_DIRS neurochrono/Timer.h REQUIRED)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	rosneuro_msgs
  DEPENDS
)


###########
## Build ##
###########

include_directories(include 
					${catkin_INCLUDE_DIRS} 
					${NEURODRAW_INCLUDE_DIRS}
					${NEUROCHRONO_INCLUDE_DIRS})

add_library(${PROJECT_NAME}	
			src/SingleWheel.cpp
			src/TrainingWheel.cpp
			src/ControlWheel.cpp
			src/TrialSequence.cpp
			src/Autopilot.cpp
			)

target_link_libraries(${PROJECT_NAME} 
					  ${catkin_LIBRARIES}
					  ${NEURODRAW_LIBRARIES}
					  ${NEUROCHRONO_LIBRARIES})

add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
)

#################
## Executables ##
#################
add_executable(trainingwheel src/trainingwheel.cpp)
target_link_libraries(trainingwheel ${PROJECT_NAME}
								 ${catkin_LIBRARIES}
					  			 ${NEURODRAW_LIBRARIES}
					  			 ${NEUROCHRONO_LIBRARIES})
add_executable(controlwheel src/controlwheel.cpp)
target_link_libraries(controlwheel ${PROJECT_NAME}
								 ${catkin_LIBRARIES}
					  			 ${NEURODRAW_LIBRARIES}
					  			 ${NEUROCHRONO_LIBRARIES})
