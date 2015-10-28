cmake_minimum_required (VERSION 2.6.0)
project(rrt)

find_package(OpenRAVE 0.9 REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS system python thread program_options REQUIRED)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
	add_definitions("-fno-strict-aliasing -Wall -std=c++0x -g")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used

include_directories(${OpenRAVE_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS} 
	include)

set(HEADERS
    include/rrtPlanner.hpp
    include/rrtNode.hpp
    include/rrtTree.hpp
    )

#link_directories(${OpenRAVE_LIBRARY_DIRS}
#	${Boost_LIBRARY_DIRS})

set(SOURCE src/rrtNode.cpp
	src/rrtTree.cpp 
        src/rrtPlanner.cpp
        src/main.cpp
    )

add_executable(rrt ${SOURCE} ${HEADERS})

#set_target_properties(rrt PROPERTIES COMPILE_FLAGS
#	"${OpenRAVE_CXX_FLAGS}"
#	LINK_FLAGS
#	"${OpenRAVE_LINK_FLAGS}")

target_link_libraries(rrt ${catkin_LIBRARIES}
	${OpenRAVE_CORE_LIBRARIES}
	${OpenRAVE_LIBRARIES}
	${Boost_LIBRARIES} 
        )
