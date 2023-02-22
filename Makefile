# @Author: Nick Steele <nichlock>
# @Date:   21:46 Sep 18 2020
# @Last modified by:   nichlock
# @Last modified time: 19:01 Sep 19 2020

PROJECT_NAME=joy_mapper
NODE_NAME=joy_mapper_node

CMD_CATKIN_MAKE=catkin_make
CMD_ROSRUN=rosrun
CMAKE_DEBUG_ARG=-DCMAKE_BUILD_TYPE=DEBUG

.PHONY: all first debug clean run

all:
	$(CMD_CATKIN_MAKE) $(CMAKE_CC_ARG) $(CMAKE_CXX_ARG)

run:
	$(CMD_ROSRUN) $(PROJECT_NAME) $(NODE_NAME)

debug:
	$(CMD_CATKIN_MAKE) $(CMAKE_CC_ARG) $(CMAKE_CXX_ARG) $(CMAKE_DEBUG_ARG)

clean:
	$(CMD_CATKIN_MAKE) clean
