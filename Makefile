# @Author: Nick Steele <nichlock>
# @Date:   21:46 Sep 18 2020
# @Last modified by:   nichlock
# @Last modified time: 22 Feb, 2023

CATKIN_MAKE=catkin_make

# Compiler setup
# CXX=g++-8
# CMAKE_CXX_ARG=-DCMAKE_CXX_COMPILER=$(CXX)

# CC=gcc-8
# CMAKE_CC_ARG=-DCMAKE_C_COMPILER=$(CC)

# Debug symbols for debugging with GDB
CMAKE_DEBUG_ARG=-DCMAKE_BUILD_TYPE=DEBUG

.PHONY: all first all-ignore-error debug clean

all:
	$(CATKIN_MAKE) $(CMAKE_CC_ARG) $(CMAKE_CXX_ARG)

debug:
	$(CATKIN_MAKE) $(CMAKE_CC_ARG) $(CMAKE_CXX_ARG) $(CMAKE_DEBUG_ARG)

clean:
	$(CATKIN_MAKE) clean
