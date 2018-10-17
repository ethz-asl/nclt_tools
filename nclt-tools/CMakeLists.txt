cmake_minimum_required(VERSION 2.8.3)
project(nclt_tools)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
  src/csv-parsing.cc
  src/transformation-helpers.cc)

cs_install()
cs_export()
