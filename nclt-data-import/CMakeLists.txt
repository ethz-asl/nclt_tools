cmake_minimum_required (VERSION 2.8)
project(nclt_data_import)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME} src/import-ground-truth.cc)

cs_install()
cs_export()
