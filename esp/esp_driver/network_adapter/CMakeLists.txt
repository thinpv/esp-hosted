# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(PROJECT_VERSION "0.4")
message(*************************************************************************************)
message("                    Building ESP-Hosted Firmware version :: ${PROJECT_VERSION}             ")
message(*************************************************************************************)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(network_adapter)
