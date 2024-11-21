message("********************************************************")
message("*                    RUN CMAKE                         *")
message("********************************************************")
# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

#project version
#If not set it will be set to the git version:
#   The hex numbers are the git commit your project is on. 
#   The -dirty indicates the source code has been changed since that commit was done
set(PROJECT_VER "0.8.26") 

#set(COMPONENTS hardware icm42688 main esptool_py)
#set(COMPONENTS hardware icm42688 main AHRS esptool_py)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

#project name
project(PRISONATOR)
