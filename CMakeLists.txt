# +====================================================================+
# |          Open MObile Robot Architecture (MORA)                     |
# |                                                                    |
# |                 https://github.com/OpenMORA                        |
# |                                                                    |
# | Copyright individual authors (C) 2010-14  - See AUTHORS.txt        |
# | Project released under BSD License                                 |
# +====================================================================+

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)  # Simpler CMake syntax.
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11) # Minimum version required by MOOS v10 cfg files

PROJECT(nav_slam_pkg)

cmake_policy(SET CMP0003 NEW)  # Don't mix rel & abs lib paths
cmake_policy(SET CMP0005 NEW)  # Autoescape -D definitions

FIND_PACKAGE(MORA REQUIRED)  # Import MOOS vars, MORA CMake macros,etc.

MORA_COMMON_CMAKE() # Set common CMake options

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${${PROJECT_NAME}_SOURCE_DIR}/cmakemodules")

# Add modules (dir names match modules' by default)
DEFINE_MORA_PROGRAM(Localization2D_Fusion)
DEFINE_MORA_PROGRAM(Localization2D_PF)
DEFINE_MORA_PROGRAM(NavigatorReactivePTG)
DEFINE_MORA_PROGRAM(NavigatorReactivePTG3D)

