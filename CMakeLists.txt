cmake_minimum_required(VERSION 2.8)

project(rockySlope CXX)

# Find Irrlicht
SET(CH_IRRLICHTDIR "D:/irrlicht-1.8"  CACHE PATH   "Where is your Irrlicht SDK installed? You must set this path to compile demos with 3D display.")
IF(EXISTS "${CH_IRRLICHTDIR}/include")
    SET(CH_IRRLICHTINC "${CH_IRRLICHTDIR}/include")
ELSE()
    SET(CH_IRRLICHTINC "${CH_IRRLICHTDIR}")
ENDIF()
SET(CH_IRRLICHTLIB "D:/irrlicht-1.8"  CACHE PATH   "Where is your Irrlicht SDK installed? You must set this path to compile demos with 3D display.")
IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
    FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win32-visualstudio")
ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
    FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win64-visualStudio")
ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
    FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win32-gcc")
ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
    FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win64-gcc")
ENDIF()

INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )

# Find Chrono
set(CHRONO_DIR chrono CACHE PATH "Build/install directory for Chrono")
find_file(CHRONOCONFIG_PATH ChronoEngineConfig.cmake PATHS ${CHRONO_DIR})
if(EXISTS ${CHRONOCONFIG_PATH})
    include(${CHRONOCONFIG_PATH})
    include_directories(${CHRONOENGINE_INCLUDE_DIRS})
endif()

#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )


#--------------------------------------------------------------

ADD_EXECUTABLE(rockySlope   	    rockySlope.cpp)
TARGET_LINK_LIBRARIES(rockySlope
	${CH_IRRLICHTLIB} 
 	ChronoEngine)
ADD_DEPENDENCIES (rockySlope ChronoEngine)

install(TARGETS rockySlope DESTINATION .)