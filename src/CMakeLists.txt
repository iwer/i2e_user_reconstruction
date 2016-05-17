# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(i2e-user-reconstruction VERSION 0.1.0 LANGUAGES CXX)

SET (CMAKE_VERBOSE_MAKEFILE 1)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_BINARY_DIR}/../tools/share/cmake")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/../bin")
set(CMAKE_EXE_LINKER_FLAGS "/NODEFAULTLIB:PocoFoundationd.lib /NODEFAULTLIB:atlthunk.lib /NODEFAULTLIB:LIBC.lib /NODEFAULTLIB:LIBCMT /NODEFAULTLIB:MSVCRT" ${CMAKE_EXE_LINKER_FLAGS} )

# Include third party libraries as targets
include(TargetsBoostStatic)

find_package(UnitTestPlusPlus REQUIRED)
add_library(unittestpp STATIC IMPORTED)
set_property(TARGET unittestpp PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${UnitTestPlusPlus_INCLUDE_DIRS})
set_property(TARGET unittestpp PROPERTY 
	IMPORTED_LOCATION_RELEASE ${UnitTestPlusPlus_LIBRARY})
set_property(TARGET unittestpp PROPERTY 
	IMPORTED_LOCATION_DEBUG ${UnitTestPlusPlus_LIBRARY_DEBUG})

include(TargetsPCLStatic)
	
# openFrameworks gets added as project
find_package(openFrameworks REQUIRED)

include(${openFrameworks_DIR}/addons/ofxGui/ofxGui.cmake)
include(${openFrameworks_DIR}/addons/ofxXmlSettings/ofxXmlSettings.cmake)
include(${openFrameworks_DIR}/addons/ofxSplashScreen/ofxSplashScreen.cmake)

add_library(ofAddons ${ADDONS_SOURCES})
set_property(TARGET ofAddons PROPERTY
	INTERFACE_INCLUDE_DIRECTORIES ${ADDONS_INCLUDE_DIRS})
target_link_libraries(ofAddons openFrameworks)	

# External 3rd party libs that we include
#add_subdirectory(libs/extlib EXCLUDE_FROM_ALL)
 
add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT )

# Targets that we develop
enable_testing()
add_subdirectory(of-pcl-bridge)
add_subdirectory(common)
add_subdirectory(librecon)
add_subdirectory(app)
add_subdirectory(recorder)
add_subdirectory(player)
add_subdirectory(calibrator)
add_subdirectory(autocalibrator)
add_subdirectory(sensorsync)
add_subdirectory(texturemapping)

#add_dependencies(ofAddons openFrameworks)
