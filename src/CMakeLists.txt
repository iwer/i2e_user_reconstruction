# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(i2e-user-reconstruction VERSION 0.1.0 LANGUAGES C CXX)

SET (CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_CUSTOM_MODULE_PATH "${CMAKE_SOURCE_DIR}/../tools/share/cmake")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CUSTOM_MODULE_PATH})
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/../bin")
if (MSVC)
	set(CMAKE_EXE_LINKER_FLAGS "/NODEFAULTLIB:PocoFoundationd.lib /NODEFAULTLIB:atlthunk.lib /NODEFAULTLIB:LIBC.lib /NODEFAULTLIB:LIBCMT /NODEFAULTLIB:MSVCRT" ${CMAKE_EXE_LINKER_FLAGS} )
endif (MSVC)

# Include third party libraries as targets
include(TargetsBoostStatic)
#include(TargetsUnitTestPlusPlus)
include(TargetsPCLStatic)

#find libprotobuf
find_package(Protobuf REQUIRED)
add_library(libprotobuf STATIC IMPORTED)
set_property(TARGET libprotobuf PROPERTY
		INTERFACE_INCLUDE_DIRECTORIES ${Protobuf_INCLUDE_DIRS})
set_property(TARGET libprotobuf PROPERTY
		IMPORTED_LOCATION_RELEASE ${Protobuf_LIBRARY})
set_property(TARGET libprotobuf PROPERTY
		IMPORTED_LOCATION_DEBUG ${Protobuf_LIBRARY_DEBUG})
	
# openFrameworks gets added as project
find_package(openFrameworks REQUIRED)
export(TARGETS openFrameworks FILE openFrameworksConfig.cmake)

include(${openFrameworks_DIR}/addons/ofxGui/ofxGui.cmake)
include(${openFrameworks_DIR}/addons/ofxXmlSettings/ofxXmlSettings.cmake)
include(${openFrameworks_DIR}/addons/ofxSplashScreen/ofxSplashScreen.cmake)

add_library(ofAddons ${ADDONS_SOURCES})
set_property(TARGET ofAddons PROPERTY
	INTERFACE_INCLUDE_DIRECTORIES ${ADDONS_INCLUDE_DIRS})
export(TARGETS ofAddons FILE ofAddonsConfig.cmake)

target_link_libraries(ofAddons openFrameworks)	

# External 3rd party libs that we include
#add_subdirectory(libs/extlib EXCLUDE_FROM_ALL)
 
#add_definitions(EIGEN_DONT_VECTORIZE EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT )

# Targets that we develop
enable_testing()
add_subdirectory(of-pcl-bridge)
add_subdirectory(common)
add_subdirectory(librecon)
#add_subdirectory(app)
add_subdirectory(recorder)
add_subdirectory(player)
add_subdirectory(calibrator)
add_subdirectory(autocalibrator)
#add_subdirectory(sensorsync)
#add_subdirectory(texturemapping)
#add_subdirectory(imageloader)
add_subdirectory(fast-reconstruction)
add_subdirectory(slow-reconstruction)
add_subdirectory(meshseq-player)
if(UNIX)
add_subdirectory(netkinect)
add_subdirectory(netkinect-demo)
endif(UNIX)
