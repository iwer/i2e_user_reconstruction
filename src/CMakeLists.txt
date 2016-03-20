# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(myproject VERSION 0.1.0 LANGUAGES CXX)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_BINARY_DIR}/../tools/share/cmake")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/../bin")
set(CMAKE_EXE_LINKER_FLAGS "/NODEFAULTLIB:PocoFoundationd.lib /NODEFAULTLIB:atlthunk.lib /NODEFAULTLIB:LIBC.lib /NODEFAULTLIB:LIBCMT /NODEFAULTLIB:MSVCRT" ${CMAKE_EXE_LINKER_FLAGS} )

# Include Boost as an imported target
if(WIN32)
	SET(Boost_COMPILER "-vc140")
endif()
set(Boost_ADDITIONAL_VERSIONS "1.47" "1.47.0" "1.60" "1.60.0")
find_package(Boost COMPONENTS "chrono" "date_time" "filesystem" "graph" "iostreams" "program_options" "regex" "serialization" "signals" "system" "thread" "wserialization" REQUIRED)
add_library(boost-datetime SHARED IMPORTED)
set_property(TARGET boost-datetime PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIRS})
set_property(TARGET boost-datetime PROPERTY 
	IMPORTED_LOCATION_RELEASE ${Boost_DATE_TIME_LIBRARY_RELEASE})
set_property(TARGET boost-datetime PROPERTY 
	IMPORTED_LOCATION_DEBUG ${Boost_DATE_TIME_LIBRARY_DEBUG})

add_library(boost-filesystem SHARED IMPORTED)
set_property(TARGET boost-filesystem PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIRS})
set_property(TARGET boost-filesystem PROPERTY 
	IMPORTED_LOCATION_RELEASE ${Boost_FILESYSTEM_LIBRARY_RELEASE})
set_property(TARGET boost-filesystem PROPERTY 
	IMPORTED_LOCATION_DEBUG ${Boost_FILESYSTEM_LIBRARY_DEBUG})

add_library(boost-system SHARED IMPORTED)
set_property(TARGET boost-system PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIRS})
set_property(TARGET boost-system PROPERTY 
	IMPORTED_LOCATION_RELEASE ${Boost_SYSTEM_LIBRARY_RELEASE})
set_property(TARGET boost-system PROPERTY 
	IMPORTED_LOCATION_DEBUG ${Boost_SYSTEM_LIBRARY_DEBUG})

add_library(boost-chrono SHARED IMPORTED)
set_property(TARGET boost-chrono PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIRS})
set_property(TARGET boost-chrono PROPERTY 
	IMPORTED_LOCATION_RELEASE ${Boost_CHRONO_LIBRARY_RELEASE})
set_property(TARGET boost-chrono PROPERTY 
	IMPORTED_LOCATION_DEBUG ${Boost_CHRONO_LIBRARY_DEBUG})

add_library(boost-iostreams SHARED IMPORTED)
set_property(TARGET boost-iostreams PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIRS})
set_property(TARGET boost-iostreams PROPERTY 
	IMPORTED_LOCATION_RELEASE ${Boost_IOSTREAMS_LIBRARY_RELEASE})
set_property(TARGET boost-iostreams PROPERTY 
	IMPORTED_LOCATION_DEBUG ${Boost_IOSTREAMS_LIBRARY_DEBUG})

add_library(boost-serialization SHARED IMPORTED)
set_property(TARGET boost-serialization PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIRS})
set_property(TARGET boost-serialization PROPERTY 
	IMPORTED_LOCATION_RELEASE ${Boost_SERIALIZATION_LIBRARY_RELEASE})
set_property(TARGET boost-serialization PROPERTY 
	IMPORTED_LOCATION_DEBUG ${Boost_SERIALIZATION_LIBRARY_DEBUG})

add_library(boost-thread SHARED IMPORTED)
set_property(TARGET boost-thread PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIRS})
set_property(TARGET boost-thread PROPERTY 
	IMPORTED_LOCATION_RELEASE ${Boost_THREAD_LIBRARY_RELEASE})
set_property(TARGET boost-thread PROPERTY 
	IMPORTED_LOCATION_DEBUG ${Boost_THREAD_LIBRARY_DEBUG})

find_package(UnitTestPlusPlus REQUIRED)
add_library(unittestpp INTERFACE IMPORTED)
set_property(TARGET unittestpp PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${UnitTestPlusPlus_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED)
add_library(pcl INTERFACE IMPORTED)
set_property(TARGET pcl PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES ${PCL_INCLUDE_DIRS})
set_property(TARGET pcl PROPERTY
	INTERFACE_COMPILE_DEFINITIONS ${PCL_DEFINITIONS})
	
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
 
# Targets that we develop
enable_testing()
add_subdirectory(librecon)
add_subdirectory(app)
add_dependencies(ofAddons openFrameworks)
