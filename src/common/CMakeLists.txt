# Define library. Only source files here!

add_library(common
    # Source files
	src/common.cpp
	src/SensorCalibrationSettings.cpp
	src/PointCloudPlayer.cpp
	# Header files (to be able to edit in VS et. al.)
	include/common/common.h
	include/common/SensorCalibrationSettings.h
	include/common/PointCloudPlayer.h
	include/common/PclCloudAndImage.h
	include/common/PclCloudAndOfImage.h
	)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# If we have compiler requirements for this library, list them
# here
target_compile_features(common
    PUBLIC cxx_auto_type
    PRIVATE cxx_variadic_templates)

# Depend on a library that we defined in the top-level file
target_link_libraries(common
	openFrameworks
	ofAddons
	of-pcl-bridge
	pcl-common
	pcl-octree
	pcl-io
	pcl-kdtree
	pcl-features
	pcl-sample-consensus
	pcl-filters
	pcl-ml
	pcl-segmentation
	pcl-registration
	pcl-keypoints
	pcl-tracking
	pcl-recognition
	pcl-stereo
	pcl-search
	pcl-surface
	librecon
)

# 'make install' to the correct location
install(TARGETS common
    ARCHIVE  DESTINATION lib
    LIBRARY  DESTINATION lib
    RUNTIME  DESTINATION bin)  # This is for Windows
install(DIRECTORY include/ DESTINATION include)

# This makes the project importable from the build directory
export(TARGETS common FILE CommonConfig.cmake)

# Every library has unit tests, of course
#add_executable(testcommon
#    test/main.cpp
#	test/sanity_check.cpp
#	test/test.h)

#target_link_libraries(testcommon
#    librecon 
#	unittestpp)

#add_test(testlibrecon testcommon)