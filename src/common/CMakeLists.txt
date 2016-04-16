# Define library. Only source files here!

add_library(common
    # Source files
	src/SensorCalibrationSettings.cpp
	# Header files (to be able to edit in VS et. al.)
	include/common/common.h
	include/common/SensorCalibrationSettings.h
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