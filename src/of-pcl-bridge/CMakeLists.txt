# Define library. Only source files here!

add_library(of-pcl-bridge
    # Source files
	src/of-pcl-bridge.cpp
	# Header files (to be able to edit in VS et. al.)
	include/of-pcl-bridge/of-pcl-bridge.h
	)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(of-pcl-bridge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# If we have compiler requirements for this library, list them
# here
target_compile_features(of-pcl-bridge
    PUBLIC cxx_auto_type
    PRIVATE cxx_variadic_templates)

# Depend on a library that we defined in the top-level file
target_link_libraries(of-pcl-bridge
	openFrameworks
	pcl-common 
	pcl-io
)

# 'make install' to the correct location
install(TARGETS of-pcl-bridge
    ARCHIVE  DESTINATION lib
    LIBRARY  DESTINATION lib
    RUNTIME  DESTINATION bin)  # This is for Windows
install(DIRECTORY include/ DESTINATION include)

# This makes the project importable from the build directory
export(TARGETS of-pcl-bridge FILE OfPclBridgeConfig.cmake)

# Every library has unit tests, of course
#add_executable(testof-pcl-bridge
#    test/main.cpp
#	test/sanity_check.cpp
#	test/test.h)

#target_link_libraries(testof-pcl-bridge
#    librecon 
#	unittestpp)

#add_test(testlibrecon testof-pcl-bridge)