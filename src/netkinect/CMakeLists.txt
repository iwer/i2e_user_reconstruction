# Define library. Only source files here!

add_library(netkinect
    # Source files
	src/ServerAPI.cpp
	src/Client.cpp
    src/TCPConnection.cpp
    gen/KinectFrameMessage.pb.cc
    gen/ConnectionMessage.pb.cc
	# Header files (to be able to edit in VS et. al.)
	include/Common.h
	include/ServerAPI.h
	include/Client.h
    include/TCPConnection.h
    include/Connection.h
    include/Logger.h
    include/PCLUtil.h
    ## protobuf generated
    gen/KinectFrameMessage.pb.h
    gen/ConnectionMessage.pb.h
	)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(netkinect PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# If we have compiler requirements for this library, list them
# here
target_compile_features(netkinect
    PUBLIC cxx_auto_type
    PRIVATE cxx_variadic_templates)

# Depend on a library that we defined in the top-level file
target_link_libraries(netkinect
    pthread
    libprotobuf
)

# 'make install' to the correct location
install(TARGETS netkinect
    ARCHIVE  DESTINATION lib
    LIBRARY  DESTINATION lib
    RUNTIME  DESTINATION bin)  # This is for Windows
install(DIRECTORY include/ DESTINATION include)

# This makes the project importable from the build directory
export(TARGETS netkinect FILE NetKinectConfig.cmake)


