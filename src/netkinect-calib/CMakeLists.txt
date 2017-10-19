# Define an executable
add_executable(netkinect-calib
    src/ofApp.cpp
    src/main.cpp
        # Headers
	src/ofApp.h
		src/NetKinectPointCloudGenerator.cpp src/NetKinectPointCloudGenerator.h src/NetKinectSensor.cpp src/NetKinectSensor.h)

# Define the libraries this project depends upon
target_link_libraries(netkinect-calib 
    openFrameworks
	ofAddons
	librecon
	common
	pcl-common
	pcl-io
	pcl-sample-consensus
	pcl-filters
	pcl-registration
	of-pcl-bridge
    netkinect
	protobuf
)
