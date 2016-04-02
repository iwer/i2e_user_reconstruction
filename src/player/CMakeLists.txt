# Define an executable
add_executable(player
    src/ofApp.cpp
    src/main.cpp
	src/PointCloudPlayer.cpp
	# Headers
	src/ofApp.h
	src/PointCloudPlayer.h
	)

# Define the libraries this project depends upon
target_link_libraries(player 
    openFrameworks 
	ofAddons 
	librecon 
	pcl-common
	pcl-io
	of-pcl-bridge
)