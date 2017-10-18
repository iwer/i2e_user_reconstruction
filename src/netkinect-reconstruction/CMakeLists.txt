# Define an executable
add_executable(netkinect-reconstruction
    src/ofApp.cpp
    src/main.cpp
	# Headers
	src/ofApp.h
	)

# Define the libraries this project depends upon
target_link_libraries(netkinect-reconstruction
    openFrameworks 
	ofAddons 
	librecon 
	common
	pcl-common
	pcl-io
	of-pcl-bridge
	boost-regex
    netkinect
)