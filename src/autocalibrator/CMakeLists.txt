# Define an executable
add_executable(autocalibrator
    src/ofApp.cpp
    src/main.cpp
	# Headers
	src/ofApp.h
	)

# Define the libraries this project depends upon
target_link_libraries(autocalibrator 
    openFrameworks 
	ofAddons 
	librecon 
	pcl-common
	pcl-io
	pcl-sample-consensus
	pcl-filters
	pcl-registration
	of-pcl-bridge
)