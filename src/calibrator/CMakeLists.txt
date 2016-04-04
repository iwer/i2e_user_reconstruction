# Define an executable
add_executable(calibrator
    src/ofApp.cpp
    src/main.cpp
	# Headers
	src/ofApp.h
	)

# Define the libraries this project depends upon
target_link_libraries(calibrator 
    openFrameworks 
	ofAddons 
	librecon 
	pcl-common
	pcl-io
	of-pcl-bridge
)