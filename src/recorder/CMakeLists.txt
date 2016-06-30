# Define an executable
add_executable(recorder
    src/ofApp.cpp
    src/main.cpp
	src/PointCloudWriter.cpp
	# Headers
	src/ofApp.h
	src/PointCloudWriter.h
	)

# Define the libraries this project depends upon
target_link_libraries(recorder 
    openFrameworks 
	ofAddons 
	librecon 
	common
	of-pcl-bridge
)