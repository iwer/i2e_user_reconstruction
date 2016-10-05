# Define an executable
add_executable(meshseq-player
    src/ofApp.cpp
    src/main.cpp
	src/PlaySettings.cpp
	# Headers
	src/ofApp.h
	src/PlaySettings.h
	)

# Define the libraries this project depends upon
target_link_libraries(meshseq-player 
    openFrameworks 
	ofAddons 
	librecon 
	common
	pcl-common
	pcl-io
	of-pcl-bridge
	boost-regex
)