# Define an executable
add_executable(app
    src/ofApp.cpp
    src/main.cpp
	src/Controls.cpp
	src/ControlsOfxGui.cpp
	# Headers
	src/ofApp.h
	src/Controls.h
	src/ControlsOfxGui.h
	)

# Define the libraries this project depends upon
target_link_libraries(app 
    openFrameworks 
	ofAddons 
	librecon
	of-pcl-bridge
	common
)