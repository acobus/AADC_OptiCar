
set(FILTER_NAME aadc_markerDetection)

#set include directories
include_directories(${aruco_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIR}) 

# add the source
adtf_add_filter(${FILTER_NAME}
				stdafx.cpp  
				stdafx.h
				cMarkerDetectFilter.h
				cMarkerDetectFilter.cpp
		)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS} ${aruco_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/imageproc) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
	
