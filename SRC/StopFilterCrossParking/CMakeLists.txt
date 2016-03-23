# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_stopFilterCrossParking)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    stopFilterCrossParking.h
    stopFilterCrossParking.cpp
	../Util/Util.cpp
)
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
