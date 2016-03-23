# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME StopTurningFilterZeitmessung)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    StopTurningFilterZeitmessung.h
    StopTurningFilterZeitmessung.cpp
)

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
