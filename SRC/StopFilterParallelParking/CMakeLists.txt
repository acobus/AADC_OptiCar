# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_stopFilterParallelParking)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    stopFilterParallelParking.h
    stopFilterParallelParking.cpp
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
