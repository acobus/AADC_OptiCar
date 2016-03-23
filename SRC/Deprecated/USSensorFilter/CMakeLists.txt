# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_USSensorFilter)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    USSensorFilter.h
    USSensorFilter.cpp
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
