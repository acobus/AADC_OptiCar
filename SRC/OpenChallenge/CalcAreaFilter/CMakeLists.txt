include_directories(../include)

set(FILTER_NAME CalcAreaFilter)
 
# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp
                stdafx.h
                CalcAreaFilter.h
                CalcAreaFilter.cpp
)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
