set(FILTER_NAME CrossingManeuverGenerator)

adtf_use_qt(QtCore QtGui)

#we build this example only if qt was found
if(QT_FOUND)

    set(HEADER_MOC
        displaywidget.h
        CrossingManeuverGenerator.h
        )
    
    adtf_add_moc(${FILTER_NAME}
                ${HEADER_MOC}
    )
    
    adtf_add_filter(${FILTER_NAME}
                    stdafx.cpp
                    stdafx.h
                    displaywidget.cpp
                    CrossingManeuverGenerator.h
                    CrossingManeuverGenerator.cpp
                    ${HEADER_MOC}
    )

    # set folder (only needed and available for MSVC)
    adtf_set_folder(${FILTER_NAME} aadcuser/CrossingManeuverGenerator) 
    
    adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
    
endif(QT_FOUND)

