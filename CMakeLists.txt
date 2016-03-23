cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)



# The project and cmake vars must only be defined in case that only the base
# filter should be build.
if(NOT AADC_BUILD_ALL_IN_ONE)
    # define a project name
    project(aadc_user)
	Message("compiler")
    #set the install prefix if not already done.
	if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)	
		if(MSVC10)
			set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../../bin/${CMAKE_SYSTEM_NAME}/vc10" CACHE PATH "The install directory" FORCE)     
		elseif(CMAKE_COMPILER_IS_GNUCXX)
			set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../../bin/${CMAKE_SYSTEM_NAME}/gcc" CACHE PATH "The install directory" FORCE)     
		else()
			set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../../bin/${CMAKE_SYSTEM_NAME}" CACHE PATH "The install directory" FORCE)  
		endif()
		set(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT FALSE)
		message("Install dir is ${CMAKE_INSTALL_PREFIX}")
	endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    # CMAKE_BUILD_TYPE is empty on Unix systems, but needed by ADTF. Otherwise the ADTF libs
    # can't be found and linked.
    if(UNIX AND NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
                "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
                FORCE)
    endif(UNIX AND NOT CMAKE_BUILD_TYPE)
    
    # set toolchain file on Unix
    if(UNIX AND NOT CMAKE_TOOLCHAINFILE)
        set(CMAKE_TOOLCHAIN_FILE "/opt/adtf/toolchainfiles/ARM_toolchain_build_on_target.cmake")
    endif(UNIX AND NOT CMAKE_TOOLCHAINFILE)

endif(NOT AADC_BUILD_ALL_IN_ONE)

find_package(ADTF REQUIRED)
#-------General------------


set(AADC_INSTALL_DIR .)
set(AADC_INSTALL_DIR_LIBS .)

# angepasst
set(CMAKE_CXX_FLAGS "-Wuninitialized -Wall")

set(AADC_DIR ../)
find_package(AADC REQUIRED)

#SET (CMAKE_VERBOSE_MAKEFILE on)

#--------------sources----------------------------
include_directories(./include)

add_subdirectory(src/CrossParkingFilter)
add_subdirectory(src/ParallelParkingFilter)
add_subdirectory(src/PullOutParking)
add_subdirectory(src/DecisionMaking)
add_subdirectory(src/StateController)
#add_subdirectory(src/StartFilter)
add_subdirectory(src/TestStopFilterParking)
add_subdirectory(src/StopFilterParallelParking)
add_subdirectory(src/StopFilterCrossParking)
add_subdirectory(src/TestStopFilter)
add_subdirectory(src/EmergencySTOP)
add_subdirectory(src/RoadSignPreprocessing)
add_subdirectory(src/LaneTracking)
#add_subdirectory(src/VisualOdometry)
add_subdirectory(src/Undistort)
add_subdirectory(src/TickFilter)
add_subdirectory(src/WheelSpeedController)
add_subdirectory(src/SwitchBoolFilter)
#add_subdirectory(src/ImageProcessing)
add_subdirectory(src/TurningFilter)
add_subdirectory(src/StopTurningFilter)
#add_subdirectory(src/StopTurningFilterZeitmessung)
add_subdirectory(src/MarkerDetection)
add_subdirectory(src/CameraAngleCalibration)
add_subdirectory(src/Hold)
add_subdirectory(src/CrossingDetect)
add_subdirectory(src/ParkDetection)
add_subdirectory(src/CrossingManeuverGenerator)
add_subdirectory(src/AdaptiveCruiseControl)
add_subdirectory(src/DriveRound)
add_subdirectory(src/TestPullOutFilter)
add_subdirectory(src/StopZebraStripesFilter)
add_subdirectory(src/TicTacTactic)
add_subdirectory(src/ZebraStripes)
add_subdirectory(src/CheckPullOut)
#add_subdirectory(src/USSensorFilter)
add_subdirectory(src/OpenChallenge/CalcAreaFilter)
add_subdirectory(src/OpenChallenge/LineTracking)
add_subdirectory(src/OpenChallenge/ScaleTracking)
add_subdirectory(src/OpenChallenge/TickFilterOC)
add_subdirectory(src/OpenChallenge/OCMarkerDetection)
