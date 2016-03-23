#!/bin/sh

export DISPLAY=:0
export ADTF_DIR=/opt/adtf/2.13.1
export ADTF_PROJECT_PATH=./config/UserConfig2/UserConfig.prj

if [ -d $ADTF_DIR ]; then
    echo "ADTF dir found."
else
    echo "ADTF dir not found in $ADTF_DIR. Check the path to ADTF."
    exit 1
fi


$ADTF_DIR/bin/adtf_devenv -project=$ADTF_PROJECT_PATH 2>> ErrorUserConfig.dat

$SHELL



