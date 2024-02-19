#! /bin/bash

filename="model"
inputfile="${1}/${filename}.sdf"
outputfile="${1}/${filename}.usd"

~/./Documents/Github/gz-usd/build/bin/sdf2usd ${inputfile} ${outputfile}

cd $1
if [ ! -d materials/textures ]; then
    mkdir -p materials/textures
    if [ -d meshes ]; then
        cp -r meshes/* materials/textures
    else
        echo $inputfile
        cp *.png materials/textures/
        cp *.jpg materials/textures/
    fi
fi