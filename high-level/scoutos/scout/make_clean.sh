#!/bin/bash

# Runs 'make clean' on all subfolders.
# Would be better to integrate this into Makefile, but I can't seem to
# make that work. -Alex

for LISTING in *
do
    if [[ -d $LISTING && (( $LISTING != "." && $LISTING != ".." )) ]]
    then
        echo "Making clean: $LISTING"
        cd $LISTING
        make clean
        cd ..
    fi
done
