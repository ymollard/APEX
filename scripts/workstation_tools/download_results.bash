#!/usr/bin/env bash

if [ $# != 1 ]; then
    echo -e "\e[31mUSAGE: $0 experiment_name\e[0m"
    exit 1
fi

path="/media/usb0/$1"
experiment="$1"

for id in `seq 1 6`; do
    echo -e "Archiving data from apex-$id..."
    ssh pi@apex-$id-torso.local "sudo tar -cf /tmp/$experiment.tar $path"
    if [ $? -eq 0 ]; then
        local_file="/tmp/$experiment-$id.tar"
        echo -e "Copying data into $local_file..."
        scp pi@apex-$id-torso.local:/tmp/$experiment.tar $local_file
        echo -e "unziping..."
        tar -xf $local_file
    fi
done
