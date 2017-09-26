#!/usr/bin/env bash
# This script gathers experiment data stored on all apex-XX-torso.local robots locally.


if [ $# != 1 ]; then
    echo -e "\e[31mUSAGE: $0 experiment_name\e[0m"
    exit 1
fi

tar_and_copy() {
    experiment=$1
    id=$2
    path="/media/usb0/$experiment"

    echo -e "Archiving data from experiment $experiment on apex-$id..."
    ssh pi@apex-$id-torso.local "sudo tar -cf /tmp/$experiment.tar $path"
    if [ $? -eq 0 ]; then
        local_file="/tmp/$experiment-$id.tar"
        echo -e "Copying data into $local_file..."
        scp pi@apex-$id-torso.local:/tmp/$experiment.tar $local_file
        echo -e "unziping..."
        tar -xf $local_file
        echo -e "\e[32mapex-$id DONE!\e[39"
    fi
}

export -f tar_and_copy

experiment="$1"
for id in `seq 1 6`; do
    tar_and_copy "$experiment" "$id" &
done
wait

