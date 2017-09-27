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
    tar_path="/media/usb0/$experiment-$id.tar"
    local_tar_path="/tmp/$experiment-$id.tar"

    echo -e "Archiving data from experiment $experiment on apex-$id..."
    ssh pi@apex-$id-torso.local "sudo tar -cf $tar_path $path --exclude \"*avi\""
    if [ $? -eq 0 ]; then
        echo -e "Copying tar file into $local_tar_path..."
        scp pi@apex-$id-torso.local:$tar_path $local_tar_path
        if [ $? -eq 0 ]; then
            echo -e "unziping tar..."
            tar -xf $local_tar_path
            echo -e "deleting remote tar..."
            ssh pi@apex-$id-torso.local "sudo rm $tar_path"
            echo -e "\e[32mapex-$id DONE!\e[39m"
        else
            echo -e "\e[31mapex-$id FAILED while copying tar file from remote!\e[39m"
        fi
    else
        echo -e "\e[31mapex-$id FAILED while creating tar archive on remote!\e[39m"
    fi
}

export -f tar_and_copy

experiment="$1"
for id in `seq 1 6`; do
    tar_and_copy "$experiment" "$id" &
done
wait

