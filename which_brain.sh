#!/bin/bash

hn=$(hostname)

if [ $hn == "28401227d07c" ]
then
    echo "brain1"
elif [ $hn == "8078cee05e73" ]
then
    echo "brain2"
elif [ $hn == "d43c614fb8cf" ]
then
    echo "brain3"
else
    echo "unknown"
fi
