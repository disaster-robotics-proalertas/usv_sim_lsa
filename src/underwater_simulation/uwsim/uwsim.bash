#!/bin/bash

#TODO: Check for ~/.uwsim/data, download if it doesn't exist

DATAFILE=data.tgz

if [ ! -d $HOME/.uwsim/data ]
then
  echo -e "The UWSim data directory (~/.uwsim/data) does not exist. We need to download ~300Mb of data and place it under ~/.uwsim/data. Note this is required to run UWSim.\nContinue (Y/n) ? "
  read answer
  if [[ -z $answer || $answer == "Y" || $answer == "y" ]]
  then
    mkdir -p ~/.uwsim/data
    wget http://www.irs.uji.es/uwsim/files/$DATAFILE -O ~/.uwsim/UWSim-data.tgz && tar -zxvf ~/.uwsim/UWSim-data.tgz -C ~/.uwsim
    rm ~/.uwsim/UWSim-data.tgz
  else
    echo "Cannot run UWSim."
    exit
  fi
fi

echo "Starting UWSim..."
rosrun uwsim uwsim_binary --dataPath ~/.uwsim/data $@
