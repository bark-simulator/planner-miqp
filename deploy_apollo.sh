#!/bin/bash

if [[ $SUDO_USER == "fortiss" ]] || [[ $USER == "fortiss" ]] ; then #car pcs
    APOLLO_DESTINATION="/home/fortiss/fortuna_ws/apollo_opensource/miqp"
elif [[ $SUDO_USER == "esterle" ]] || [[ $USER == "esterle" ]] ; then
    APOLLO_DESTINATION="/home/esterle/development/apollo/miqp"
elif [[ $SUDO_USER == "kessler" ]] || [[ $USER == "kessler" ]] ; then
    APOLLO_DESTINATION="/home/kessler/fav/apollo_opensource/miqp"
else
    echo "Sudo User is: " $SUDO_USER " User is " $USER
    echo "user not known, using arguments"
    APOLLO_DESTINATION=$1
fi
echo "Copying files to :" $APOLLO_DESTINATION

chmod -R 777 $APOLLO_DESTINATION

mkdir -p $APOLLO_DESTINATION/lib
cp bazel-bin/src/libmiqp_planner_c_api.so $APOLLO_DESTINATION/lib/libmiqp_planner_c_api.so

mkdir -p $APOLLO_DESTINATION/include
cp src/miqp_planner_c_api.h $APOLLO_DESTINATION/include/src/miqp_planner_c_api.h
cp src/miqp_planner_settings.h $APOLLO_DESTINATION/include/src/miqp_planner_settings.h

mkdir -p $APOLLO_DESTINATION/cplex_modfiles
find cplexmodel/ -type f | grep -i mod$ | xargs -i cp {} $APOLLO_DESTINATION/cplex_modfiles/
