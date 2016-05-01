#!/bin/bash
#
# model_list.sh
# calculating the number of picture for each model

BIN_PATH=/home/dxq/ORB_SLAM/Model;

# for d in `ls -l * |grep "^d"|wc -l `
# ls -F /opt/soft |grep /$  
# echo "`ls -F * |grep /$`"  
for d in `ls -F ~/ORB_SLAM/Model |grep /$|wc -l`

do

#   IMAGE_DIR = "$d"
echo "$d"
  cd $BIN_PATH/$d
  NUM = `ls -ls -|*|grep "^-"| wc -|`
  cd ..
  echo "NUM"
done
