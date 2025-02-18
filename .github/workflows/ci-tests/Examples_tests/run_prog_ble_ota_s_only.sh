#!/bin/bash

echo "--- acquire lock"

python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 3600 /home/$USER/Workspace/Resource_Share/max32655_0.txt
python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py -l -t 3600 /home/$USER/Workspace/Resource_Share/max32655_1.txt

#GITHUB_WORKSPACE=/home/btm-ci/Workspace/btm-ci-github-runner2/_work/msdk/msdk

cd /home/$USER/Workspace/msdk-main
bash /home/$USER/Workspace/BLE-examples-test/Examples_tests/prog_ble_ota_s_only.sh | tee /home/$USER/Workspace/BLE-examples-test/Examples_tests/test.log

echo "--- release lock"

python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32655_1.txt
python3 /home/$USER/Workspace/Resource_Share/Resource_Share.py /home/$USER/Workspace/Resource_Share/max32655_0.txt
