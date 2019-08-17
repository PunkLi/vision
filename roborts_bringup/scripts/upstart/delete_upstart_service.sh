#!/bin/bash
echo " "
echo "Start to remove script files from /home/dji"
echo ""
rm  /home/dji/roborts-start.sh
echo " "
echo "Start to remove service files from /lib/systemd/system/"
echo ""
sudo rm  /lib/systemd/system/roborts.service
echo " "
echo "Disable roborts service for upstart! "
echo ""
sudo systemctl disable roborts.service
echo "Finish"
