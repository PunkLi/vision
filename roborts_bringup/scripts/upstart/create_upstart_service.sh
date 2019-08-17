#!/bin/bash
echo " "
echo "Start to copy script files to /home/dji"
echo ""
cp `rospack find roborts_bringup`/scripts/upstart/roborts-start.sh /home/dji
cp `rospack find roborts_bringup`/scripts/upstart/max_performance.sh /home/dji
echo " "
echo "Start to copy service files to /lib/systemd/system/"
echo ""
sudo cp `rospack find roborts_bringup`/scripts/upstart/roborts.service  /lib/systemd/system/
echo " "
echo "Enable roborts service for upstart! "
echo ""
sudo systemctl enable roborts.service
echo "Finish "
