#!/bin/bash
set -e
sudo mkdir -p /var/lib/local/power-switch
sudo cp powerswitch.py /var/lib/local/power-switch
sudo cp powerbuttons.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable powerbuttons.service
sudo systemctl restart powerbuttons.service
