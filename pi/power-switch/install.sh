#!/bin/bash
set -e
sudo cp powerbuttons.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable powerbuttons.service
sudo systemctl start powerbuttons.service
