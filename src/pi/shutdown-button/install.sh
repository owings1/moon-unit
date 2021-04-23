#!/bin/bash
set -e
sudo mkdir -p /var/lib/local/shutdown-button
sudo cp shutdown-button.py /var/lib/local/shutdown-button
sudo cp shutdown-button.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable shutdown-button.service
sudo systemctl restart shutdown-button.service
