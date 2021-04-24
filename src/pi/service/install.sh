#!/bin/bash
sudo cp moon-unit.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable moon-unit.service
sudo systemctl restart moon-unit.service