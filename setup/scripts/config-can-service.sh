#! /bin/bash

sudo cp ../resources/enable-can.service /etc/systemd/system/

sudo chmod 664 /etc/systemd/system/enable-can.service

sudo chmod 744 ../resources/enable-can.sh

sudo systemctl daemon-reload

sudo systemctl enable enable-can.service