#! /bin/bash

sudo cp ../resources/00-installer-config.yaml /etc/netplan/

sudo chmod 600 /etc/netplan/00-installer-config.yaml

sudo netplan apply