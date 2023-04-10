#!/bin/bash

# Launches OpenVPN.
# NOTE: This makes assumptions on where the OpenVPN configurations are found.
OPENVPN_CONFIG_DIR='~/openvpn'
OPENVPN_CONFIG_FILE='spot-static.ovpn'

cd $OPENVPN_CONFIG_DIR
sudo openvpn --config $OPENVPN_CONFIG_FILE