#!/bin/bash

sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

sudo ifconfig enxf8e43ba99d1a down
sudo ifconfig enxf8e43ba99d1a up 192.168.123.162 netmask 255.255.255.0

