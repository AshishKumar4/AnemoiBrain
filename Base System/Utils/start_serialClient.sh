#!/bin/bash

pkill -f socat

if [[ ! $1 ]]; then
    sock="/dev/ttyS0"
else 
    sock=$1
fi

if [[ ! $2 ]]; then
    port=54321
else 
    port=$2
fi

if [[ ! $3 ]]; then
    ip=0.0.0.0
else 
    ip=$3
fi
socat -d -d pty,link=$sock,waitslave tcp:$ip:$port