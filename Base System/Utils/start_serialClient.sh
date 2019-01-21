#!/bin/bash

pkill -f socat

if [[ ! $1 ]]; then
    ip=192.168.43.106
else 
    ip=$1
fi

if [[ ! $2 ]]; then
    sock="/dev/ttyA0"
else 
    sock=$2
fi

if [[ ! $3 ]]; then
    port=54321
else 
    port=$3
fi

socat -d -d pty,link=$sock,waitslave tcp:$ip:$port
