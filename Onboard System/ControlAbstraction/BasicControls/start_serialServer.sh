#!/bin/bash

if [[ ! $1 ]]; then
    sock="/dev/ttyUSB0"
    socklock="/dev/ttyUSB0.lock"
else 
    sock=$1
    socklock="/tmp/vsock.lock"
fi

if [[ ! $2 ]]; then
    port=54321
else 
    port=$2
fi
socat -d -d -d tcp-l:$port,reuseaddr,fork file:$sock,raw,nonblock,waitlock=$socklock,b115200,echo=0,icanon=1,crnl &