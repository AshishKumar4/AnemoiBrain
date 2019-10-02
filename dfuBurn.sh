#!/bin/bash
dfu-util -a 0 -D $1 -s 0x08000000:leave