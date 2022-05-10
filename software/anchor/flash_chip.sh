#!/bin/bash

espefuse.py --port /dev/cu.wchusbserial1440 --do-not-confirm set_flash_voltage 3.3V
make flash monitor
