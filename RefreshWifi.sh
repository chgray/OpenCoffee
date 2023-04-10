#!/bin/sh


#rshell "rm /pyboard.main.py"
#rshell "cp ./WiFiTest.py /pyboard/main.py"

ampy -p $1 put WiFiTest.py main.py
ampy -p $1 reset
