#!/bin/bash

roscore -p 8001 &
sleep 5

python -m SimpleHTTPServer 8000 &
#python echoserver.py & 

python wfield_receiver.py &
python whalers_field_control.py &


