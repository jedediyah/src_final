#!/bin/bash

#python -m SimpleHTTPServer 8000 &
#python echoserver.py & 

python wfield_receiver.py &
python whalers_field_control.py &

roscore -p 8001


