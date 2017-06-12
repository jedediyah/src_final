#!/bin/bash

python -m SimpleHTTPServer 8000 &
python -m echoserver.py & 

#python -m wfield_receiver.py &
#python -m whalers_field_control.py &

roscore -p 8001


