#!/bin/bash
python -m SimpleHTTPServer 8000 &
python echoserver.py & 
roscore -p 8001
