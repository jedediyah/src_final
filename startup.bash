#!/bin/bash
python -m SimpleHTTPServer 8000 &
roscore -p 8001
