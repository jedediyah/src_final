#!/bin/bash

#python wfield_receiver.py &
#python whalers_field_control.py

python -m SimpleHTTPServer 8000 &

python echoserverudp.py


