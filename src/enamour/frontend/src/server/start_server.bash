#!/bin/sh
cd /home/lars/Desktop/project/src/enamour/frontend/src/server
gunicorn --workers 1 --threads 8 -k gevent --bind 0.0.0.0:5000 server:app