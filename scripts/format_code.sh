#!/bin/sh

black --line-length 120 $PROJECT_DIR/src/enamour

flake8 --max-line-length=120 --extend-ignore=E203,E501,F841 --select=C,E,F,W,B,B950 $PROJECT_DIR/src/enamour
