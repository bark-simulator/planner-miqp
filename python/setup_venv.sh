#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
mkdir -p $DIR/venv
virtualenv --system-site-packages -p python3.7 $DIR/venv
source $DIR/venv/bin/activate && pip3 install -r $DIR/requirements.txt
