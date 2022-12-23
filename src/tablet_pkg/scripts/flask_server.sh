#!/bin/bash

TB_PKG=$( cd -- "$(dirname "${0}")" >/dev/null 2>&1 ; pwd -P )

cd "$TB_PKG/../flask_server"
python3 app.py

# /home/sonfra/Documents/GitHub/cogrob-todolist/src/tablet_pkg/scripts
# /home/sonfra/Documents/GitHub/cogrob-todolist/src/tablet_pkg/flask_server/app.py