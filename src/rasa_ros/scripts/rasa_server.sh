#!/bin/bash

BOT_DIR="/home/sonfra/cogrob-todolist/src/rasa_ros/chatbots"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
