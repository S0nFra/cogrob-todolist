#!/bin/bash

BOT_DIR="/home/mario/Documents/GitHub/cogrob-todolist/src/rasa_ros/chatbot"

cd $BOT_DIR

rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
