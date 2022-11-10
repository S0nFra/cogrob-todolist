# Cognitive Robotics

## ROS setup

Installazione di ROS

```bash
# Setup sources.list and keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Installation and environment setup
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo source /opt/ros/noetic/setup.bash
sudo echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# To use with python3
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo apt install python3-rosdep -y
sudo apt-get install python3-catkin-tools -y
sudo rosdep init
rosdep update
```



## RASA Setup

Installazione di RASA

```bash
sudo apt update
sudo apt install python3-pip
python3 -m pip install pip==22.0.0
python3 -m pip install pyOpenSSL
python3 –m pip install rasa==2.7.2
python3 –m pip install rasa[spacy]
python3 –m pip install rasa[transformers]
```

Installazione del modello utilizzato

```
python3 -m spacy download en_core_web_md
```

Scaricare e preparare Duckling

```bash
# Dependecies
wget -qO- https://get.haskellstack.org/ | sh 
sudo apt install libicu-dev
sudo apt install libpcre3-dev

# Clone repository
cd cogrob-todolist/src/rasa_ros/chatbot 
git clone https://github.com/facebook/duckling
cd cogrob-todolist/src/rasa_ros/chatbot/duckling
stack build
```



## Setup progetto



Rendere gli script di lancio eseguibili

```
chmod u+x cogrob-todolist/src/rasa_ros/scripts/*
```

Una volta recati nella proprio ROS worksapace

```
roslaunch rasa_ros chatbot.launch
```

