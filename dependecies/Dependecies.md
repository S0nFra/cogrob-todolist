# Cognitive Robotics

You can easily set the environment with the following commands otherwise read on for manual setting.

```bash
pip install -r requirements.txt
# or
conda env create -n ENVNAME --file env.yml
```



## ROS Setup

Install ROS

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

Install RASA

```bash
sudo apt update
sudo apt install python3-pip
python3 -m pip install pip==22.0.0
python3 -m pip install pyOpenSSL
python3 –m pip install rasa==2.7.2
python3 –m pip install rasa[spacy]
python3 –m pip install rasa[transformers]
```

Install used NLP model

```
python3 -m spacy download en_core_web_md
```



### Duckling

Duckling is a RASA extractor that we use to extract temporal entities like the deadline. For more information, refer to the official documentation [here](https://rasa.com/docs/rasa/2.x/components#ducklingentityextractor).

Run the following commands for configuration. Follow the paths shown.

```bash
# Dependecies for Duckling
wget -qO- https://get.haskellstack.org/ | sh 
sudo apt install libicu-dev
sudo apt install libpcre3-dev

# Clone repository
cd cogrob-todolist/src/rasa_ros/chatbot 
git clone https://github.com/facebook/duckling
cd cogrob-todolist/src/rasa_ros/chatbot/duckling
stack build
```



## Tablet Setup

### Flask Server

Flask is a Web Server running on the local network used for displaying a user's activities and categories on an HTML page. For more information, refer to the official documentation [here](https://flask.palletsprojects.com/en/2.2.x/).

```
pip install flask
```



### Rosbridge 

Rosbridge provides a JSON API to ROS functionality for non-ROS programs. For more information, refer to the official documentation [here](http://wiki.ros.org/rosbridge_suite).

```bash
sudo apt-get install ros-noetic-rosbridge-server
pip install roslibpy
```



## Audio Setup

```bash
sudo apt-get install libasound-dev ffmpeg portaudio19-dev libportaudio2 libportaudiocpp0
pip3 install --user pyaudio speechrecognition librosa sounddevice python_speech_features scipy
```



# Contacts - Team 6

| Student            | E-mail | Matricola |
| ------------------ | ------ | --------- |
| Mario Amato        | m.amato72@studenti.unisa.it | 0622701670 |
| Margherita Avitabile   | m.avitabile6@studenti.unisa.it | 0622701825 |
| Lucia Battipaglia| l.battipaglia6@studenti.unisa.it | 0622701758 |
| Francesco Sonnessa | f.sonnessa@studenti.unisa.it | 0622701672 |
