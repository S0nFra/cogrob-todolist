# Cognitive Robotics

## ROS Setup

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



### Duckling

Duckling è un extractor per RASA da noi utilizzato per estrarre entità temporali come la deadline. Per maggiori informazioni rimandiamo alla documentazione ufficiale [qui](https://rasa.com/docs/rasa/2.x/components#ducklingentityextractor).

Seguire i seguenti comandi per la configurazione. Attenersi ai path riportati.

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



### Flask Server
Flask è un Web Server in esecuzione sulla rete locale utilizzato per la visualizzazione di attività e categorie di un utente su una pagina HTML. Per maggiori informazioni rimandiamo alla documentazione ufficiale [qui](https://flask.palletsprojects.com/en/2.2.x/).

Istallazione di falsk

```
pip install flask
```



### Rosbridge 
Rosbridge fornisce un'API JSON alla funzionalità ROS per i programmi non ROS. Utilizziamo questo 'nodo fittizio' per far interagire un programma python con l'ambiente ROS. Per maggiori informazioni rimandiamo alla documentazione ufficiale [qui](http://wiki.ros.org/rosbridge_suite).

Istallazione di Rosbridge

```
sudo apt-get install ros-noetic-rosbridge-server
```



## Run project

Rendere gli script di lancio eseguibili se non dovessero esserlo

```bash
chmod u+x cogrob-todolist/src/rasa_ros/scripts/*
```

A questo punto il setup è completo. Per lanciare il progetto dalla workspace ROS eseguire i seguenti comandi:

```bash
cd cogrob-todolist # workspace ROS
source setup.sh
roslaunch rasa_ros chatbot.launch
```

In un altro terminale eseguire i seguenti comandi: 

```bash
source setup.sh
roslaunch ros_audio_pkg audio.launch
```

In un altro terminale: 

```bash
source setup.sh
roslaunch tablet_pkg tablet.launch
```

In un altro terminale: 

```bash
source setup.sh
roslaunch pepper_nodes pepper_bringup.launch
```

Se si vuole visualizzare quello che viene identificato dallo Speeker Recognition, eseguire i seguenti comandi in un altro terminale: 

```bash
source setup.sh
roslaunch ros_audio_pkg speech2text.launch
```



# Contacts - Team 6

| Student            | E-mail | Matricola |
| ------------------ | ------ | --------- |
| Mario Amato        | m.amato72@studenti.unisa.it | 0622701670 |
| Margherita Avitabile   | m.avitabile6@studenti.unisa.it | 0622701825 |
| Lucia Battipaglia| l.battipaglia6@studenti.unisa.it | 0622701758 |
| Francesco Sonnessa | f.sonnessa@studenti.unisa.it | 0622701672 |
