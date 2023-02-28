# Check config.py file befoure debugging
# $1: pepper_ip ID, $2: flask_ip
source setup.sh

echo -n "Running Flask server... "
gnome-terminal --tab -- python3 src/tablet_pkg/flask_server/app.py -d
echo "DONE"

if [ $1 -ne -1 ]; # $1 = -1 means that you want run app without Pepper
then
    echo -n "Running pepper nodes... "
    gnome-terminal --tab -- rgnome-terminal --tab -- roslaunch pepper_nodes pepper_bringup.launch nao_ip:=$1
    sleep 5
    echo "DONE"

    echo -n "Running tablet node... "
    gnome-terminal --tab -- roslaunch tablet_pkg tablet.launch flask_ip:=$2
    sleep 1
    echo "DONE"
fi

echo -n "Running chatbot node... "
gnome-terminal --tab -- roslaunch rasa_ros chatbot.launch
sleep 10
echo "DONE"

echo -n "Running dialog interface... "
gnome-terminal --window -- rosrun rasa_ros dialogue_interface.py
sleep 50
echo "DONE"

echo -n "Running audio nodes... "
gnome-terminal --window -- roslaunch ros_audio_pkg audio.launch
echo "DONE"