# Check config.py file befoure debugging
# $1: pepper_ip ID, $2: flask_ip
source setup.sh

if [ $1 -ne -1 ]; # $1 = -1 means that you want run app without Pepper
then
    echo -n "Running pepper nodes..."
    gnome-terminal --tab -- rgnome-terminal --tab -- roslaunch pepper_nodes pepper_bringup.launch nao_ip:=$1
    sleep 5
    echo "DONE"
fi
echo -n "Running tablet node..."
gnome-terminal --tab -- roslaunch tablet_pkg tablet.launch flask_ip:=$2
sleep 1
echo "DONE"

echo -n "Running chatbot node..."
gnome-terminal --window -- roslaunch rasa_ros chatbot.launch
sleep 100
echo -n "DONE"

echo "Running audio and re-identification nodes"
gnome-terminal --window -- roslaunch ros_audio_pkg reidentification.launch
