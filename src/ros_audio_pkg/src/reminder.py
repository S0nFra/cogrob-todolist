#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from rasa_ros.chatbot.actions.actions import get_connetion


def callback(username):
    pub = rospy.Publisher('give_reminder', String, queue_size=10)
    rospy.init_node('reminder_node', anonymous=True)
    
    con, cur = get_connetion()
    str = None
    query = f"select deadline from todolist where reminder=True and user={username}"
    res = cur.execute(query)
    tmp = res.fetchall()
    if len(tmp) == 0:
        return None

    for el in tmp:
        str += el + "\n"
    con.close()
    pub.publish(str)
    rospy.spin()

if __name__ == '__main__':
    callback()