#!/usr/bin/env python3
import rospy
from datetime import datetime,timedelta
from std_msgs.msg import String
import sqlite3 as sql
from rasa_ros.chatbot.actions.actions import get_connetion

CREATE_TABLE_QUERY = """CREATE TABLE todolist(
	tag INTEGER PRIMARY KEY AUTOINCREMENT,
  	user varchar(255) NOT NULL,
	category varchar(255),
	activity varchar(255),
	deadline datetimeoffset,
    reminder boolean,
	unique(user,activity,category)
);
"""

# def get_connetion(path = "Music\\database.db"):
#     con = sql.connect(path)
#     cur = con.cursor()

#     try:
#         cur.execute("SELECT * FROM todolist")
#     except sql.OperationalError as e:
#         print("Table creation...")
#         cur.execute(CREATE_TABLE_QUERY)
    
#   return con, cur

def callback(username):
    pub = rospy.Publisher('give_reminder', String, queue_size=10)
    rospy.init_node('reminder_node', anonymous=True)
    
    con, cur = get_connetion()
    str = None
    query = f"select deadline from todolist where reminder=true and user='francesco'"
    print(query)
    res = cur.execute(query)
    print(res)
    tmp = res.fetchall()
    print(tmp)
    if len(tmp) == 0:
        print("ciao")
        return None
    print(tmp)
    for el in tmp:
        deadline = datetime.strptime(el,'%Y-%m-%dT%H:%M:%S.%fZ')
        print(deadline)
        if (deadline - timedelta(hours=1) < datetime.now()):
            str += el + "\n"
    con.close()
    print(str)
    pub.publish(str)
    return str

def listener():
    stringa = rospy.Subscriber("give_reminder", String, callback('francesco'))
    rospy.spin()

if __name__ == '__main__':
    listener()