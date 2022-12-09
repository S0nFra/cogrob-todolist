#!/usr/bin/env python3
import rospy
from datetime import datetime,timedelta
from std_msgs.msg import String
import sqlite3 as sql

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

def get_connetion(path = "Music\\database.db"):
    con = sql.connect(path)
    cur = con.cursor()

    try:
        cur.execute("SELECT * FROM todolist")
    except sql.OperationalError as e:
        print("Table creation...")
        cur.execute(CREATE_TABLE_QUERY)
    
    return con, cur

def callback(username):
    pub = rospy.Publisher('give_reminder', String, queue_size=10)
    rospy.init_node('reminder_node', anonymous=True)
    
    con, cur = get_connetion()
    str = None
    query = f"select activity,deadline from todolist where reminder=true and user={username}"
    res = cur.execute(query)
    tmp = res.fetchall()
    if len(tmp) == 0:
        return None
    for el in tmp:
        deadline = datetime.fromisoformat(el[1])
        if (deadline.replace(tzinfo=None) - timedelta(hours=1) < datetime.now()):
            str += el[0] + "\n"
    con.close()
    pub.publish(str)
    return str

def listener():
    stringa = rospy.Subscriber("give_reminder", String, callback('francesco'))
    rospy.spin()

if __name__ == '__main__':
    listener()