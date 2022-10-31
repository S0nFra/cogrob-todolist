#!/usr/bin/env python3
from rasa_node.srv import *
import rospy

def insert(user, activity, category='general', deadline=0):
    rospy.wait_for_service('database')

    query = f"insert into todolist(user, category, activity, deadline) values(\'{user}\',\'{activity}\',\'{category}\',{deadline})"

    try:
        # create a handle to the operations service
        handle = rospy.ServiceProxy('database', database)
        
        print("Query: %s " % (query))

        # formal style
        resp = handle.call(databaseRequest(query))

        if not resp.resp == 'OK':
            raise Exception("Failure, returned result was %s" % resp.resp)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == '__main__':
    data = [('Francesco','vai a caccare','general',0),
('Francesco','compra il latte','spesa',0),
('Francesco','pomodori','spesa',0),
('Mario','fai benzina','macchina',0)]
    
    for a,b,c,d in data:
        insert(a,b,c,d)
   