#!/usr/bin/env python3
from rasa_node.srv import *
import rospy

def update(user, activity, category='general', deadline=0):
    rospy.wait_for_service('database')
    print('>> Sevice available <<')

    query = f"update todolist set category =\'{category}\',activity=\'{activity}\', deadline={deadline}) where user=\'{user}\'"

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
    update("Francesco","vai a prendere il latte")
