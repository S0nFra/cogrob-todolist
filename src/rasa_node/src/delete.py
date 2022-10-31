#!/usr/bin/env python3
from rasa_node.srv import *
from multipledispatch import dispatch
import rospy


@dispatch(str)
def delete(user):
    rospy.wait_for_service('database')
    print('>> Sevice available <<')

    query = f"delete from todolist where user=\'{user}\'"

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

@dispatch(int)
def delete(tag):
    rospy.wait_for_service('database')
    print('>> Sevice available <<')

    query = f"delete from todolist where tag=\'{tag}\'"

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

@dispatch(str,str,str)
def delete(user,activity,category):
    rospy.wait_for_service('database')
    print('>> Sevice available <<')

    query = f"delete from todolist where user=\'{user}\' and category = \'{category}\' and activity = \'{activity}\'"

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

#you can also pass data type of any value as per requirement
@dispatch(str,str)
def delete(user,category):
    rospy.wait_for_service('database')
    print('>> Sevice available <<')

    query = f"delete from todolist where user=\'{user}\' and category = \'{category}\'"

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
    delete('Mario','fai benzina')
