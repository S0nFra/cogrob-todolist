#!/usr/bin/python3

import rospy
from pepper_nodes.srv import LoadUrl, LoadUrlRequest, LoadUrlResponse
from tablet_pkg.msg import ShowData
from std_msgs.msg import String

from config import * 

class Handler:
    '''
    The constructor creates the service proxy object, which is able to display the desired URL on the tablet.
    '''
    def __init__(self):
        self.tablet_service = rospy.ServiceProxy("load_url", LoadUrl)

    '''
    This method calls the tablet service and sends it the URL of the web page to be displayed.
    '''
    def load_url(self, url):
        msg = LoadUrlRequest()
        msg.url = url
        resp = self.tablet_service(msg)
        rospy.loginfo(resp.ack)

handler = Handler()

def callback(data):
    user, category = data.data.split('#')
    # user = data.user
    # category = data.category
    url = ""

    if category != "":
        url = "http://10.0.1.215:5000/show?user={}&category={}".format(user, category)
    else:
        url = "http://10.0.1.215:5000/show?user={}&category=all".format(user)

    handler.load_url(url)
    print(url)

if __name__ == "__main__":
    NODE_NAME = "tablet_manager_node"
    print(NODE_NAME)
    rospy.init_node(NODE_NAME)
    rospy.Subscriber('show_data', String, callback=callback)
    rospy.spin()