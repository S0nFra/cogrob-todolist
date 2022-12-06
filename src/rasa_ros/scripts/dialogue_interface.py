#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse
from std_msgs.msg import Int16MultiArray, String

class TerminalInterface:
    '''Class implementing a terminal i/o interface. 

    Methods
    - get_text(self): return a string read from the terminal
    - set_text(self, text): prints the text on the terminal

    '''

    def get_text(self):
        return input("[IN]:  ") 

    def set_text(self,text):
        print("[OUT]:",text)

class S2TInterface():
    
    def __init__(self):
        self.changed = False
    
    def callback(self, data:String):
        self.message = data.data
        self.changed = True
    
    def get_text(self):
        print("[IN]:", self.message)
        self.changed = False
        return self.message

    def set_text(self,text):
        print("[OUT]:",text)

def main():
    rospy.init_node('writing')
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)

    # terminal = TerminalInterface()
    terminal = S2TInterface()
    rospy.Subscriber("voice_txt", String, terminal.callback)
    print('[READY]')
    
    while not rospy.is_shutdown():
        if terminal.changed:
            message = terminal.get_text()
            if message == 'exit': 
                break
            try:
                bot_answer = dialogue_service(message)
                terminal.set_text(bot_answer.answer)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass