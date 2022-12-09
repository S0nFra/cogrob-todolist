#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse
from std_msgs.msg import Int16MultiArray, String

from ros_audio_pkg.srv import *

PEPPER = False
LANGUAGE = 'en-GB' # it-IT

def t2s(text):
    if PEPPER:
        # parla pepper
        pass
    else:
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
    rospy.init_node('dialog_interface')
    
    ## Servizi per conversazione con il chatbot
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
    
    ## Servizi per re-identification
    # rospy.wait_for_service('get_predicted_identity')
    # get_predicted_identity = rospy.ServiceProxy('get_predicted_identity', Dialogue)
    
    # rospy.wait_for_service('set_current_user')
    # set_current_user = rospy.ServiceProxy('set_current_user', Dialogue)
    
    # rospy.wait_for_service('reset_user')
    # reset_user = rospy.ServiceProxy('reset_user', Dialogue)
    
    # rospy.Subscriber("voice_txt", String, terminal.callback)
    terminal = S2TInterface()
    print('[READY]')
    
    while not rospy.is_shutdown():
        id = rospy.wait_for_message("predicted_identity", String)
        print('>>>',id.data)
        if id.data != '':
            dialogue_service('Hi')
            dialogue_service(f"I'm {id.data}")
        else:
            bot_answer = dialogue_service('Hi')
            # t2s(bot_answer)
            user = rospy.wait_for_message("voice_txt", String)
            print('0>>',user)
            user = user.data.split(' ')[-1]
            print('1>>',user)
            # TODO: Inserire la pubblicazione dello user sul topic
              
        session = True        
        while session:
            message = rospy.wait_for_message("voice_txt", String)
            if message.data == 'exit':
                break
            if 'bye' in message.data:
                session = False
                # TODO: far resettare lo user per accoglierne uno nuovo
            try:
                bot_answer = dialogue_service(message.data)
                terminal.set_text(bot_answer.answer)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
    
    # bot_answer = dialogue_service(f"I'm {id}")
        
    # while not rospy.is_shutdown():
    #     if terminal.changed:
    #         message = terminal.get_text()            
    #         if message == 'exit': 
    #             break            
    #         try:
    #             bot_answer = dialogue_service(message)
    #             terminal.set_text(bot_answer.answer)
    #         except rospy.ServiceException as e:
    #             print("Service call failed: %s"%e)

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass