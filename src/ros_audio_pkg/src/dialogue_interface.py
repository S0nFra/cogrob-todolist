#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse
from std_msgs.msg import Int16MultiArray, String

from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse

from config import * 

class T2SInterface():
    
    def __init__(self):
        self.tts = rospy.ServiceProxy("/tts", Text2Speech)

    def speech(self, text: str):
        if PEPPER:
            msg = Text2SpeechRequest()
            msg.speech = text
            resp = self.tts(text)
            rospy.loginfo(resp.ack)
        print("[OUT]:",text)

def main():
    rospy.init_node('dialog_interface')
    
    ## Servizi per conversazione con il chatbot
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
    
    pub_current_user = rospy.Publisher('current_user', String, queue_size=3)
    pub_reset_user = rospy.Publisher('reset_user', String, queue_size=3)
    t2s = T2SInterface()
    
    # rospy.Subscriber("voice_txt", String, terminal.callback)
    # terminal = S2TInterface()
    print('[CHATBOT] READY')
    
    while not rospy.is_shutdown():
        id = rospy.wait_for_message("predicted_identity", String)
        print('>>>',id.data)
        if id.data != '':
            dialogue_service('Hi')
            bot_answer = dialogue_service(f"I'm {id.data}")
            t2s.speech(bot_answer.answer)
        else:
            bot_answer = dialogue_service('Hi')
            t2s.speech(bot_answer.answer)
            user = rospy.wait_for_message("voice_txt", String)
            print('0>>',user)
            user = user.data.split(' ')[-1]
            print('1>>',user)
            pub_current_user.publish(user)
            bot_answer = dialogue_service(f"I'm {user}")
            t2s.speech(bot_answer.answer)
              
        session = True
        while session:
            message = rospy.wait_for_message("voice_txt", String)
            if message.data == 'exit':
                break
            try:
                bot_answer = dialogue_service(message.data)
                t2s.speech(bot_answer.answer)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            if 'bye' in message.data:
                session = False
                id = ''
                user = None
                pub_reset_user.publish('reset')
                print('reset')
                rospy.wait_for_message("predicted_identity", String)
                
if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:

    ## Servizi per re-identification
    # rospy.wait_for_service('get_predicted_identity')
    # get_predicted_identity = rospy.ServiceProxy('get_predicted_identity', Dialogue)
    
    # rospy.wait_for_service('set_current_user')
    # set_current_user = rospy.ServiceProxy('set_current_user', Dialogue)
    
    # rospy.wait_for_service('reset_user')
    # reset_user = rospy.ServiceProxy('reset_user', Dialogue)
    
    
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

# class S2TInterface():
    
#     def __init__(self):
#         self.changed = False
    
#     def callback(self, data:String):
#         self.message = data.data
#         self.changed = True
    
#     def get_text(self):
#         print("[IN]:", self.message)
#         self.changed = False
#         return self.message

#     def set_text(self,text):
#         print("[OUT]:",text)
        pass