#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse
from std_msgs.msg import Int16MultiArray, String

from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse

from config import * 
from reminder import Reminder

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
    reminder = Reminder(DB_PATH)
    
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
            # print('0>>',user)
            user = user.data.split(' ')[-1]
            # print('1>>',user)
            pub_current_user.publish(user)
            bot_answer = dialogue_service(f"I'm {user}")
            t2s.speech(bot_answer.answer)
            
        reminder.set_username(user)
        rem = reminder.remind_me()
        if rem is not None:
            t2s.speech(rem)
              
        session = True
        while session:
            message = rospy.wait_for_message("voice_txt", String)
            if message.data == 'exit':
                break
            try:
                bot_answer = dialogue_service(message.data)
                t2s.speech(bot_answer.answer)
            except rospy.ServiceException as e:
                print("[CHATBOT] Service call failed: %s"%e)
            if 'bye' in message.data:
                session = False
                id = ''
                user = None
                pub_reset_user.publish('reset')
                print('[CHATBOT] reset')
                rospy.wait_for_message("predicted_identity", String)
                
            rem = reminder.remind_me()
            if rem is not None:
                t2s.speech(rem)
                
if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass