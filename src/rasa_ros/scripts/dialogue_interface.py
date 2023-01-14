#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse
from std_msgs.msg import Int16MultiArray, String

from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse
from ros_audio_pkg.srv import *

import os, time
from gtts import gTTS
from playsound import playsound

from config import * 
from reminder import Reminder

class T2SInterface():
    
    def __init__(self):
        self.tts = rospy.ServiceProxy("/tts", Text2Speech)
        rospy.wait_for_service('listen_start')
        self.mic_on = rospy.ServiceProxy('listen_start', ListenStart)    
        rospy.wait_for_service('listen_stop')
        self.mic_off = rospy.ServiceProxy('listen_stop', ListenStop)

    def speech(self, text: str):
        self.mic_off()
        if PEPPER:
            msg = Text2SpeechRequest()
            msg.speech = text
            resp = self.tts(text)
            rospy.sleep(len(text)*CHAR_SPEED)
        else:
            try:
                to_speak = gTTS(text=text, lang=LANGUAGE, slow=False)
                to_speak.save("temp.wav")
                playsound("temp.wav")
                os.remove("temp.wav")
            except AssertionError:
                pass
        print("[OUT]:",text)
        time.sleep(1.5)
        self.mic_on()

def main():
    rospy.init_node('dialog_interface')
    
    ## Servizi per conversazione con il chatbot
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
    print('[CHATBOT] RASA server online')
    
    ## Topic
    pub_current_user = rospy.Publisher('current_user', String, queue_size=3)
    pub_reset_user = rospy.Publisher('reset_user', String, queue_size=3)
    print('[CHATBOT] Topics OK')
    
    ## Classi per integrazione
    t2s = T2SInterface()
    reminder = Reminder(DB_PATH)
    print('[CHATBOT] Classes for integration OK')
    
    print('[CHATBOT] READY')
    
    while not rospy.is_shutdown():
        print('[CHATBOT] Wait for user')
        id = rospy.wait_for_message("predicted_identity", String)
        
        if id.data != '':
            user = id.data
            dialogue_service('Hi')
            bot_answer = dialogue_service(f"I'm {user}")
            t2s.speech(bot_answer.answer)
        else:
            bot_answer = dialogue_service('Hi')
            t2s.speech(bot_answer.answer)
            user = rospy.wait_for_message("voice_txt", String)
            user = user.data.split(' ')[-1]
            pub_current_user.publish(user)
            bot_answer = dialogue_service(f"I'm {user}")
            t2s.speech(bot_answer.answer)
        
        print('[CHATBOT] Current user:',user)
        reminder.set_username(user)
        rem = reminder.remind_me()
        if rem is not None:
            t2s.speech(rem)
              
        session = True
        while session:
            message = rospy.wait_for_message("voice_txt", String)
            print('[IN]:',message.data)
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