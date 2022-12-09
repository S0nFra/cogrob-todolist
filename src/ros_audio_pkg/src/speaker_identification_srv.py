#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, String, Float32MultiArray
import numpy as np
import pickle
import os

from identification.deep_speaker.audio import get_mfcc
from identification.deep_speaker.model import get_deep_speaker
from identification.utils import batch_cosine_similarity, dist2id

from config import *

class SpeakerIdentification():

    def __init__(self):
        # Load model
        self.model = get_deep_speaker(os.path.join(REF_PATH,'deep_speaker.h5'))
        
        # Load existring embedding if exists
        self.data = self._load_data()
        self.pred_identity = None
        self.current_user = None
        self.queue = []        
        
    def _load_data(self):
        try:
            with open(os.path.join(REF_PATH, EMBEDDING_FILENAME), 'rb') as fh:
                data = pickle.load(fh)
            
            print('[RE-IDENTIFICATION] Embedding loaded:')
            for u in set(data['y']):
                print('  >',u,data['y'].count(u))
        except Exception as e:
            data = dict()
            data['X'] = list()
            data['y'] = list()
        return data
    
    def _save_data(self):
        try:
            with open(os.path.join(REF_PATH, EMBEDDING_FILENAME), 'wb') as fh:
                pickle.dump(self.data, fh)
        except:
            print("[RE-IDENTIFICATION] Can't save the state.")
        
    def start(self):
        rospy.init_node('speaker_reidentification_node')
        self.pub_predicted_identity = rospy.Publisher("predicted_identity", String, queue_size=10)
        rospy.Subscriber("current_user", String, self._set_current_user)
        rospy.Subscriber("reset_user", String, self._reset_user)
        rospy.Subscriber("voice_data", Int16MultiArray, self._identify)
        rospy.spin()
        
    def _identify(self, data):
        
        # while not rospy.is_shutdown():
        # data = rospy.wait_for_message("voice_data", Int16MultiArray)
        
        audio_data = np.array(data.data)
        
        # to float32
        audio_data = audio_data.astype(np.float32, order='C') / 32768.0
        
        # Processing
        ukn = get_mfcc(audio_data, RATE)
        
        # Prediction
        ukn = self.model.predict(np.expand_dims(ukn, 0))
        
        if len(self.data['X']) > 0:
            print("[RE-IDENTIFICATION] Try...")
            # Distance between the sample and the support set
            emb_voice = np.repeat(ukn, len(self.data['X']), 0)
            cos_dist = batch_cosine_similarity(np.array(self.data['X']), emb_voice)
            # Matching
            self.pred_identity = dist2id(cos_dist, self.data['y'], TH, mode='avg')
            
        if len(self.data['X']) == 0 or self.pred_identity is None:
            
            self.queue.append(ukn[0])
            print('[RE-IDENTIFICATION] Unknown, len(queue):',len(self.queue))
            
            if len(self.queue) > 0 and self.current_user is not None:
                # print('[RE-IDENTIFICATION] Populating embedding')
                for _ in range(len(self.queue)):
                    self.data['X'].append(self.queue.pop(0))
                    self.data['y'].append(self.current_user)
                
                if self.data['y'].count(self.current_user) >= QUEUE_MAX:
                    print('[RE-IDENTIFICATION] Time to save')
                    self._save_data()
                
            elif len(self.queue) >= QUEUE_MAX:
                self.queue.pop(0)            
        
        print('[RE-IDENTIFICATION] Predicted:',self.pred_identity)
        self.pub_predicted_identity.publish(self.pred_identity)
        
        pass
    
    def _get_predicted_identity(self, data):
        return self.pred_identity
    
    def _set_current_user(self, user):
        self.current_user = user.data
        print('[RE-IDENTIFICATION] Current user setted:',user.data)
    
    def _reset_user(self, data):
        
        if len(self.queue) > 0 and self.current_user is not None:
            for _ in range(len(self.queue)):
                self.data['X'].append(self.queue.pop(0))
                self.data['y'].append(self.current_user)
            self._save_data()
        self.current_user = None
        self.pred_identity = None
        
        print('[RE-IDENTIFICATION] User reset')
        print('[RE-IDENTIFICATION] Embedding saved:')
        for u in set(self.data['y']):
            print('  >',u, self.data['y'].count(u))

if __name__ == '__main__':
    try:
        identifcator = SpeakerIdentification()
        print('[RE-IDENTIFICATION] Start')
        identifcator.start()
    except rospy.ROSInterruptException:
        pass