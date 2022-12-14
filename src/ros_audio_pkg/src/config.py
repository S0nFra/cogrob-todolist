import os
import pathlib

PEPPER = True
LANGUAGE = 'en-US' # en-GB # it-IT

## Speaker identification variables
REF_PATH = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(REF_PATH,'deep_speaker.h5')
EMBEDDING_FILENAME = os.path.join(REF_PATH,'embedding.pk')
RATE = 16000
TH = 0.75

QUEUE_MAX = 5 # 10

DB_PATH=str(pathlib.Path(__file__).parent.absolute()) + "/../../rasa_ros/database.db"
