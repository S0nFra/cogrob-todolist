# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


# This is a simple example for a custom action which utters "Hello World!"

from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet
import pathlib
import sqlite3 as sql

DB_PATH=str(pathlib.Path(__file__).parent.absolute()) + "/../../database.db"

def get_connetion(path = DB_PATH):
    con = sql.connect(path)
    cur = con.cursor()

    try:
        cur.execute("SELECT * FROM todolist")
    except sql.OperationalError as e:
        print("Table creation...",end=' ')
        cur.execute('''CREATE TABLE todolist(
        Tag INTEGER PRIMARY KEY AUTOINCREMENT,
        User varchar(255) NOT NULL,
        Category varchar(255),
        Activity varchar(255),
        Deadline datetime);''')
    
    return con, cur

class ActionInsert(Action):
    
    def name(self) -> Text:
        return "action_insert"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        # prendere lo username
        username = tracker.get_slot('username')
        username = username.lower()

        print("Username:", username)
        
        # controllare che tutte le entità siano presenti
        # se manca un entità richiederne l'inserimento
        category = tracker.get_slot("category")
        if category is None:
            dispatcher.utter_message(text = "Please insert category")
            return []

        activity = tracker.get_slot("activity")
        if activity is None:
            dispatcher.utter_message(text = "Please insert activity")
            return []

        # deadline = tracker.get_slot("deadline")
        # if deadline is None:
        #     dispatcher.utter_message(text = "Do you want insert a deadline?") 
        #     return []
        # print("deadline setted:", deadline)

        con, cur = get_connetion()
        query = f"insert into todolist(user, category, activity, deadline) values(\'{username}\',\'{activity}\',\'{category}\',0)"
        cur.execute(query)
        con.commit()
        con.close()
        
        print(query)

        dispatcher.utter_message(text = "Perfect! I have added \"" + activity + "\" in \"" + category + "\"")
        
        return [SlotSet("category", None), SlotSet("activity", None), SlotSet("deadline", None)]


class ActionRemove(Action):
    
    def name(self) -> Text:
        return "action_remove"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        # prendere lo username
        username = tracker.get_slot('username')
        username = username.lower()

        print("Username:", username)
        
        # controllare che tutte le entità siano presenti
        # se manca un entità richiederne l'inserimento
        category = tracker.get_slot("category")
        if category is None:
            dispatcher.utter_message(text = "Please insert category")
            return []

        activity = tracker.get_slot("activity")
        if activity is None:
            dispatcher.utter_message(text = "Please insert activity")
            return []

        # deadline = tracker.get_slot("deadline")
        # if deadline is None:
        #     dispatcher.utter_message(text = "Do you want insert a deadline?") 
        #     return []
        # print("deadline setted:", deadline)

        query = f"delete from todolist where user=\'{username}\' and category = \'{category}\' and activity = \'{activity}\'"
        
        print(query)

        dispatcher.utter_message(text = "Deleted \"" + activity + "\" in \"" + category + "\"")
        
        return [SlotSet("category", None), SlotSet("activity", None), SlotSet("deadline", None)]

class ActionShow(Action):
    
    def name(self) -> Text:
        return "action_show"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        # prendere lo username
        username = tracker.get_slot('username')
        username = username.lower()

        print("Username:", username)
        
        # controllare che tutte le entità siano presenti
        # se manca un entità richiederne l'inserimento
        category = tracker.get_slot("category")
        if category is None:
            dispatcher.utter_message(text = "Please insert category")
            return []

        activity = tracker.get_slot("activity")
        if activity is None:
            dispatcher.utter_message(text = "Please insert activity")
            return []

        # deadline = tracker.get_slot("deadline")
        # if deadline is None:
        #     dispatcher.utter_message(text = "Do you want insert a deadline?") 
        #     return []
        # print("deadline setted:", deadline)

        query = f"select * from todolist "
        
        print(query)

        dispatcher.utter_message(text = "Showing...")
        
        return [SlotSet("category", None), SlotSet("activity", None), SlotSet("deadline", None)]
