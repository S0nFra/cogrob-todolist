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

CREATE_TABLE_QUERY = """CREATE TABLE todolist(
	tag INTEGER PRIMARY KEY AUTOINCREMENT,
  	user varchar(255) NOT NULL,
	category varchar(255),
	activity varchar(255),
	deadline datetime,
	unique(user,activity,category)
);
"""

def get_connetion(path = DB_PATH):
    con = sql.connect(path)
    cur = con.cursor()

    try:
        cur.execute("SELECT * FROM todolist")
    except sql.OperationalError as e:
        print("Table creation...")
        cur.execute(CREATE_TABLE_QUERY)
    
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
        query = f"insert into todolist(user, category, activity, deadline) values(\'{username}\',\'{category}\',\'{activity}\',0)"
        try:
            cur.execute(query)
        except sql.OperationalError as e:
            dispatcher.utter_message(text = "Somethigs goes wrong! maybe empty database")
        except sql.IntegrityError as err:
            dispatcher.utter_message(text = "This activity already exists, try something else")
            return [SlotSet("category", None), SlotSet("activity", None), SlotSet("deadline", None)]
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
        activity = tracker.get_slot("activity")
        con, cur = get_connetion()

        # Elimina una categoria
        if category is not None and activity is None:
            query = f"delete from todolist where user=\'{username}\' and category = \'{category}\' "
            res = cur.execute(query)
            tmp = res.fetchall()
            if len(tmp) == 0:
                dispatcher.utter_message(text = "Something wrong, maybe no category")
                return [SlotSet("category", None)]
            con.commit()
            con.close()
            print(query)
            dispatcher.utter_message(text = "Perfect \""+ username + "\"! I have deleted the category \"" + category + "\"")
            return [SlotSet("category", None)]

        # elimina una attività in una categoria
        query = f"delete from todolist where user=\'{username}\' and category = \'{category}\' and activity = \'{activity}\'"
        res = cur.execute(query)
        tmp = res.fetchall()
        if len(tmp) == 0:
            dispatcher.utter_message(text = "Something wrong, maybe no activity")
            return [SlotSet("category", None), SlotSet("activity", None), SlotSet("deadline", None)]

        con.commit()
        con.close()
        print(query)
        dispatcher.utter_message(text = "Perfect \""+ username + "\" I have deleted the activity \"" + activity + "\" from the category \"" + category + "\"")
        
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
        con, cur = get_connetion()

        # data la categoria mostrami tutte le attività in essa contenute
        if category is not None:
            query = f"select tag, activity, deadline from todolist where user=\'{username}\' and category = \'{category}\'"
            res = cur.execute(query)
            tmp = res.fetchall()
            if len(tmp) == 0:
                dispatcher.utter_message(text = "Something wrong, maybe no activities")
                return [SlotSet("category", None)] 
            
            dispatcher.utter_message(text = f"Ok {username}, showing activities in {category}")
            for col in tmp:
                dispatcher.utter_message(text = "Tag: " + str(col[0]) + "\t activity: " + str(col[1]) + "\t deadline " + str(col[2]))
            con.commit()
            con.close()
            print(query)
            return [SlotSet("category", None)]            

        # mostrami tutte le categorie per l'unte corrente
        query = f"select category from todolist where user=\'{username}\'"
        res = cur.execute(query)
        tmp = set(res.fetchall())
        if len(tmp) == 0:
            dispatcher.utter_message(text = "Something wrong, maybe no categories")
            return [SlotSet("category", None), SlotSet("activity", None), SlotSet("deadline", None)]
        
        dispatcher.utter_message(text = f"Ok {username}, showing your category")
        for i,col in enumerate(tmp):
              dispatcher.utter_message(text = str(i+1) + " " + str(col[0]))
        print(query)
        
        return [SlotSet("category", None), SlotSet("activity", None), SlotSet("deadline", None)]

class ActionResetUser(Action):
    
    def name(self) -> Text:
        return "action_reset_user"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        return [SlotSet("username", None)]  