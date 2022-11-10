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
	deadline datetimeoffset,
    reminder boolean,
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

def check_exists_activity(cur,username=None, category=None, activity=None, tag = None):
    if tag is not None:
        query = f"select * from todolist where tag=\'{tag}\'"
    else:
        query = f"select * from todolist where user=\'{username}\' and category = \'{category}\' and activity = \'{activity}\'"
    
    res = cur.execute(query)
    return len(res.fetchall()) != 0
    
def reset_slots(slots = ["category","activity","deadline","reminder","logical"]):
    to_reset = [SlotSet(slot, None) for slot in slots]
    return to_reset

class ActionInsert(Action):
    
    def name(self) -> Text:
        return "action_insert"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        # take current user
        username = tracker.get_slot('username').lower()
        print("\n> Username:", username)
        
        # load needed slots
        category = tracker.get_slot("category").lower()
        activity = tracker.get_slot("activity").lower()
        deadline = tracker.get_slot("deadline")
        logical = tracker.get_slot("logical")
        reminder = tracker.get_slot("reminder")

        con, cur = get_connetion()

        if logical == False:
            query = f"insert into todolist(user, category, activity, deadline, reminder) values(\'{username}\',\'{category}\',\'{activity}\', \'None\', False)"
            print(query)
            
            try:
                res = cur.execute(query)
            except sql.OperationalError as e:
                dispatcher.utter_message(text = "Somethigs goes wrong! maybe already exists")
                return reset_slots()
            except sql.IntegrityError as err:
                dispatcher.utter_message(text = "This activity already exists, try something else")
                return reset_slots()
            con.commit()
            con.close()
            dispatcher.utter_message(text = "Perfect! I have added \"" + activity + "\" in \"" + category + "\" ")
            
            return reset_slots()
        else:
            query = f"insert into todolist(user, category, activity, deadline, reminder) values(\'{username}\',\'{category}\',\'{activity}\',\'{deadline}\',\'{reminder}\')"
            print(query)

            try:
                res = cur.execute(query)
            except sql.OperationalError as e:
                dispatcher.utter_message(text = "Somethigs goes wrong! maybe already exists")
                return reset_slots()
            except sql.IntegrityError as err:
                dispatcher.utter_message(text = "This activity already exists, try something else")
                return reset_slots()
            con.commit()
            con.close()
            
            dispatcher.utter_message(text = "Perfect! I have added \"" + activity + "\" in \"" + category + "\" with deadline \"" +str(deadline) + "\"" + "\" and reminder setted to \"" +str(reminder) + "\"")
            
            return reset_slots()

class ActionRemove(Action):
    
    def name(self) -> Text:
        return "action_remove"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        # take current user
        username = tracker.get_slot('username').lower()
        print("\n> Username:", username)
        
        # load needed slots
        category = tracker.get_slot("category")
        activity = tracker.get_slot("activity")
        con, cur = get_connetion()

        # Remove category
        if category is not None and activity is None:
            category.lower()
            query = f"delete from todolist where user=\'{username}\' and category = \'{category}\'"
            print(query)

            res = cur.execute(query)

            # cur.rowcount -> returns the number of rows involved in the last query
            if cur.rowcount == 0:
                dispatcher.utter_message(text = "Something wrong, maybe no category")
                return [SlotSet("category", None)]
            con.commit()
            con.close()
            
            dispatcher.utter_message(text = "Perfect "+ username + "! I have deleted the category \"" + category + "\"")
            return [SlotSet("category", None)]
        elif category is None and activity is not None:
            dispatcher.utter_message(text = "Sorry, you have insert the activity but you haven't specify the category. Please rewrite the phrase specifying the category.")
            return reset_slots()
        # Remove an activity
        query = f"delete from todolist where user=\'{username}\' and category = \'{category}\' and activity = \'{activity}\'"
        print(query)

        res = cur.execute(query)

        if cur.rowcount == 0:
            dispatcher.utter_message(text = "Something wrong, maybe no activity")
            return reset_slots()

        con.commit()
        con.close()
        dispatcher.utter_message(text = "Perfect "+ username + ", I have deleted the activity \"" + activity + "\" from the category \"" + category + "\"")
        
        return reset_slots()

class ActionShow(Action):
    
    def name(self) -> Text:
        return "action_show"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        # take current user
        username = tracker.get_slot('username').lower()
        print("\n> Username:", username)
        
        category = tracker.get_slot("category")
        con, cur = get_connetion()

        # given the category show me all the activities contained in it
        if category is not None:
            category = category.lower()
            query = f"select tag, activity, deadline, reminder from todolist where user=\'{username}\' and category = \'{category}\'"
            print(query)

            res = cur.execute(query)
            tmp = res.fetchall()
            if len(tmp) == 0:
                dispatcher.utter_message(text = f"Something wrong, maybe no activities in {category}")
                return reset_slots()
            
            dispatcher.utter_message(text = f"Ok {username}, showing activities in \"{category}\"")
            for col in tmp:
                dispatcher.utter_message(text = "Tag: " + str(col[0]) + "\tactivity: " + str(col[1]) + "\tdeadline: " + str(col[2]) + "\treminder: " + str(col[3]))

            con.close()
            return reset_slots()           

        # show me all categories for the current user
        query = f"select category from todolist where user=\'{username}\'"
        print(query)
        res = cur.execute(query)
        tmp = set(res.fetchall())
        if len(tmp) == 0:
            dispatcher.utter_message(text = "Something wrong, maybe no categories")
            return reset_slots()
        
        dispatcher.utter_message(text = f"Ok {username}, showing your category")
        for i,col in enumerate(tmp):
              dispatcher.utter_message(text = str(i+1) + " " + str(col[0]))
        
        con.close()
        return reset_slots()

class ActionUpdate(Action):
    
    def name(self) -> Text:
        return "action_update"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        # take current user
        username = tracker.get_slot('username').lower()
        print("\n> Username:", username)
        
        # load needed slots
        category = tracker.get_slot("category")
        tag = tracker.get_slot("number")
        activities = tracker.get_latest_entity_values("activity")
        con, cur = get_connetion()

        # update activity by (username, category, activity)
        if tag is None:
            category.lower()
            for activity in activities:
                activity = activity.lower()
                if check_exists_activity(cur, username, category, activity):
                    old_activity = activity
                else:
                    new_activity = activity
            query = f"update todolist set activity=\'{new_activity}\' where user=\'{username}\' and category =\'{category}\' and activity = \'{old_activity}\'"
            msg = f"The {old_activity} activity has been replaced successfully with {new_activity} activity"

        # update activity by tag
        elif check_exists_activity(cur, tag=tag):
            new_activity = next(activities).lower()
            query = f"update todolist set activity=\'{new_activity}\' where tag=\'{tag}\'"
            msg = f"The activity tagged with {tag} has been replaced successfully with {new_activity} activity"
        
        print(query)
        if query is None:
            dispatcher.utter_message(text = "Somesthing wrong, maybe need more informations\nTry:\nTag + new activity OR category + old activity + new activity")
        
        res = cur.execute(query)
        con.commit()
        con.close()
        dispatcher.utter_message(text = msg)
        return reset_slots()
