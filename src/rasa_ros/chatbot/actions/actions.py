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
        query = f"select * from todolist where tag = \'{tag}\'"
        res = cur.execute(query)
    else:
        query = f"select * from todolist where user= ? and category = ? and activity = ?"
        res = cur.execute(query,(username,category,activity))

    return len(res.fetchall()) != 0
    
def reset_slots(slots = ["category","activity","deadline","reminder","logical","time"]):
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
            query = f"insert into todolist(user, category, activity, deadline, reminder) values(?,?,?, NULL, 0)"
            print(query)
            
            try:
                res = cur.execute(query,(username,category,activity))
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
            query = "insert into todolist(user,category,activity,deadline,reminder) values(?,?,?,?,?)"
            print(query)

            try:
                res = cur.execute(query,(username,category,activity,deadline,reminder))
            except sql.OperationalError as e:
                dispatcher.utter_message(text = e)
                return reset_slots()
            except sql.IntegrityError as err:
                dispatcher.utter_message(text = "This activity already exists, try something else")
                return reset_slots()
            con.commit()
            con.close()
            
            dispatcher.utter_message(text = "Perfect! I have added \"" + activity + "\" in \"" + category + "\" with deadline \"" +str(deadline) + "\" and reminder setted to \"" +str(reminder) + "\"")
            
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
            query = f"delete from todolist where user= ? and category = ?"
            print(query)

            res = cur.execute(query,(username,category))

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
        query = f"delete from todolist where user= ? and category = ? and activity = ?"
        print(query)

        res = cur.execute(query,(username,category,activity))

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
            query = f"select tag, activity, deadline, reminder from todolist where user= ? and category = ? "
            print(query)

            res = cur.execute(query,(username,category))
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
        query = f"select category from todolist where user= ?"
        print(query)
        res = cur.execute(query,(username,))
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
        activities = list(tracker.get_latest_entity_values("activity"))
        deadline = tracker.get_slot("time")
        con, cur = get_connetion()

        old_activity = None
        new_activity = None
        query = None
        
        # update by (username, category, activity)
        if len(activities) > 1:
            # update activity
            for activity in activities:
                activity = activity.lower()
                if check_exists_activity(cur, username, category, activity):
                    old_activity = activity
                else:
                    new_activity = activity
            if old_activity == None:
                dispatcher.utter_message(text = "There isn't this activity in " + category + "category")
                return reset_slots()
            elif new_activity == None:
                dispatcher.utter_message(text = "Ops, you haven't insert a new activity")
                return reset_slots()
            query = f"update todolist set activity= ? where user= ? and category = ? and activity = ?"
            msg = f"The \"{old_activity}\" activity has been replaced successfully with \"{new_activity}\" activity"
            res = cur.execute(query, (new_activity,username,category,old_activity))
        
        elif deadline is not None:
            # update deadline
            activity = tracker.get_slot("activity")
            if not check_exists_activity(cur, username, category, activity):
                dispatcher.utter_message(text = "No activity with this deadline")
                return reset_slots()
            query = f"update todolist set deadline= ? where user= ? and category = ? and activity = ?"
            msg = f"Previus deadline has been replaced with {deadline}"
            res = cur.execute(query, (deadline, username, category, activity))
        
        else:
            # update reminder
            activity = tracker.get_slot("activity")
            if not check_exists_activity(cur, username, category, activity):
                dispatcher.utter_message(text = "Ops, activity does not exists")
                return reset_slots()
            query = f"select reminder from todolist where user= ? and category = ? and activity = ? and deadline != \'NULL\'"
            res = cur.execute(query, (username, category, activity))
            tmp = res.fetchall()
            if len(tmp) == 0:
                dispatcher.utter_message(text = "Something wrong, maybe no deadline to remind")
                return reset_slots()
            
            # tmp is something like this [(0,)]
            # print(tmp)
            query = f"update todolist set reminder= ? where user= ? and category = ? and activity = ?"
            if tmp[0][0]:      
                tmp = False
                msg = f"Reminder disabled"
            else:
                tmp = True
                msg = f"Reminder enabled"
                
            res = cur.execute(query, (tmp, username, category, activity))
        
        print(query)
        if query is None:
            dispatcher.utter_message(text = "Somesthing wrong, maybe need more informations\nTry:\nTag + new activity OR category + old activity + new activity")

        con.commit()
        con.close()
        dispatcher.utter_message(text = msg)
        return reset_slots()
