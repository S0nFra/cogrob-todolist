# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


# This is a simple example for a custom action which utters "Hello World!"

from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher

class ActionHelloWorld(Action):

    def name(self) -> Text:
        return "action_hello_world"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        dispatcher.utter_message(text="Hello World!")

        return []

class ActionInsert(Action):
    
    def name(self) -> Text:
        return "action_insert"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        print("\n Ciao:")
        # prendere lo username
        username = tracker.get_slot('username')
        username = username.lower()

        print("\n Username:", username)
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

        deadline = tracker.get_slot("deadline")
        if deadline is None:
            dispatcher.utter_message(text = "Do you want insert a deadline?") 
            return []
        
        dispatcher.utter_message(text = "Tutto apposto")
        query = f"insert into todolist(user, category, activity, deadline) values(\'{username}\',\'{activity}\',\'{category}\',{deadline})"
          
        return []
        # query di inserimento al server