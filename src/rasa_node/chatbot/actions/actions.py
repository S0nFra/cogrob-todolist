from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet

DATA = {
    'San Francisco' : {
        "rbry-mqwu": "Street Z, Number Y",
        "b27b-2uc7": "Street X, Number Y",
        "9wzi-peqs": "Street Y, Number X"
    }
}

class ActionSubmit(Action):

    def name(self) -> Text:
        return "action_submit"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        facility_type = tracker.get_slot("facility_type")
        location = tracker.get_slot("location")

        print("\nFacility type:",facility_type)
        print("Location:",location)
        
        if location in DATA and facility_type in DATA[location]:
            dispatcher.utter_message(text=f"This is the address: {DATA[location][facility_type]}") 
            return [SlotSet("facility_address", DATA[location][facility_type])]
        
        dispatcher.utter_message(text="Sorry. I can't find this facility.") 

        return []
