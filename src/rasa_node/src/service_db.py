#!/usr/bin/env python3
from rasa_node.srv import *
import rospy
import sqlite3 as sql

class Database():    

    def __init__(self, db_path = None) -> None:

        self.DB_PATH = db_path if db_path is not None else '.'
        self._query_stack = {}

        # Database connection
        print("Connection...",end=' ')
        self.con = sql.connect(self.DB_PATH)
        self.cur = self.con.cursor()
        print("DONE")

        # Check table existences
        try:
            self.cur.execute("SELECT * FROM todolist")
        except sql.OperationalError as e:
            print("Table creation...",end=' ')
            self.cur.execute('''CREATE TABLE todolist(
        Tag INTEGER PRIMARY KEY AUTOINCREMENT,
        User varchar(255) NOT NULL,
        Category varchar(255),
        Activity varchar(255),
        Deadline datetime);''')
            print("DONE")

        # Close all
        self.cur.close()
        self.con.close()

        # Node declararation
        rospy.init_node('database_node')

        # Service declaration
        s = rospy.Service("database", database, self._manage_query)

        print(">> RUNNIG <<")

        rospy.spin()

        print("\nSTOPPED")

    def _manage_query(self, req):
        try:            
            self.con = sql.connect(self.DB_PATH)
            self.cur = self.con.cursor()
            res = self.cur.execute(req.query)
            self.con.commit()
            self.con.close()
            print(req.query)
        except Exception as e:
            print(e)
        return databaseResponse(str('OK'))

if __name__ == '__main__':
    db = Database(db_path="/home/sonfra/prj_cogrob/src/database/database.db")