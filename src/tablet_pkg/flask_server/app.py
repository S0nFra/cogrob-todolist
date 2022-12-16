from flask import Flask, render_template, request
import sqlite3
import os

from config import *

app = Flask(__name__)

@app.route("/show", methods=['GET','POST'])
def show():
        
    con = sqlite3.connect(DB_PATH)
    cursor = con.cursor()
    print("Successfully Connected to SQLite")
    
    user=request.args.get('user')
    category=request.args.get('category')
    
    if category != "all": 
        sqlite_search_query = "select tag, activity, deadline, reminder from todolist where user= ? and category = ? "
        cursor.execute(sqlite_search_query, (user, category, ))
    else: 
        sqlite_search_query = "select category from todolist where user= ? "
        cursor.execute(sqlite_search_query, (user, ))
        
    data = cursor.fetchall()
    con.close()
    
    #return render_template('index.html', data =project_home)

    return render_template('index.html', data=set(data))

if __name__=="__main__": 
    app.run(host="0.0.0.0")
