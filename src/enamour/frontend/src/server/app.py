#!/usr/bin/env python3

import json
import os
from sys import argv

import flask

from media_queue import MediaQueue
from wsgi_server import WSGIServer

app = flask.Flask(__name__, static_folder="static")
media_Queue = MediaQueue()

# Mapping between CMD and media files (face-expression.json)
__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
expression_json_file = os.path.join(__location__, "face-expression.json")
with open(expression_json_file) as expression_json:
    registry = json.load(expression_json)


@app.route("/", methods=["GET"])
def index():
    return flask.render_template("index.html")


@app.route("/favicon.ico", methods=["GET"])
def fav():
    return flask.url_for("static", filename="favicon.ico")


# Set next facial expression
@app.route("/update-facial-expression", methods=["POST"])
def update():
    try:
        data = flask.request.get_json()
        media = cmd_to_media(data)
        media_Queue.publish(msg=media)
        return {}, 200
    except Exception as e:
        print(e)
        print("error")
        return {}, 400


# Get next facial expression
@app.route("/next-facial-expression", methods=["GET"])
def listen():
    def stream():
        messages = media_Queue.subscribe()  # returns a queue.Queue
        while True:
            msg = messages.get()  # blocks until a new message arrives
            yield msg

    return flask.Response(stream(), mimetype="text/event-stream")


# Convert received command to media files
def cmd_to_media(data):
    dic = json.loads(data)
    try:
        obj = registry[dic["expression"]]
        json_str = json.dumps(obj)
    except KeyError:
        obj = registry["default"]
        json_str = json.dumps(obj)
    return json_str


if __name__ == "__main__":
    default_config = {"bind": "127.0.0.1:5000", "workers": 1, "threads": 8, "k": "gevent"}
    server = WSGIServer(app)
    server.init(opts=default_config, args=argv)
    server.run()
