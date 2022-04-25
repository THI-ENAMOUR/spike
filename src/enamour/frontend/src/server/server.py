import flask
import json
from media_queue import MediaQueue


app = flask.Flask(__name__, static_folder="static")

media_Queue = MediaQueue()

# Mapping between CMD and media files (face-expression.json)
with open("face-expression.json") as expression_json:
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


# Start webserver
# Requirements:
# flask, gunicorn, gevent
# CMD:
# gunicorn --workers 1 --threads 12 -k gevent --bind 0.0.0.0:5000 server:app
