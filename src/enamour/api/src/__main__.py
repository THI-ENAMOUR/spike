#!/usr/bin/env python2.7
import json
import signal
import threading
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

import rospy
from std_msgs.msg import String

from api_error import ApiError


class RequestHandler(BaseHTTPRequestHandler):
    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "X-Requested-With")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Headers", "access-control-allow-origin")
        self.end_headers()

    def do_POST(self):
        if self.path == "/intent":
            print("Received intent post request")
            content_len = int(self.headers.getheader("content-length", 0))
            post_body = self.rfile.read(content_len)
            if not self.is_valid_json(post_body):
                error = ApiError(
                    title="Invalid request payload format",
                    message="The payload is no valid json",
                    http_status=400,
                )
                print("Received data is not of a valid json format: " + str(post_body))
                return self.sendResponse(error.to_json(), error.status)

            publish_intent(post_body)
            return self.sendResponse(None, 204)

    @staticmethod
    def is_valid_json(data):
        try:
            json.loads(data)
            return True
        except ValueError:
            return False

    def sendResponse(self, res, status):
        print("Sending response " + str(res))
        self.send_response(status)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        if res is not None:
            self.wfile.write(res)
        return


def publish_intent(intent):
    print("Publishing intent: " + str(intent))
    pub.publish(intent)


API_PORT = 5000
rospy.init_node("intent_api_service")
pub = rospy.Publisher("intent", String, queue_size=10)
server = HTTPServer(("", API_PORT), RequestHandler)


def on_shutdown(*args):
    """Shuts down the api server"""
    # The *args parameter is required, do not remove. The shutdown function needs to be executed in a new thread,
    # else it deadlocks. See shutdown() documentation.
    shutdown_process = threading.Thread(target=lambda: server.shutdown())
    shutdown_process.start()


# Register the on_shutdown function to be called when the system receives the request to shut down.
signal.signal(signal.SIGINT, on_shutdown)
signal.signal(signal.SIGTERM, on_shutdown)

if __name__ == "__main__":
    print("Starting API Server")
    server.serve_forever()
