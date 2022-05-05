import json
import queue


class MediaQueue:
    def __init__(self):
        # Array of all registeres subscriber queue
        self.subscribers = []

    def subscribe(self):
        # Queue save messages for subscriber
        q = queue.Queue(maxsize=5)
        self.subscribers.append(q)
        # Get index of new subscriber Queue
        index = len(self.subscribers) - 1
        # Send an initial message to connect to the client
        msg = {"info": "Connection successfully established"}
        self.subscribers[index].put_nowait(self.format_sse(data=json.dumps(msg)))
        return self.subscribers[-1]

    def publish(self, msg):
        # We go in reverse order because we might have to delete an element,
        # which will shift the indices backward
        for i in reversed(range(len(self.subscribers))):
            try:
                self.subscribers[i].put_nowait(self.format_sse(data=msg))
            except queue.Full:
                # Delte queue after five messages not picked up by client
                del self.subscribers[i]

    # "Server-Sent-Events"-message format
    def format_sse(self, data: str, event=None) -> str:
        msg = f"data: {data}\n\n"
        if event is not None:
            msg = f"event: {event}\n{msg}"
        return msg
