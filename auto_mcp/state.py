import asyncio


class LockedState:
    """
    Class to store the last message received from ROS topic.
    """

    def __init__(self):
        self.messages = {}
        self.topics = {}
        self.lock = asyncio.Lock()

    async def set_last_message(self, topic, message):
        async with self.lock:
            self.messages[topic] = message

    async def get_last_message(self, topic):
        async with self.lock:
            return self.messages.get(topic, "")
    
    def add_topic_sync(self, topic, interface):
        """
        Add a topic and its interface to the state.
        This method is synchronous and should be called from the main thread.
        """
        self.topics[topic] = interface

    async def add_topic(self, topic, interface):
        async with self.lock:
            self.topics[topic] = interface

    async def get_topics(self):
        async with self.lock:
            return self.topics
