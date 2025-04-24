import asyncio
import uvicorn

from mcp.server.fastmcp import FastMCP

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_interface

app = FastMCP("automcp")
sse = app.sse_app

topics = {}
messages = {}


@app.resource("automcp://resource")
async def resource() -> str:
    return "This is an example resource."


@app.resource("automcp://topics", name="Topics", mime_type="application/json")
async def get_topics() -> str:
    print(f"Topics: {topics}")
    return topics


@app.resource("automcp://interface", name="Interface", mime_type="text/plain")
async def interface() -> str:
    # Implement interface details here
    return "Interface details"


def run_introspect():
    """
    Introspects the ROS2 topics and creates resources for each topic.
    """
    rclpy.init()
    node = Node("automcp_introspect")
    node.get_logger().info("Introspecting ROS2 topics...")

    found_topics = node.get_topic_names_and_types()
    node.get_logger().info(f"Total topics found: {len(found_topics)}")
    for t in found_topics:
        try:
            # Store the topic name and its interface in global topics dict
            topics[t[0]] = t[1][0]

            # Using a closure to create a resource for each topic
            # This is necessary to avoid late binding issues in the loop
            # and ensure that the correct topic name is used in the resource.
            def resource_closure(topic_name):
                @app.resource(
                    f"automcp://message/{t[0]}", name=t[0], mime_type="text/plain"
                )
                def f():
                    print(f"Getting message for topic {topic_name}")
                    return messages.get(topic_name, "")

            resource_closure(t[0])
        except Exception as e:
            print(f"Error: {e}")

    rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()


def msg_callback_factory(topic: str):
    """
    Callback factory for creating a message callback for a specific topic.
    Again necessary to avoid late binding issues in the loop.

    :param topic: The name of the topic.
    :return: A function that sets the message for the topic.
    """

    def set_message(msg):
        """
        Sets the message for the given topic in the global messages dictionary.
        This function is called whenever a new message is received on the topic.

        :param msg: The message received on the topic.
        """
        messages[topic] = msg

    return set_message


async def run_mcp():
    """
    Runs the MCP server with the SSE app using Uvicorn.
    """
    config = uvicorn.Config(sse, host="0.0.0.0", port=5000, log_level="debug")
    server = uvicorn.Server(config)
    await server.serve()


async def run_listener():
    """
    Runs the ROS2 listener node that subscribes to the topics.
    """

    rclpy.init()
    node = Node("automcp_listener")

    # Create subscriptions for each topic in the global topics dict
    for t, interface in topics.items():
        try:
            node.create_subscription(
                get_interface(interface), t, msg_callback_factory(t), 10
            )
        except Exception as e:
            print(f"Error creating subscription for topic {t}: {e}")

    # Spin the node to keep it alive and process incoming messages
    while rclpy.ok():
        await asyncio.sleep(0.001)
        rclpy.spin_once(node, timeout_sec=0)

    rclpy.shutdown()


if __name__ == "__main__":
    run_introspect()
    asyncio.run(asyncio.wait([run_mcp(), run_listener()]))
