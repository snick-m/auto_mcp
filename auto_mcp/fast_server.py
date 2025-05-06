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
publishers = {}

node: Node = None


@app.resource("automcp://topics", name="Topics", mime_type="application/json")
async def get_topics() -> str:
    global node

    node.get_logger().debug(
        f"Returning list of introspected topics - Count: {len(topics)}"
    )
    return topics


@app.tool(name="get_topic_message")
async def get_message(topic: str) -> str:
    """
    Returns the last message received on the specified topic.

    :param topic: The name of the topic.
    :return: The last message received on the topic.
    """
    global node
    
    node.get_logger().debug(f"Returning last received message for topic {topic}")
    return messages.get(topic, "")


def run_introspect():
    """
    Introspects the ROS2 topics and creates resources for each topic.
    """
    global node

    rclpy.init()
    node = Node("automcp_introspect")
    node.get_logger().info("Introspecting ROS2 topics...")

    found_topics = node.get_topic_names_and_types()
    node.get_logger().info(f"Total topics found: {len(found_topics)}")
    for t in found_topics:
        try:
            # Store the topic name and its interface in global topics dict
            topic_name = t[0][
                1:
            ]  # Remove leading slash to stay safe with tool parameter standards (Claude desktop for now)
            topics[topic_name] = t[1][0]
        except Exception as e:
            node.get_logger().error(f"Error: {e}")

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


def dict_to_msg(d: dict, msg_type):
    msg = msg_type()  # Create empty message instance
    node.get_logger().debug(f"Converting dict to message type: {msg_type}")
    types = msg.get_fields_and_field_types()

    for field in types.keys():
        if field in d:
            value = d[field]

            # Handle nested messages (e.g., `header.stamp`)
            if isinstance(value, dict):
                node.get_logger().debug(f"Nested message found for field: {field}")
                field_type = types[field].replace(
                    "/", "/msg/"
                )
                node.get_logger().debug(f"Field types: {field_type}")
                nested = dict_to_msg(value, get_interface(field_type))
                node.get_logger().debug(f"Setting nested field {field} to value {nested}")
                setattr(msg, field, nested)
            else:
                # Parse value to the correct type
                if types[field] == "float":
                    value = float(value)
                elif types[field] == "int":
                    value = int(value)
                elif types[field] == "double":
                    value = float(value)
                elif types[field] == "bool":
                    value = bool(value)
                elif types[field] == "string":
                    value = str(value)
                node.get_logger().debug(f"Setting field {field} of type {types[field]} to value {value}")
                setattr(msg, field, value)
    return msg


async def run_listener():
    """
    Runs the ROS2 listener node that subscribes to the topics.
    """
    global node

    rclpy.init()
    node = Node("auto_mcp")
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    @app.tool(name="send_topic_message")
    async def send_message(topic: str, message: dict) -> str:
        """
        Sends a message to the specified topic.
        Exclude any 0 float values from the message no matter where it is.

        :param topic: The name of the topic.
        :param message: The message to send.
        :return: A confirmation message.
        """

        node.get_logger().info(f"Publishers: {publishers}")
        if topic in publishers:
            typed_message = get_interface(topics[topic])()
            typed_message = dict_to_msg(message, get_interface(topics[topic]))

            publishers[topic].publish(typed_message)
            return f"Message sent to topic {topic}"
        else:
            return f"Topic {topic} not found"

    # Create subscriptions for each topic in the global topics dict
    for t, interface in topics.items():
        try:
            node.create_subscription(
                get_interface(interface), t, msg_callback_factory(t), 10
            )
            publishers[t] = node.create_publisher(
                get_interface(interface), t, 10
            )  # Create a publisher for the topic
        except Exception as e:
            node.get_logger().error(f"Error creating subscription for topic {t}: {e}")

    # Spin the node to keep it alive and process incoming messages
    while rclpy.ok():
        await asyncio.sleep(0.001)
        rclpy.spin_once(node, timeout_sec=0)

    rclpy.shutdown()


def main():
    """
    Main function to run the MCP server and the ROS2 listener.
    """
    # Run the introspection function to populate topics
    run_introspect()

    # Start the MCP server and the listener
    asyncio.run(asyncio.wait([run_mcp(), run_listener()]))


if __name__ == "__main__":
    main()
