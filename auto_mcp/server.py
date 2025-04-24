import asyncio
import json

from pydantic import AnyUrl
import mcp.types as types
from mcp.server import Server
from mcp.server.fastmcp import FastMCP


from auto_mcp import MCPSSE
from auto_mcp import LockedState

import rclpy

from rclpy.node import Node
from rosidl_runtime_py.utilities import get_interface

import uvicorn

app = Server("automcp")
sse = MCPSSE(app)

state = LockedState()


@app.list_resources()
async def list_resources() -> list[types.Resource]:
    return [
        types.Resource(uri="automcp://resource", name="Example Resource"),
        types.Resource(
            uri="automcp://topics",
            name="Topics",
            description="List of available topics and their interfaces.",
        ),
        types.Resource(
            uri="automcp://interface",
            name="Interface",
            description="Get details about a specific interface.",
        ),
    ]


@app.read_resource()
async def read_resource(url: AnyUrl) -> str:
    global state
    print(f"Reading resource at {url}")

    if url.host == "topics":
        t = await state.get_topics()
        print(f"Topics: {t}")
        return [{"content": json.dumps(t), "mime_type": "application/json"}]
    elif url.host == "interface":
        print(url.host)
        # Implement interface details here
        return [{"content": "Interface details", "mime_type": "text/plain"}]
    elif url.host == "resource":
        return [{"content": "This is an example resource.", "mime_type": "text/plain"}]


async def run_mcp(q: asyncio.Queue):
    config = uvicorn.Config(sse, host="0.0.0.0", port=5000, log_level="debug")
    server = uvicorn.Server(config)
    await server.serve()


def run_introspect():
    rclpy.init()
    node = Node("example_node")
    for t in node.get_topic_names_and_types():
        try:
            state.add_topic_sync(t[0], t[1][0])
        except Exception as e:
            print(f"Error: {e}")

    rclpy.spin_once(node, timeout_sec=0.1)
    print("Topics:", state.topics)
    rclpy.shutdown()


def set_message(topic: str, msg):
    print(f"Setting message for topic {topic}: {msg}")
    # TODO: Implement the logic to set the message in the state


async def run_listener(q: asyncio.Queue):
    rclpy.init()
    node = Node("automcp_listener")
    topics = await state.get_topics()
    for topic, interface in topics.items():
        try:
            node.create_subscription(
                get_interface(interface), topic, lambda msg: set_message(topic, msg), 10
            )
        except Exception as e:
            print(f"Error creating subscription for topic {topic}: {e}")

    while rclpy.ok():
        await asyncio.sleep(0.001)
        rclpy.spin_once(node, timeout_sec=0)

    rclpy.shutdown()


if __name__ == "__main__":
    run_introspect()
    q = asyncio.Queue()
    asyncio.run(asyncio.wait([run_mcp(q), run_listener(q)]))
