import asyncio
import mcp.types as types
from mcp.server import Server
from auto_mcp import MCPSSE
import uvicorn

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_interface


app = Server("example-server")
sse = MCPSSE(app)


@app.list_resources()
async def list_resources() -> list[types.Resource]:
    return [types.Resource(uri="example://resource", name="Example Resource")]


@app.read_resource()
async def read_resource(url) -> str:
    print(f"Reading resource at {url}")
    return "This is an example resource."


async def main():
    config = uvicorn.Config("server:sse", host="0.0.0.0", port=5000, log_level="debug")
    server = uvicorn.Server(config)
    await server.serve()
    # async with stdio_server() as streams:
    #     await app.run(
    #         streams[0],
    #         streams[1],
    #         app.create_initialization_options()
    #     )


if __name__ == "__main__":
    asyncio.run(main())
