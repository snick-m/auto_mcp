from mcp.server.sse import SseServerTransport
from mcp.server import Server


class MCPSSE:
    """
    A class to handle Server-Sent Events (SSE) for the MCP server.
    This class is responsible for managing the connection and communication
    with the SSE clients.
    """

    def __init__(self, app: Server, sse_path: str = "/sse", post_path: str = "/messages"):
        """
        Initialize the MCPSSE instance.

        :param app: The MCP server application instance.
        :param sse_path: The path for the SSE connection.
        :param post_path: The path for posting messages.
        """
        self.sse_path = sse_path
        self.post_path = post_path
        self.sse = SseServerTransport(self.post_path)
        self.app = app

    async def __call__(self, scope, receive, send):
        """
        Call the appropriate handler based on the scope.

        :param scope: The ASGI scope.
        :param receive: The ASGI receive channel.
        :param send: The ASGI send channel.
        """
        if scope["type"] == "http":
            if scope["path"] == self.sse_path:
                await self.handle_sse(scope, receive, send)
            elif scope["path"] == self.post_path:
                await self.handle_messages(scope, receive, send)

    async def handle_sse(self, scope, receive, send):
        """
        Handle the SSE connection.

        :param scope: The ASGI scope.
        :param receive: The ASGI receive channel.
        :param send: The ASGI send channel.
        """
        async with self.sse.connect_sse(scope, receive, send) as streams:
            await self.app.run(
                streams[0], streams[1], self.app.create_initialization_options()
            )

    async def handle_messages(self, scope, receive, send):
        """
        Handle incoming messages from the SSE clients.

        :param scope: The ASGI scope.
        :param receive: The ASGI receive channel.
        :param send: The ASGI send channel.
        """
        await self.sse.handle_post_message(scope, receive, send)
