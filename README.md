# Auto MCP ✨🤖
This package works as a drop-in MCP Server for any ROS based system.
Using introspection tools, the `automcp_listener` node picks up the list of topics and their interfaces that are live on the system. It then proceeds to create an SSE over HTTP Transport MCP server at port 5000.

Due to this introspection at start behavior - it should be the last node to run.

## What is it ❔
Essentially, this package will allow you to give your ROS based robot a big LLM brain with very little work.

It works by creating an MCP server that you can connect with your LLM/Client of choice providing your **robot a brain, and the LLM a body**.

Model Context Protocol or MCP for short is a new protocol for LLMs to interact with various tools. Anthropic, the company behind Claude developed this protocol which has recently seen support from OpenAI as well. Learn more about it from [the official website](https://modelcontextprotocol.io)

Now, this Auto MCP ROS2 package aims to make the process of creating an MCP for your robot effortless. Through introspection, this package can detect all the topics running on a ROS system and provide necessary tools. All you have to do is run its node after everything else, and add the configuration to your preferred client.

## Why is it ❕
As LLMs get more and more powerful, I figured many more robotics developers will want to build connections between their robots and LLMs. As I had been working on such a concept to run on the Mars rover prototype at TMR, I saw MCP become popular and realized this is perfect for this use case.

But as a firm believer in quality of life improvements, I wanted to make this super easy to use and thus - Auto MCP.

## Setup 🔧
The dependencies in the `package.xml` contain rosdep dependencies. However, the MCP SDK for Python is not yet in the rosdep repo. I have a [pull request](https://github.com/ros/rosdistro/pull/45588) open for that.

Until that gets resolved, you can use `pip install mcp` to install the dependency as noted in the [python-sdk repo for the Model Context Protocol](https://github.com/modelcontextprotocol/python-sdk).

## Installation 📥
In order to install this package create a colcon workspace following the instructions on the [official ros website](https://docs.ros.org/en/foxy/index.html).

After that, clone this repo to the `src` directory in your workspace.
```bash
cd src
git clone https://github.com/snick-m/auto_mcp.git
```
Then, go back to the root workspace directory and run the build command.
```bash
cd ../

# If you want to only build this package
colcon build --packages-select auto_mcp

# If you want to build all packages in the workspace
colcon build
```
After this you should be able to proceed to using it.

## Usage 🏃🏼‍➡️
Once built and installed using your ros2 workspace, you can launch the node by simply running
```
ros2 run auto_mcp server
```
This will launch the node that will first do introspection and start the MCP server on `0.0.0.0:5000`, therefore, on the local system you can access the SSE endpoint at `http://localhost:5000/sse`

### Adding to Claude Desktop 🔗
Currently I've only tested with Claude Desktop, I'll try to add more necessary instructions over time.

At the time of writing this, Claude Desktop doesn't support adding MCPs with SSE over HTTP transport. However, a really simple [workaround from Cloudflare](https://developers.cloudflare.com/agents/guides/remote-mcp-server/#connect-your-remote-mcp-server-to-claude-and-other-mcp-clients-via-a-local-proxy) makes it usable.

For the Claude Desktop MCP configuration just use the following while modifying the url based on your setup.
```json
{
  "mcpServers": {
    "auto_mcp": {
      "command": "npx",
      "args": [
        "mcp-remote",
        "http://localhost:5000/sse"
      ]
    }
  }
}
```

## Contribution 🫂
Contribution guidelines are soon to come, for now check the issues to see the planned upgrades to this package. If any of them interest you, feel free to ping me on the issue to work on it.