These are tasks to be completed to reach the initial version 1.0.0 of the package.

1. Implement differentiation logic between subscribing topic and publishing topic.
2. Setup introspection and tools for Publishing topics.
3. Deal with metadata of the package in setup.py and package.xml.
4. Ensure dependencies are properly defined in package.xml
5. Tests (?)
6. Contribution guidelines.

---

1. Wait till Tool field values can start with non alphanumeric char in claude desktop.
2. Currently float values that are 0.0 get truncated to be only 0 from both the MCP Inspector and Claude desktop.
    This causes error with creating the messages to be published. A rosbridge solution could work out in this case but is not desirable.