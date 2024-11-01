# Glowbuzzer Python API (GBP) ROS2 Example

This directory contains an example of how to use the glowbuzzer Python API within a ROS2 node.

GBP uses asyncio to handle asynchronous communications with the Glowbuzzer Control (GBC). In ROS,
we use the GBC status message to trigger work to be done within the ROS node. Provided GBC is connected,
this ticks over at a reasonable rate, typically every 100ms.

Within the ROS callbacks, you are able to call coroutines in GBP and await completion. This allows you to
interact with GBC asyncronously.

It's important to use the MultiThreadedExecutor in ROS2 so that the asyncio thread loop runs in parallel with the ROS
node worker threads.

## Project setup in Jetbrains tools (PyCharm, Intellij IDEA)

1. Ensure you have the Python plugin installed
2. Build the Dockerfile and give it a tag, eg. `ros`
3. Open Project Structure and go to SDKs
4. Create a new SDK, choosing Add Python SDK from Disk
5. Select Docker and select the built image. For Python interpreter path, enter /py_wrapper.sh

   The Python interpreter path is important. This calls a shell script which first sources the ROS environment, so that
   the correct Python path is read by the IDE and used at runtime when programs are executed in the container.
6. Once the SDK is created, click Project and change the Project SDK accordingly
7. Click Module, click on src a the top level and click Sources to add to the python path at runtime. This ensures that
   gbp is recognised as a module and imported correctly
8. Create a Python run configuration. Select the SDK of the module and examples/ros/src/main.py as the script. Click the
   folder icon next to Docker container settings and add `--network host`, to ensure the client can connect to GBC
   running on the host or another container. The settings should be similar to
   `--entrypoint= -v d:/gb/gbp:/opt/project --network host --rm`

