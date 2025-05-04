<h1>Term Project</h1>

## Note: please put the API key in both agent.py and tools.py
## For grading the purpose, my API key is provided as a token to the Professor and TA. 

<h2>An AI agent for ROS2</h2>
## Please click and watch the video

[![Watch the video](https://img.youtube.com/vi/GmDL7iSYY0I/0.jpg)](https://www.youtube.com/watch?v=GmDL7iSYY0I)

https://youtu.be/GmDL7iSYY0I
<h3>Step-1: Build and Launch the Docker Environment</h3>

```bash
# First build turtlebot3 from the link below:
https://github.com/pantelis/turtlebot-maze

# In the cloned turtlebot3_behavior_demos directory:
cd turtlebot3_behavior_demos
docker compose build

# Launch the demo world:
docker compose up demo-world

```


<h3>Step-2: Build a dev container</h3>

```bash
# In another terminal
# In the cloned turtlebot3_behavior_demos directory:
cd turtlebot3_behavior_demos
# Launch the development container in another terminal:
docker compose up dev
```



<h3>Step-3: Launch Local NATS Server on Linux</h3>

```bash 
# In another terminal 
# In the cloned turtlebot3_behavior_demos directory:
cd turtlebot3_behavior_demos
# Access the development container from another terminal:
docker compose exec -it dev bash

# If NATS is not downloaded yet, 
# Download the server files 
wget https://github.com/nats-io/nats-server/releases/download/v2.10.11/nats-server-v2.10.11-linux-amd64.zip

# Step 2: Unzip it
unzip nats-server-v2.10.11-linux-amd64.zip

# Step 3: Enter the directory
cd nats-server-v2.10.11-linux-amd64
# Step 4: Run the server
./nats-server
```
<h3>Step-4: Run ros2_Robot for AI agent</h3>

```bash 
# In another terminal 
# In the cloned turtlebot3_behavior_demos directory:
cd turtlebot3_behavior_demos
# Access the development container from another terminal:
docker compose exec -it dev bash

# Navigate to the source folder:
cd src

# Clone this repo 
git clone https://github.com/MI-Akash/ros2_agentic_ai.git

#Getting back to the previous folder
cd ..

# Make the script executable:
chmod +x src/ros2_ai_agent/ros2_robot_pkg/ros2_robot.py


# Build the specific package:
colcon build --packages-select ros2_robot_pkg

# Source the install setup (only if not already sourced):
source install/setup.bash

# Run the node:
ros2 run ros2_robot_pkg ros2_robot
```


<h3>Step-5: Run the AI agent</h3>

It is not necessary to run agentic_ai from the same device, e.g., PC, but there is no need to run from the dev container. 
Therefore, form this repo downolad the agentic_ai folder or the two python files from there (agent.py and tools.py)

## Note: please put the API key in both agent.py and tools.py
## For grading the purpose, my API key is provided as a token to the Professor and TA. 
Open the agent.py file and place the OpenAI API key. Scroll down and find
os.environ["OPENAI_API_KEY"] = "place the API key"  # 🔐 Replace with your actual key

Open the tools.py and place the OpebAI API key. Scroll down and find 
os.environ["OPENAI_API_KEY"] = "place the API key"  # Replace with your actual key

Then run agent.py 



# Or


``` bash
# from another terminal 
# Clone this repo 
git clone https://github.com/MI-Akash/ros2_agentic_ai.git

# Go to

cd agentic_ai

# open the agent.py file and place the OpenAI API key. Scroll down and find
os.environ["OPENAI_API_KEY"] = "place the API key"  # 🔐 Replace with your actual key

# open the tools.py and place the OpebAI API key. Scroll down and find 
os.environ["OPENAI_API_KEY"] = "place the API key"  # Replace with your actual key

# run

python agent.py

```
