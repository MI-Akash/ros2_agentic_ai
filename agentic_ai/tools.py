# tools.py

import os
import asyncio
import json
from typing import Optional, Literal
import base64
from nats.aio.client import Client as NATS
from pydantic_ai.tools import Tool
from openai import OpenAI
from pathlib import Path



os.environ["OPENAI_API_KEY"] = "place the API key"  # Replace with your actual key

# Initialize OpenAI client
client = OpenAI()

# -----------------------------------------------------------
# Tool 1: Get robot posture
# -----------------------------------------------------------

async def get_robot_posture(query: Optional[str] = "Get the current posture of the robot.") -> dict:
    try:
        nc = NATS()
        await nc.connect()
        response = await nc.request("robot.status.request", b"status?", timeout=1)
        await nc.close()
        return {"status": "success", "message": response.data.decode()}
    except Exception as e:
        return {"status": "error", "message": f"Failed to retrieve robot status: {e}"}

get_robot_posture_tool = Tool(
    name="get_robot_posture",
    description="Retrieve the robot's current posture from the robot simulator.",
    function=get_robot_posture
)

# -----------------------------------------------------------
# Tool 2: Move robot
# -----------------------------------------------------------

async def move_robot(
    move_direction: Literal["forward", "backward"],
    move_distance: float,
    rotate_direction: Literal["left", "right"],
    rotate_angle: float
) -> dict:
    command = {
        "Move": {"direction": move_direction, "distance": move_distance},
        "Rotate": {"direction": rotate_direction, "angle": rotate_angle}
    }

    try:
        nc = NATS()
        await nc.connect()
        await nc.publish("robot.command.execute", json.dumps(command).encode())
        await nc.flush()
        await nc.close()
        return {"status": "success", "message": f"Command sent: {command}"}
    except Exception as e:
        return {"status": "error", "message": f"Failed to send move command: {e}"}

move_robot_tool = Tool(
    name="move_robot",
    description="Send a movement command to the robot (forward/backward + rotation).",
    function=move_robot
)

# -----------------------------------------------------------
# Tool 3: Get system report
# -----------------------------------------------------------

async def get_system_report(note: Optional[str] = "MUST read the system report first, then briefly explain the system report in natural language for non tech people") -> dict:
    try:
        nc = NATS()
        await nc.connect()
        response = await nc.request("robot.system.report", b"report?", timeout=5)
        await nc.close()
        return {"status": "success", "report": response.data.decode()}
    except Exception as e:
        return {"status": "error", "report": f"Failed to get system report: {e}"}

get_system_report_tool = Tool(
    name="get_system_report",
    description="Get a full system report (battery, CPU, health, temperature, etc.).",
    function=get_system_report
)

# -----------------------------------------------------------
# Tool 4: Capture camera image (GPT-4o-compatible return)
# -----------------------------------------------------------

async def capture_camera_image(query: Optional[str] = "What do you see in this image?") -> str:
    #print("[DEBUG] Running capture_camera_image tool...")
    try:
        nc = NATS()
        await nc.connect()
        response = await nc.request("robot.camera.capture", b"image?", timeout=5)
        await nc.close()

        image_bytes = response.data
        #print(f"[DEBUG] Received {len(image_bytes)} bytes from NATS camera topic.")

        image_path = "captured_image.jpg"
        with open(image_path, "wb") as f:
            f.write(image_bytes)
        
        #print(f"[DEBUG] Image saved at: {os.path.abspath(image_path)}")

        # Use a fixed image file path
        file_path = image_path
        img_path = Path(file_path)

        if not img_path.exists():
            return f"[ERROR] Image file not found: {file_path}"

        # Encode image to base64
        encoded_image = base64.b64encode(img_path.read_bytes()).decode("utf-8")

        # Send the image and query to GPT-4o
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": "onserve the major objects present in the image. Descrine their relative positions with respect to the robot camera, e.g., 2 m away and at 30 degrees or negative 20 degrees"},
                {"role": "user", "content": [
                    {"type": "text", "text": query},
                    {"type": "image_url", "image_url": {
                        "url": f"data:image/jpeg;base64,{encoded_image}"
                    }}
                ]}
            ]
        )

        return {"status": "success", "report": response.choices[0].message.content}

    except Exception as e:
        return {"status": "error", "message": f"Failed to analyze image: {e}"}

capture_camera_image_tool = Tool(
    name="capture_camera_image",
    description="Capture an image from the robot's camera and return it for GPT-4o visual analysis.",
    function=capture_camera_image
)
