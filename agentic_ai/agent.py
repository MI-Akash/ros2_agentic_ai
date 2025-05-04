# agent.py

import os
from pydantic_ai import Agent
from pydantic_ai.models.openai import OpenAIModel
from pydantic_ai.providers.openai import OpenAIProvider
from pydantic import BaseModel
from typing import Union, Optional

from tools import (
    get_robot_posture_tool,
    move_robot_tool,
    get_system_report_tool,
    capture_camera_image_tool
)

# -------------------------------
# Output Schemas
# -------------------------------

class RobotStatusResult(BaseModel):
    status: str
    message: str

class RobotCommandResult(BaseModel):
    status: str
    message: str

class SystemReportResult(BaseModel):
    status: str
    report: str
class ImageResult(BaseModel):
    status: str
    report: str

# Camera tool returns a base64 string
AgentOutput = Union[
    RobotStatusResult,
    RobotCommandResult,
    SystemReportResult,
    ImageResult  # base64-encoded image string
]

# -------------------------------
# API Key & Model
# -------------------------------

os.environ["OPENAI_API_KEY"] = "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"  # 🔐 Replace with your actual key

openai_model = OpenAIModel(
    model_name="gpt-4o",
    provider=OpenAIProvider()
)

# -------------------------------
# Agent Definition
# -------------------------------

agent = Agent(
    model=openai_model,
    output_type=AgentOutput,
    tools=[
        get_robot_posture_tool,
        move_robot_tool,
        get_system_report_tool,
        capture_camera_image_tool
    ],
    system_prompt = (
        "You are a helpful robot assistant.:\n"
        "- If you are asked to change the posture, e.g., fall down, immediately call robot posture tool and respond whether you need to change the posture\n"
        "- If you are asked about system report, immediately read the full system report first, then briefly describe the system report with, especially CPU use, Memory, temperature and overal health).\n\n"
        "The system report is the following:\n"
        "CPU Status: \n"
        "Memory Status: \n"
        "Battery Status: \n"
        "Overwall Health: \n"
        "- If you are asked to move or rotate, immediately, send the motion command\n"
        "- When the user asks anything related to an image, call the `capture_camera_image_tool` tool immediately.\n\n"
        "Describe the relative distance and orentation of the objects you observe in the image with respec to the robot, e.g.,\n"
        "An object x is two meter away and at positive 30 degrees from the robot, ..."
    )
)

# -------------------------------
# Terminal Interaction Loop
# -------------------------------

print("\n🤖 Robot Assistant Ready. Type 'exit' to quit.\n")

while True:
    user_input = input("🧑 You: ").strip()
    if user_input.lower() in {"exit", "quit"}:
        print("👋 Goodbye!")
        break

    result = agent.run_sync(user_input)
    output = result.output
    print("🤖 Robot:", output)

    #print("DEBUG: result.output type:", type(result.output))
    #print("DEBUG: result.output:", result.output[:100])  # Print first 100 chars if it's long
