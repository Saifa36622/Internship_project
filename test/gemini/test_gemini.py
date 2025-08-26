# from google import genai

# # The client gets the API key from the environment variable `GEMINI_API_KEY`.
# client = genai.Client()
# allowed_commands = ["move_to", "pick_up", "place"]
# content = f"""Right now I am trying to use you as a decision maker for my mobile manipulation robot.
# For the task: "pick the water bottle up", analyze the task and then respond in JSON format.
# The JSON should contain a single key, "steps", which holds a list of dictionaries.
# Each dictionary in the list should have two keys: "command" and "object".
# The "command" must be one of the following: {allowed_commands}.

# Example of desired JSON format:
# {{
#   "steps": [
#     {{"command": "move_to", "object": "water bottle"}},
#     {{"command": "pick_up", "object": "water bottle"}},
#     {{"command": "place", "object": "water bottle"}}
#   ]
# }}
# """
# response = client.models.generate_content(
#     model="gemini-2.5-flash", contents=content
# )
# print(response.text)


import json
from google import genai

# The client gets the API key from the environment variable `GEMINI_API_KEY`.
client = genai.Client()

# Define the allowed commands for clarity in the prompt
allowed_commands = ["move_to", "pick_up", "place"]

robot_input = "pick the water bottle up"

# Construct the prompt to clearly instruct Gemini on the desired JSON format
content = f"""Right now I am trying to use you as a decision maker for my mobile manipulation robot.
For the task: "{robot_input}", analyze the task and then respond in JSON format.
The JSON should contain a single key, "steps", which holds a list of dictionaries.
Each dictionary in the list should have two keys: "command" and "object".
The "command" must be one of the following: {allowed_commands}.

Example of desired JSON format:
{{
  "steps": [
    {{"command": "move_to", "object": "water bottle"}},
    {{"command": "pick_up", "object": "water bottle"}},
    {{"command": "place", "object": "water bottle"}}
  ]
}}
"""

try:
    response = client.models.generate_content(
        model="gemini-2.5-flash", # Or "gemini-1.5-flash" or "gemini-1.5-pro" for potentially better reasoning
        contents=content
    )

    # Print the raw text response from Gemini (for debugging)
    print("Raw response from Gemini:")
    print(response.text)
    print("\n" + "="*30 + "\n")

    # Attempt to parse the JSON string
    # It's good practice to strip any leading/trailing whitespace or ```json ``` markdown
    json_string = response.text.strip()
    if json_string.startswith("```json"):
        json_string = json_string[len("```json"):].strip()
    if json_string.endswith("```"):
        json_string = json_string[:-len("```")].strip()

    parsed_data = json.loads(json_string)

    # Validate the structure (optional but recommended)
    if "steps" in parsed_data and isinstance(parsed_data["steps"], list):
        print("Parsed Python Dictionary/List from JSON:")
        for step in parsed_data["steps"]:
            if isinstance(step, dict) and "command" in step and "object" in step:
                print(f"  Command: {step['command']}, Object: {step['object']}")
            else:
                print(f"  Warning: Unexpected step format: {step}")
        
        # You can now easily access the list of steps
        robot_commands = parsed_data["steps"]
        print(f"\nYour robot's command list is: {robot_commands}")

    else:
        print("JSON structure not as expected. 'steps' key not found or is not a list.")
        print(f"Parsed data: {parsed_data}")

except json.JSONDecodeError as e:
    print(f"Error decoding JSON: {e}")
    print(f"Problematic text that caused the error:\n{response.text}")
except Exception as e:
    print(f"An unexpected error occurred: {e}")