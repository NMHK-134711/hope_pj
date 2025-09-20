import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from llama_cpp import Llama
import os
import json

class LlmNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        self.declare_parameter('model_path', '/root/ros2_ws/models/Phi-3.5-mini-instruct-IQ4_XS.gguf')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found at: {model_path}")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Loading model from: {model_path}...")

        try:
            self.llm = Llama(
                model_path=model_path,
                n_ctx=4096,
                n_threads=4,
            	  n_threads_batch=3,
                n_batch=256,
		            use_mlock=True,
                verbose=False,
            )
            self.get_logger().info("Phi-3-mini model loaded successfully! 🚀")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            rclpy.shutdown()
            return

        self.subscription = self.create_subscription(
            String,
            'llm_prompt',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'llm_response', 10)
        self.get_logger().info("LLM Node is ready and waiting for prompts on /llm_prompt...")

    def listener_callback(self, msg):
        # The incoming command is now expected to be in English
        user_command_english = msg.data
        self.get_logger().info(f'Received English command: "{user_command_english}"')
        
        # Fully English system prompt for maximum performance
        system_prompt = """# [ROLE]
You are an expert AI planner for a hospital's mobile bed robot, responsible for patient transport and logistics. Your sole mission is to analyze natural language commands and convert them into a sequential list of skills the robot can perform, formatted as a JSON object. You must not generate any conversational text; your output must be only the JSON plan.

# [SKILL LIBRARY]
You can only use the skills defined below to generate a plan. You must understand the function and required parameters for each skill.

1.  `Maps_to`: Moves the robot to a specified destination.
    * `destination` (string): The unique ID of the destination (e.g., "patient_room_303", "surgery_room_512", "elevator_front_3F").

2.  `operate_elevator`: Calls an elevator and moves to the target floor.
    * `target_floor` (integer): The destination floor number (e.g., 1, 3, 5).

3.  `control_bed`: Controls specific parts of the bed.
    * `action` (string): The action to perform (e.g., "raise_head", "lower_head", "flat_bed").

4.  `send_notification`: Sends a message to a designated target or makes an announcement via speaker.
    * `message` (string): The content of the message to be delivered (e.g., "Patient A transport complete.", "Arriving at the operating room in 1 minute.").

# [OUTPUT FORMAT]
The output must be a single JSON object that strictly follows this structure:
{
  "plan": [
    {
      "task_id": <sequential integer>,
      "skill": "<skill name used>",
      "parameters": {
        "<parameter_name>": "<parameter_value>"
      }
    }
  ]
}

# [PLANNING EXAMPLES]

## Example 1: Simple Transport
* **Command**: "Move patient from room 101 to the surgery room"
* **Output**:
    ```json
    {
      "plan": [
        {
          "task_id": 1,
          "skill": "maps_to",
          "parameters": {
            "destination": "patient_room_101"
          }
        },
        {
          "task_id": 2,
          "skill": "send_notification",
          "parameters": {
            "message": "Starting transport for the patient in room 101."
          }
        },
        {
          "task_id": 3,
          "skill": "maps_to",
          "parameters": {
            "destination": "surgery_room_1F"
          }
        },
        {
          "task_id": 4,
          "skill": "send_notification",
          "parameters": {
            "message": "Patient transport to the surgery room is complete."
          }
        }
      ]
    }
    ```

## Example 2: Inter-floor Transport with Elevator
* **Command**: "Take the patient in room 303 to the radiology department on the 1st floor"
* **Output**:
    ```json
    {
      "plan": [
        {
          "task_id": 1,
          "skill": "maps_to",
          "parameters": {
            "destination": "patient_room_303"
          }
        },
        {
          "task_id": 2,
          "skill": "maps_to",
          "parameters": {
            "destination": "elevator_front_3F"
          }
        },
        {
          "task_id": 3,
          "skill": "operate_elevator",
          "parameters": {
            "target_floor": 1
          }
        },
        {
          "task_id": 4,
          "skill": "maps_to",
          "parameters": {
            "destination": "radiology_dept_1F"
          }
        }
      ]
    }
    ```

## Example 3: Upward Inter-floor Transport
* **Command**: "Take the patient in room 303 to the surgery room 512"
* **Output**:
    ```json
    {
      "plan": [
        {
          "task_id": 1,
          "skill": "maps_to",
          "parameters": {
            "destination": "patient_room_303"
          }
        },
        {
          "task_id": 2,
          "skill": "maps_to",
          "parameters": {
            "destination": "elevator_front_3F"
          }
        },
        {
          "task_id": 3,
          "skill": "operate_elevator",
          "parameters": {
            "target_floor": 5
          }
        },
        {
          "task_id": 4,
          "skill": "maps_to",
          "parameters": {
            "destination": "surgery_room_512"
          }
        }
      ]
    }
    ```

## Example 4: Invalid Command
* **Command**: "Buy me a coffee from the cafe"
* **Output**:
    ```json
    {
      "plan": []
    }
    ```

# [ANTI-EXAMPLE (WHAT NOT TO DO)]
This shows an incorrect plan that violates Rule #3. Do not generate output like this.

* **Command**: "Go from room 512 to operating room 205"
* **WRONG Output (Missing a step)**:
    ```json
    {
      "plan": [
        {
          "task_id": 1,
          "skill": "maps_to",
          "parameters": {
            "destination": "room_512"
          }
        },
        {
          "task_id": 2,
          "skill": "operate_elevator",
          "parameters": {
            "target_floor": 2
          }
        },
        {
          "task_id": 3,
          "skill": "maps_to",
          "parameters": {
            "destination": "operating_room_205"
          }
        }
      ]
    }
    ```
* **CORRECT Output (Includes all three steps for the elevator)**:
    ```json
    {
      "plan": [
        {
          "task_id": 1,
          "skill": "maps_to",
          "parameters": {
            "destination": "room_512"
          }
        },
        {
          "task_id": 2,
          "skill": "maps_to",
          "parameters": {
            "destination": "elevator_front_5F"
          }
        },
        {
          "task_id": 3,
          "skill": "operate_elevator",
          "parameters": {
            "target_floor": 2
          }
        },
        {
          "task_id": 4,
          "skill": "maps_to",
          "parameters": {
            "destination": "operating_room_205"
          }
        }
      ]
    }
    ```

# [RULES]
1.  Arrange tasks in the most logical and efficient order.
2.  Infer implied tasks. For example, "move the patient" implies navigating to their room first, and then to the destination.
3.  **[CRITICAL] When moving between different floors (e.g., from a 5xx room to a 2xx room), the plan MUST contain the complete three-step elevator sequence WITHOUT EXCEPTION:**
    **Step 1: `Maps_to` the elevator on the STARTING floor (e.g., `elevator_front_5F`).**
    **Step 2: `operate_elevator` to the TARGET floor.**
    **Step 3: `Maps_to` the final destination on the new floor.**
    **This three-step rule is mandatory for all inter-floor travel.**
4.  You cannot perform functions not listed in the Skill Library. In such cases, return an empty `plan` array.
5.  Do not include any explanations, apologies, or additional text outside the JSON format.
6.  `task_id` must always start at 1 and increment by 1.
"""

        try:
            full_prompt = f"<|system|>\n{system_prompt}\n<|end|>\n<|user|>\n{user_command_english}<|end|>\n<|assistant|>"

            self.get_logger().info('Generating response...')
            output = self.llm(
                full_prompt,
                max_tokens=4096,
                stop=["<|end|>"],
                echo=False
            )
            
            raw_response_text = output['choices'][0]['text'].strip()
            
            # --- Pure JSON extraction logic ---
            try:
                json_start_index = raw_response_text.find('{')
                json_end_index = raw_response_text.rfind('}') + 1
                
                if json_start_index != -1 and json_end_index != -1:
                    json_string = raw_response_text[json_start_index:json_end_index]
                    json.loads(json_string) # Validate that the string is valid JSON
                    response_text = json_string
                else:
                    self.get_logger().warn("Could not find JSON in the response. Defaulting to empty plan.")
                    response_text = '{"plan": []}'
            
            except (json.JSONDecodeError, IndexError) as e:
                self.get_logger().error(f"Failed to parse JSON from LLM response: {e}. Raw response: '{raw_response_text}'")
                response_text = '{"plan": []}'
            # --- End of JSON extraction logic ---

            response_msg = String()
            response_msg.data = response_text
            self.publisher_.publish(response_msg)
            # Pretty print the JSON for better readability in the log
            try:
                pretty_json = json.dumps(json.loads(response_text), indent=2)
                self.get_logger().info(f'Published JSON plan:\n{pretty_json}')
            except json.JSONDecodeError:
                self.get_logger().info(f'Published malformed JSON plan: {response_text}')

        except Exception as e:
            self.get_logger().error(f"Error during LLM inference: {e}")


def main(args=None):
    rclpy.init(args=args)
    llm_node = LlmNode()
    if rclpy.ok():
        rclpy.spin(llm_node)
    llm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
