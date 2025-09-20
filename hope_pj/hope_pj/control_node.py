import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import time

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # --- 상태 관리를 위한 변수 ---
        self.current_plan = []
        self.current_task_index = 0
        self.is_executing_plan = False
        self.is_waiting_for_nav = False

        # --- 퍼블리셔(Publisher) 정의 ---
        # 1. SlamNode로 목표 지점을 보내는 퍼블리셔
        self.slam_publisher = self.create_publisher(String, '/target_destination', 10)
        # 2. PC(디버그)로 현재 상태를 보내는 퍼블리셔
        self.status_publisher = self.create_publisher(String, '/control_status', 10)

        # --- 서브스크라이버(Subscriber) 정의 ---
        # 1. LlmNode로부터 JSON 계획을 받는 서브스크라이버
        self.llm_subscription = self.create_subscription(
            String,
            '/llm_response',
            self.llm_response_callback,
            10)
        # 2. SlamNode로부터 이동 완료 신호를 받는 서브스크라이버
        self.nav_subscription = self.create_subscription(
            String, # SlamNode가 성공/실패 여부를 String으로 보낼 수 있도록 설정
            '/navigation_complete',
            self.navigation_complete_callback,
            10)

        self.get_logger().info('✅ Control Node has been initialized and is ready.')
        self.publish_status("Control Node is waiting for a plan from LlmNode.")

    def publish_status(self, message):
        """디버그 및 상태 모니터링을 위한 메시지를 발행하는 헬퍼 함수"""
        self.get_logger().info(message)
        msg = String()
        msg.data = message
        self.status_publisher.publish(msg)

    def llm_response_callback(self, msg):
        """LlmNode로부터 받은 JSON 계획을 처리하는 콜백 함수"""
        if self.is_executing_plan:
            self.publish_status("⚠️ Warning: Received a new plan, but a current plan is already executing. Ignoring new plan.")
            return

        self.publish_status(f"Received new plan from LLM: {msg.data}")

        try:
            data = json.loads(msg.data)
            self.current_plan = data.get('plan', [])

            if not self.current_plan:
                self.publish_status("Plan is empty. Nothing to execute.")
                return

            # 계획 실행 시작
            self.is_executing_plan = True
            self.current_task_index = 0
            self.publish_status(f"🚀 Starting execution of a new plan with {len(self.current_plan)} tasks.")
            self.execute_current_task()

        except json.JSONDecodeError:
            self.publish_status("❌ Error: Failed to decode JSON from /llm_response.")
            self.reset_state()

    def navigation_complete_callback(self, msg):
        """SlamNode로부터 이동 완료 신호를 받았을 때 호출되는 콜백 함수"""
        if not self.is_waiting_for_nav:
            self.publish_status("⚠️ Warning: Received a navigation complete signal, but was not waiting for one. Ignoring.")
            return

        self.is_waiting_for_nav = False
        self.publish_status(f"✅ Navigation task completed with status: '{msg.data}'. Proceeding to the next task.")
        
        # 다음 작업으로 이동
        self.current_task_index += 1
        self.execute_current_task()

    def execute_current_task(self):
        """현재 인덱스에 해당하는 작업을 실행하는 메인 함수"""
        if self.current_task_index >= len(self.current_plan):
            self.publish_status("🎉 Plan execution complete!")
            self.reset_state()
            return

        task = self.current_plan[self.current_task_index]
        task_id = task.get("task_id", "N/A")
        skill = task.get("skill", "N/A")
        parameters = task.get("parameters", {})

        self.publish_status(f"Executing Task {task_id}: Skill='{skill}', Params={parameters}")

        if skill == 'maps_to':
            destination = parameters.get('destination')
            if destination:
                # SlamNode에 목적지 발행
                dest_msg = String()
                dest_msg.data = destination
                self.slam_publisher.publish(dest_msg)
                self.publish_status(f"Sent destination '{destination}' to SlamNode. Waiting for completion...")
                self.is_waiting_for_nav = True
                # 이동 완료 신호를 기다리므로 여기서 함수 종료
            else:
                self.publish_status("❌ Error: 'maps_to' skill is missing 'destination' parameter. Skipping task.")
                self.proceed_to_next_task_immediately()

        elif skill in ['operate_elevator', 'control_bed', 'send_notification']:
            # 이 스킬들은 즉시 완료되는 것으로 간주
            # 실제 하드웨어 제어 코드가 필요하다면 여기에 추가
            self.publish_status(f"Simulating immediate execution of skill '{skill}'.")
            time.sleep(1) # 실제 동작을 흉내 내기 위한 약간의 딜레이
            self.proceed_to_next_task_immediately()

        else:
            self.publish_status(f"⚠️ Warning: Unknown skill '{skill}'. Skipping task.")
            self.proceed_to_next_task_immediately()
    
    def proceed_to_next_task_immediately(self):
        """즉시 완료되는 작업을 처리하고 바로 다음 작업을 실행"""
        self.current_task_index += 1
        # 재귀적으로 다음 작업 호출
        self.execute_current_task()

    def reset_state(self):
        """하나의 계획 실행이 끝난 후 상태를 초기화"""
        self.current_plan = []
        self.current_task_index = 0
        self.is_executing_plan = False
        self.is_waiting_for_nav = False
        self.publish_status("State has been reset. Ready for a new plan.")

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
