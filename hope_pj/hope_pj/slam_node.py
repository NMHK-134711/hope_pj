import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import yaml
import math

class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')

        # YAML 파일 경로를 파라미터로 선언 및 로드
        self.declare_parameter('locations_file', 'default_path.yaml')
        locations_file_path = self.get_parameter('locations_file').get_parameter_value().string_value
        self.locations = self.load_locations_from_yaml(locations_file_path)

        # Nav2 액션 클라이언트 초기화
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ControlNode와 통신할 퍼블리셔/서브스크라이버
        self.nav_complete_publisher = self.create_publisher(String, '/navigation_complete', 10)
        self.target_subscription = self.create_subscription(
            String,
            '/target_destination',
            self.target_destination_callback,
            10)

        self.is_navigating = False
        self.current_destination = ""
        self.get_logger().info('✅ Slam Node is ready and waiting for a destination.')

    def load_locations_from_yaml(self, file_path):
        """YAML 파일에서 목적지 이름과 좌표를 불러옵니다."""
        try:
            with open(file_path, 'r') as file:
                locations_data = yaml.safe_load(file)
                self.get_logger().info(f"Successfully loaded {len(locations_data)} locations from {file_path}")
                return locations_data
        except FileNotFoundError:
            self.get_logger().error(f"Locations file not found at: {file_path}. Node will not be able to navigate.")
            return {}
        except Exception as e:
            self.get_logger().error(f"Failed to parse YAML file: {e}")
            return {}

    def target_destination_callback(self, msg):
        """ControlNode로부터 목적지를 수신하는 콜백 함수"""
        if self.is_navigating:
            self.get_logger().warn(f"Already navigating to '{self.current_destination}'. Ignoring new request for '{msg.data}'.")
            return

        destination_name = msg.data
        self.current_destination = destination_name
        
        if destination_name not in self.locations:
            self.get_logger().error(f"Destination '{destination_name}' not found in the locations file.")
            self.report_completion(False)
            return

        # 목적지 좌표로 PoseStamped 메시지 생성
        location_data = self.locations[destination_name]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(location_data['x'])
        goal_pose.pose.position.y = float(location_data['y'])

        # Yaw(라디안) 값을 Quaternion으로 변환
        yaw = float(location_data['yaw'])
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        goal_pose.pose.orientation.z = sy
        goal_pose.pose.orientation.w = cy
        
        self.send_navigation_goal(goal_pose)

    def send_navigation_goal(self, goal_pose):
        """Nav2 액션 서버에 목표 지점을 전송합니다."""
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            self.report_completion(False)
            return

        self.is_navigating = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f"Sending goal to Nav2: {self.current_destination}")
        
        # 비동기적으로 골을 전송하고 결과 처리를 위한 콜백을 연결
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Nav2가 목표를 수락/거부했는지에 대한 응답 콜백"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2 server.')
            self.report_completion(False)
            return

        self.get_logger().info('Goal accepted by Nav2 server. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Nav2의 최종 이동 결과 콜백"""
        from rclpy.action.client import ClientGoalHandle, GoalStatus
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.SUCCEEDED:
            self.get_logger().info(f"Navigation succeeded to '{self.current_destination}'!")
            self.report_completion(True)
        else:
            self.get_logger().error(f"Navigation failed with status: {status}")
            self.report_completion(False)

    def report_completion(self, success):
        """ControlNode에 성공/실패 결과를 보고합니다."""
        msg = String()
        if success:
            msg.data = f"Success: Reached {self.current_destination}"
        else:
            msg.data = f"Failure: Could not reach {self.current_destination}"
        
        self.nav_complete_publisher.publish(msg)
        self.is_navigating = False
        self.current_destination = ""

def main(args=None):
    rclpy.init(args=args)
    slam_node = SlamNode()
    rclpy.spin(slam_node)
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
