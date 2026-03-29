"""
ROS2 Toggle Publisher Node (독립 실행용)

역할:
  - WebUI 토글 버튼의 상태(True/False)를 ROS2 토픽으로 퍼블리시하는 노드
  - /toggle_state 토픽에 std_msgs/Bool 메시지를 10Hz로 발행
  - backend.py 에서 통합 실행할 수도 있고, 이 파일만 단독 실행도 가능

실행 방법:
  ros2 run robot_ui_pkg robot_ui_publisher
"""

# ── ROS2 핵심 라이브러리 ─────────────────────────────────────────────────────
import rclpy                    # ROS2 Python 클라이언트 라이브러리 (초기화, 스핀 등)
from rclpy.node import Node     # 모든 ROS2 노드의 기본 클래스
from std_msgs.msg import Bool   # True/False 값을 담는 표준 메시지 타입


class TogglePublisher(Node):
    """
    토글 퍼블리셔 노드
    - B1~B12 사이트 각각에 대해 독립적인 퍼블리셔와 상태를 관리
    - 0.1초(10Hz) 주기로 모든 토픽에 현재 상태를 발행
    - set_state(site, state)로 개별 사이트 상태 변경 가능
    """

    # 관리할 사이트 목록
    SITE_NAMES = [f'B{i}' for i in range(1, 13)]  # B1 ~ B12

    def __init__(self):
        # 'robot_ui_publisher'라는 이름으로 ROS2 노드 생성
        super().__init__('robot_ui_publisher')

        # ── 사이트별 퍼블리셔 생성 (각각 독립된 토픽) ──
        self.publishers_ = {}   # {사이트명: 퍼블리셔} 딕셔너리
        self.states = {}        # {사이트명: bool} 각 사이트의 토글 상태

        for site in self.SITE_NAMES:
            topic = f'/ui/{site.lower()}_site_toggle_state'
            self.publishers_[site] = self.create_publisher(Bool, topic, 10)
            self.states[site] = False  # 초기값: False (OFF)
            self.get_logger().info(f'  퍼블리셔 생성: {topic}')

        # 0.1초(10Hz) 간격으로 timer_callback 실행 → 모든 토픽에 최신 상태 발행
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            f'robot_ui_publisher started. {len(self.SITE_NAMES)}개 토픽 발행 중'
        )

    def timer_callback(self):
        """타이머에 의해 주기적으로 호출 — 각 사이트의 현재 상태를 해당 토픽에 발행"""
        for site in self.SITE_NAMES:
            msg = Bool()
            msg.data = self.states[site]
            self.publishers_[site].publish(msg)

    def set_state(self, site: str, state: bool):
        """특정 사이트의 토글 상태를 변경 (예: set_state('B1', True))"""
        if site in self.states:
            self.states[site] = state
            self.get_logger().info(f'{site} toggle → {state}')
        else:
            self.get_logger().warn(f'알 수 없는 사이트: {site}')

    def set_all_states(self, state: bool):
        """모든 사이트의 토글 상태를 일괄 변경"""
        for site in self.SITE_NAMES:
            self.states[site] = state
        self.get_logger().info(f'전체 toggle → {state}')


# ── 전역 노드 참조 (backend.py 등 외부 모듈에서 접근용) ──────────────────────
_node: TogglePublisher | None = None


def get_node() -> TogglePublisher | None:
    """현재 실행 중인 robot_ui_publisher 노드 인스턴스를 반환"""
    return _node


# ── 메인 엔트리포인트 ────────────────────────────────────────────────────────
def main(args=None):
    """노드를 초기화하고 스핀(이벤트 루프)을 시작하는 진입점"""
    global _node

    rclpy.init(args=args)           # ROS2 통신 시스템 초기화
    _node = TogglePublisher()       # 퍼블리셔 노드 인스턴스 생성

    try:
        rclpy.spin(_node)           # 콜백(타이머 등)을 처리하는 이벤트 루프 실행
    except KeyboardInterrupt:
        pass                        # Ctrl+C로 종료 시 정상 처리
    finally:
        _node.destroy_node()        # 노드 자원 해제
        rclpy.shutdown()            # ROS2 시스템 종료


if __name__ == '__main__':
    main()
