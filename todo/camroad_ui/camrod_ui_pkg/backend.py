"""
FastAPI 백엔드 — React WebUI와 ROS2 퍼블리셔 노드를 연결하는 브릿지 서버

역할:
  1. WebSocket 서버(/ws)를 통해 프론트엔드와 실시간 양방향 통신
  2. 프론트엔드에서 받은 토글 상태를 ROS2 노드에 전달
  3. ROS2 노드가 /toggle_state 토픽에 Bool 메시지를 발행
  4. React 빌드 파일(정적 파일)을 HTTP로 서빙

실행 방법:
    python3 -m robot_ui_pkg.backend
    또는 ros2 run robot_ui_pkg backend
"""

# ── 표준 라이브러리 ──────────────────────────────────────────────────────────
import json          # WebSocket 메시지(JSON 문자열) 파싱용
import asyncio       # 동기 콜백에서 비동기 브로드캐스트 호출용
import threading     # ROS2 스핀을 별도 스레드에서 실행하기 위함
import pathlib       # 파일 경로를 OS 독립적으로 다루기 위함import httpx         # 날씨 API 호출용 (비동기 HTTP 클라이언트)
# ── ROS2 라이브러리 ──────────────────────────────────────────────────────────
import rclpy                    # ROS2 Python 클라이언트 (초기화, 스핀)
from rclpy.node import Node     # ROS2 노드 기본 클래스
from std_msgs.msg import Bool                           # True/False를 담는 표준 메시지 타입
from diagnostic_msgs.msg import DiagnosticArray         # 진단 집계 메시지 타입

# ── 웹 프레임워크 ────────────────────────────────────────────────────────────
import uvicorn                                      # ASGI 서버 (FastAPI 실행용)
from fastapi import FastAPI, WebSocket, WebSocketDisconnect  # 웹 프레임워크 & WebSocket
from fastapi.middleware.cors import CORSMiddleware   # CORS 허용 미들웨어
from fastapi.responses import FileResponse           # 단일 파일 응답 (index.html)
from fastapi.staticfiles import StaticFiles          # 정적 파일 서빙 (JS, CSS 등)


# ══════════════════════════════════════════════════════════════════════════════
# ROS2 노드 — 백엔드 프로세스 내부에서 동작하는 퍼블리셔
# ══════════════════════════════════════════════════════════════════════════════

class TogglePublisherNode(Node):
    """
    백엔드에 내장된 ROS2 퍼블리셔 노드
    - B1~B12 사이트별로 독립된 퍼블리셔와 상태를 관리
    - 각 /ui/{site}_site_toggle_state 토픽에 Bool 메시지를 10Hz로 발행
    - set_state(site, state)로 개별 사이트 제어
    """

    # 관리할 사이트 목록 (toggle_publisher.py와 동일)
    SITE_NAMES = [f'B{i}' for i in range(1, 13)]  # B1 ~ B12

    def __init__(self):
        # 'toggle_publisher'라는 이름으로 ROS2 노드 초기화
        super().__init__('toggle_publisher')

        # ── 사이트별 퍼블리셔 생성 (각각 독립된 토픽) ──
        self.publishers_ = {}   # {사이트명: 퍼블리셔}
        self.states = {}        # {사이트명: bool} 각 사이트의 토글 상태

        for site in self.SITE_NAMES:
            topic = f'/ui/{site.lower()}_site_toggle_state'
            self.publishers_[site] = self.create_publisher(Bool, topic, 10)
            self.states[site] = False  # 초기값: False (OFF)

        # 0.1초(10Hz) 주기로 _publish() 호출하는 타이머 생성
        self.timer = self.create_timer(0.1, self._publish)

        # ── /AMR_arrive 토픽 구독 (도착 알림) ──
        self.arrive_sub = self.create_subscription(
            Bool, '/AMR_arrive', self._on_amr_arrive, 10
        )
        self.arrive_callback = None  # 외부에서 등록할 콜백

        # ── /diagnostics_agg 토픽 구독 (진단 집계) ──
        self.latest_diagnostics = []  # 최신 진단 데이터 저장
        self.diag_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics_agg', self._on_diagnostics, 10
        )

        self.get_logger().info(
            f'TogglePublisher ready — {len(self.SITE_NAMES)}개 토픽 발행 중'
        )

    def _on_diagnostics(self, msg: DiagnosticArray):
        """/diagnostics_agg 수신 시 최신 진단 데이터 갱신"""
        LEVEL_STR = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}
        self.latest_diagnostics = [
            {
                "name":    s.name,
                "level":   s.level[0] if isinstance(s.level, (bytes, bytearray)) else int(s.level),  # 0=OK, 1=WARN, 2=ERROR, 3=STALE
                "message": s.message,
                "values":  [{"key": kv.key, "value": kv.value} for kv in s.values],
            }
            for s in msg.status
        ]
        counts = {lv: 0 for lv in LEVEL_STR.values()}
        for d in self.latest_diagnostics:
            counts[LEVEL_STR.get(d['level'], 'STALE')] += 1
        self.get_logger().info(
            f'[diagnostics_agg] {len(self.latest_diagnostics)}개 수신 — '
            f"OK:{counts['OK']} WARN:{counts['WARN']} ERROR:{counts['ERROR']} STALE:{counts['STALE']}"
        )

    def _on_amr_arrive(self, msg):
        """/AMR_arrive 토픽에서 True 수신 시 토글 해제 + 프론트엔드 알림"""
        if msg.data:
            # 현재 ON인 사이트 찾기
            arrived_site = None
            for site in self.SITE_NAMES:
                if self.states[site]:
                    arrived_site = site
                    break
            if arrived_site:
                self.get_logger().info(f'AMR 도착: {arrived_site}')
                if self.arrive_callback:
                    self.arrive_callback(arrived_site)

    def _publish(self):
        """타이머 콜백 — 각 사이트의 현재 상태를 해당 토픽에 발행"""
        for site in self.SITE_NAMES:
            msg = Bool()
            msg.data = self.states[site]
            self.publishers_[site].publish(msg)

    def set_state(self, site: str, state: bool):
        """특정 사이트의 토글 상태를 변경 (예: set_state('B1', True))"""
        if site in self.states:
            self.states[site] = state
            self.get_logger().info(f'{site} toggle → {state}')

    def get_all_states(self) -> dict:
        """모든 사이트의 현재 상태를 딕셔너리로 반환"""
        return dict(self.states)


# ══════════════════════════════════════════════════════════════════════════════
# FastAPI 앱 설정
# ══════════════════════════════════════════════════════════════════════════════

# FastAPI 앱 인스턴스 생성
app = FastAPI(title='Robot UI Backend')

# CORS 미들웨어 추가 — 프론트엔드(다른 포트)에서의 요청을 허용
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],      # 모든 출처 허용 (개발 환경)
    allow_methods=["*"],      # 모든 HTTP 메서드 허용
    allow_headers=["*"],      # 모든 헤더 허용
)

# main()에서 초기화되는 ROS2 노드 참조
ros_node: TogglePublisherNode | None = None

# 현재 연결된 WebSocket 클라이언트 집합 (상태 브로드캐스트용)
ws_clients: set[WebSocket] = set()

# uvicorn 이벤트 루프 참조 (동기 콜백에서 비동기 브로드캐스트 호출용)
main_loop: asyncio.AbstractEventLoop | None = None


# ══════════════════════════════════════════════════════════════════════════════
# WebSocket 엔드포인트 — 프론트엔드와 실시간 통신
# ══════════════════════════════════════════════════════════════════════════════

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    """
    WebSocket 연결 처리:
    1) 클라이언트 접속 시 모든 사이트의 현재 상태를 전송
    2) 클라이언트에서 {"site": "B1", "state": true} 수신 → 해당 사이트 ROS2 퍼블리셔에 전달
    3) 변경된 상태를 모든 연결 클라이언트에 브로드캐스트
    """
    await ws.accept()           # WebSocket 핸드셰이크 수락
    ws_clients.add(ws)          # 클라이언트 목록에 추가

    # 접속 직후 모든 사이트의 현재 상태를 전송 (UI 초기 동기화)
    # 형식: {"states": {"B1": false, "B2": false, ..., "B12": false}}
    all_states = ros_node.get_all_states() if ros_node else {}
    await ws.send_json({"states": all_states})

    try:
        while True:
            # 클라이언트로부터 JSON 텍스트 메시지 수신 대기
            data = await ws.receive_text()
            payload = json.loads(data)  # JSON 문자열 → Python dict

            # {"site": "B1", "state": true} 형식으로 개별 사이트 토글 처리
            if "site" in payload and "state" in payload and ros_node is not None:
                site = payload["site"]
                new_state = bool(payload["state"])
                ros_node.set_state(site, new_state)  # ROS2 노드에 새 상태 전달

                # 연결된 모든 클라이언트에 변경된 사이트 상태를 브로드캐스트
                for client in list(ws_clients):
                    try:
                        await client.send_json({"site": site, "state": new_state})
                    except Exception:
                        ws_clients.discard(client)  # 전송 실패한 클라이언트 제거
    except WebSocketDisconnect:
        ws_clients.discard(ws)  # 연결 종료 시 목록에서 제거


async def broadcast_arrive(site: str):
    """프론트엔드에 도착 알림 브로드캐스트 + 토글 OFF"""
    if ros_node:
        ros_node.set_state(site, False)
    # 도착 알림 전송
    for client in list(ws_clients):
        try:
            await client.send_json({"arrived": site})
            await client.send_json({"site": site, "state": False})
        except Exception:
            ws_clients.discard(client)


def on_amr_arrive_callback(site: str):
    """ROS2 콜백 스레드에서 호출 — 이벤트 루프로 브로드캐스트 예약"""
    if main_loop:
        asyncio.run_coroutine_threadsafe(broadcast_arrive(site), main_loop)


# ══════════════════════════════════════════════════════════════════════════════
# 진단 API — /diagnostics_agg 최신 데이터 반환
# ══════════════════════════════════════════════════════════════════════════════

@app.get("/api/diagnostics")
async def get_diagnostics():
    """최신 /diagnostics_agg 데이터를 JSON으로 반환"""
    return {"status": ros_node.latest_diagnostics if ros_node else []}


# ══════════════════════════════════════════════════════════════════════════════
# 날씨 API 프록시 — wttr.in에서 월악산 날씨 정보 조회
# ══════════════════════════════════════════════════════════════════════════════

# wttr.in 날씨 코드 → 한국어 매핑
WEATHER_DESC_KO = {
    'Clear': '맑음', 'Sunny': '맑음',
    'Partly cloudy': '구름 조금', 'Cloudy': '흐림', 'Overcast': '흐림',
    'Mist': '안개', 'Fog': '안개', 'Haze': '연무',
    'Light rain': '약한 비', 'Moderate rain': '비', 'Heavy rain': '폭우',
    'Light snow': '약한 눈', 'Moderate snow': '눈', 'Heavy snow': '폭설',
    'Thunderstorm': '뇌우', 'Patchy rain possible': '비 가능성',
    'Light drizzle': '이슬비', 'Patchy snow possible': '눈 가능성',
}


@app.get("/api/weather")
async def get_weather():
    """월악산 현재 날씨 정보를 wttr.in에서 조회하여 반환"""
    try:
        async with httpx.AsyncClient(timeout=5.0) as client:
            resp = await client.get("https://wttr.in/Woraksan?format=j1")
            resp.raise_for_status()
            data = resp.json()
            current = data["current_condition"][0]
            temp_c = current["temp_C"]
            desc_en = current["weatherDesc"][0]["value"]
            desc_ko = WEATHER_DESC_KO.get(desc_en, desc_en)
            return {"temp": temp_c, "desc": desc_ko, "desc_en": desc_en}
    except Exception:
        return {"temp": "--", "desc": "정보 없음", "desc_en": "N/A"}


# ══════════════════════════════════════════════════════════════════════════════# 정적 파일 서빙 — React 빌드 결과물(index.html, JS, CSS)을 HTTP로 제공
# ══════════════════════════════════════════════════════════════════════════════

# React 빌드 출력 디렉토리: colcon 설치 시 share 디렉토리에 있음
try:
    from ament_index_python.packages import get_package_share_directory
    FRONTEND_DIR = pathlib.Path(get_package_share_directory('robot_ui_pkg')) / 'frontend' / 'build'
except Exception:
    # 개발 환경 fallback: 소스 디렉토리 기준
    FRONTEND_DIR = pathlib.Path(__file__).resolve().parent.parent / "frontend" / "build"


@app.get("/")
async def serve_index():
    """루트(/) 요청 시 React의 index.html을 반환"""
    index = FRONTEND_DIR / "index.html"
    if index.exists():
        return FileResponse(str(index))
    return {"message": "Frontend not built yet. Run: cd frontend && npm run build"}


# 빌드 폴더가 존재하면 정적 파일(JS, CSS, 이미지 등)을 마운트
if FRONTEND_DIR.exists():
    app.mount("/", StaticFiles(directory=str(FRONTEND_DIR), html=True), name="frontend")


# ══════════════════════════════════════════════════════════════════════════════
# 메인 엔트리포인트 — ROS2 + FastAPI를 하나의 프로세스에서 동시 실행
# ══════════════════════════════════════════════════════════════════════════════

def main():
    """
    실행 흐름:
    1) rclpy 초기화 → ROS2 퍼블리셔 노드 생성
    2) ROS2 spin을 별도 데몬 스레드에서 실행 (백그라운드)
    3) uvicorn(FastAPI)을 메인 스레드에서 실행 (포트 8000)
    """
    global ros_node, main_loop

    # ROS2 통신 시스템 초기화
    rclpy.init()
    # 퍼블리셔 노드 인스턴스 생성
    ros_node = TogglePublisherNode()
    ros_node.arrive_callback = on_amr_arrive_callback

    # ROS2 이벤트 루프(spin)를 백그라운드 데몬 스레드에서 실행
    # → 타이머 콜백(_publish)이 주기적으로 동작하게 됨
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    # FastAPI(uvicorn) 서버 시작 — 메인 스레드에서 블로킹 실행
    # 브라우저에서 http://localhost:8000 으로 접속 가능
    try:
        main_loop = asyncio.new_event_loop()
        config = uvicorn.Config(app, host="0.0.0.0", port=8000, log_level="info", loop="asyncio")
        server = uvicorn.Server(config)
        main_loop.run_until_complete(server.serve())
    except KeyboardInterrupt:
        pass  # Ctrl+C로 종료 시 정상 처리
    finally:
        ros_node.destroy_node()  # ROS2 노드 자원 해제
        rclpy.shutdown()         # ROS2 시스템 종료


if __name__ == '__main__':
    main()
