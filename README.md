# 🏥 APTS: On-Board LLM 기반 지능형 병원 침대 로봇

**APTS**: **A**utonomous **P**atient **T**ransport **S**ystem

## 📝 프로젝트 개요 (Overview)

**APTS**는 On-Board LLM을 탑재하여 자연어 명령을 이해하고 자율적으로 환자 이송 임무를 수행하는 지능형 병원 침대 로봇입니다.

고령화 사회 진입으로 인해 거동이 불편한 환자 수는 급증하는 반면, 한정된 의료 인력은 과도한 육체노동과 번아웃에 직면하고 있습니다. 본 프로젝트는 이러한 의료 현장의 '그림자 노동'인 환자 이송 업무를 자동화하여, 의료진이 환자 케어라는 본연의 업무에 집중할 수 있는 환경을 만들고, 병원 운영의 효율성과 안전성을 극대화하는 것을 목표로 합니다.

## ✨ 주요 기능 (Features)

  * **자연어 명령 기반 임무 수행**: "905호실 환자를 412호 수술실로 이송해 줘"와 같은 일상적인 언어 명령을 LLM이 분석하여 로봇의 작업 계획(JSON)으로 자동 변환합니다.
  * **자율주행 및 장애물 회피**: SLAM(Cartographer)으로 제작된 병원 지도를 기반으로, ROS 2 Nav2 스택을 활용하여 복잡한 환경에서도 안정적으로 목표 지점까지 이동합니다.
  * **실시간 원격 제어 및 모니터링**: PyQt5로 제작된 원격 GUI 컨트롤러를 통해 PC에서 로봇의 상태를 실시간으로 확인하고 새로운 명령을 내릴 수 있습니다.
  * **엣지 디바이스 기반 AI 추론**: 모델 양자화(GGUF) 기술을 통해 경량화된 LLM(Phi-3.5-mini)을 Raspberry Pi 5에서 직접 구동하여, 저전력 고효율의 AI 시스템을 구현했습니다.
  * **안정적인 시스템 관리**: ROS 2 Lifecycle Management를 도입하여 각 노드의 상태를 체계적으로 관리하고, 오류 발생 시 시스템의 강건성을 확보했습니다.

## 🛠️ 시스템 아키텍처 (System Architecture)

### 하드웨어 아키텍처

| 컴포넌트 | 모델명 / 역할 |
| :--- | :--- |
| **Main Computer** | Raspberry Pi 5 (8GB) |
| **MCU** | OpenCR Board |
| **LiDAR** | YDLIDAR G6 |
| **Actuator** | Dynamixel (for Motion Control) |
| **IMU** | 9-Axis Sensor |

### 소프트웨어 아키텍처

| 노드 (Node) | 역할 | 핵심 기술 |
| :--- | :--- | :--- |
| **LlmNode** | 자연어 명령을 작업 계획(JSON)으로 변환 | **LLM (Phi-3.5-mini)**, **4bit Quantization (GGUF)** |
| **ControlNode** | LLM의 계획을 해석하여 각 노드에 명령 전달 | **Task Orchestration** |
| **SlamNode** | 자율주행 실행 및 결과 보고 | **ROS 2 Nav2**, **AMCL**, **Cartographer** |
| **PiMonitorNode** | Raspberry Pi 하드웨어 상태 모니터링 | `psutil`, Custom ROS Message |
| **LLMController** | 원격 PC GUI 컨트롤러 | **PyQt5**, Multi-threading |

## 🚀 시작하기 (Getting Started)

### 사전 요구사항

  * **Robot H/W**: Raspberry Pi 5, OpenCR, YDLIDAR G6, Dynamixel XL430-W250-T
  * **Robot S/W**: Ubuntu 22.04, ROS 2 Humble by docker
  * **PC S/W**: Ubuntu 22.04 (Conda 환경 권장), ROS 2 Humble, PyQt5

### 설치 및 실행

1.  **GitHub 저장소 복제:**

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/NMHK-134711/hope_pj.git
    ```

2.  **의존성 패키지 설치 및 빌드:**

    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    ```

3.  **로봇 노드 실행 (Raspberry Pi):**

    ```bash
    source ~/ros2_ws/install/setup.bash
    # (예시) 각 노드를 별도의 터미널에서 실행
    ros2 run hope_pj llm_node
    ros2 run hope_pj control_node
    ros2 run hope_pj mock_slam_node # 또는 실제 SlamNode
    ros2 run hope_pj pi_monitor_node
    ```

4.  **원격 컨트롤러 실행 (PC):**

    ```bash
    # ROS 2 및 Conda 환경 활성화
    source /opt/ros/humble/setup.bash
    conda activate your_ros_env

    # 컨트롤러 실행
    python3 /path/to/hope_pj/main_controller.py
    ```

**뚝딱이와 딸깍이** 팀.
