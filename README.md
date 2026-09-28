# AMR Search and Rescue - TECH WEEK

센서로 미지 환경을 탐색하고 모든 표식 목표에 안전하게 접근한 뒤 출발점으로 복귀하는 Webots 프로젝트입니다.

- 자세한 설계, 실제 구조 상황의 고려 사항, 창의적 요소: [설계와 대회 전략](docs/설계와_대회전략.md)
- 실행 검증 및 알려진 한계: [검증 결과](docs/검증결과.md)
- Worlds: `worlds/rescue_medium.wbt`, `worlds/rescue_loop.wbt`, `worlds/rescue_zigzag.wbt`,
  `worlds/rescue_crossroads.wbt`, `worlds/rescue_courtyard.wbt`
- Robot: `protos/RescueBot.proto`

## Windows에서 실행

설치된 Webots R2025a와 NumPy를 포함하는 Python 3.10 이상을 사용합니다. 현재 PC에서는 실행 스크립트가 Codex의 번들 Python도 찾아 사용합니다.

```powershell
# 일반 Python을 직접 지정할 때
$env:SAR_PYTHON = 'C:\path\to\python.exe'
& $env:SAR_PYTHON -m pip install -r requirements.txt
& $env:SAR_PYTHON tools/build_world.py

# Webots 창을 일시정지 상태로 열고 재생 버튼으로 시작
.\scripts\run_webots.ps1

# 자동 평가: 미션 종료 시 Webots 종료, 로그 보존
.\scripts\run_webots.ps1 -Batch -RunId my_test
```

현재 PC에서 `$env:SAR_PYTHON` 지정 없이 `scripts/run_webots.ps1`을 실행하면 번들 Python을 사용합니다. Webots GUI에서 파일만 직접 여는 경우, 먼저 이 스크립트로 실행하거나 두 컨트롤러의 `runtime.ini`에 원하는 Python COMMAND를 지정해야 합니다.

## Ubuntu 22.04에서 실행

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
export SAR_PYTHON="$(which python)"
export SAR_RUN_ID="linux_demo"
python tools/build_world.py
webots --mode=pause worlds/rescue_medium.wbt
```

추가 맵은 다음처럼 실행합니다.

```powershell
webots --mode=pause worlds/rescue_loop.wbt
webots --mode=pause worlds/rescue_zigzag.wbt
webots --mode=pause worlds/rescue_crossroads.wbt
webots --mode=pause worlds/rescue_courtyard.wbt
```

각 추가 맵의 원본 시나리오는 같은 이름의 `config/scenario_*.json` 파일이며,
평가기는 월드에 연결된 시나리오를 자동으로 사용합니다.

Linux 실행 절차는 제공하지만 현재 PC에서 검증한 운영체제는 Windows입니다.

## 설정과 코드

`config/robot.json`: 시작 pose, 바퀴 치수, 안전거리, 시간 제한, 목표 시각 특징. 시작 pose를 바꾸면 월드도 다시 생성합니다. 로봇/센서 물리 치수 변경 시 PROTO와 설정을 함께 변경합니다. `config/scenario.json`: 맵, 장애물, 목표의 정답 위치, 순찰 경로. **시나리오 설정은 월드 생성기와 평가기만 읽습니다.**

`sar/`의 알고리즘은 Webots 없이 테스트할 수 있고 `Mission(detector=..., frontier_policy=...)`로 인식기와 탐색 전략을 주입할 수 있습니다. 모터 명령은 `(v m/s, w rad/s)`이고 어댑터가 좌/우 바퀴 rad/s로 변환합니다.

```bash
python -m unittest discover -s tests -v
python tools/run_tests.py  # 실행 결과를 output/validation/tests.log에 저장
python tools/run_batch.py --run-id validation
```

각 실행은 `output/runs/<run-id>/`에 로봇 telemetry, 방문 events, 독립 평가, 정답 궤적, 최종 지도와 이미지를 저장합니다. `evaluation.json.success`는 **모든 목표 방문 + 복귀 + 제한 시간 + 충돌 0회**를 평가한 데모 결과입니다. 공식 대회 배점으로 환산한 점수는 아닙니다. `ground_truth.jsonl`은 분석용이며 로봇은 읽지 않습니다.

수정 중인 검증 실행을 안전하게 끝내려면 해당 실행 폴더에 빈 `STOP` 파일을 만듭니다. 이 종료는 성공 처리되지 않습니다.
