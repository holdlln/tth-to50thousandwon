"""Create Korean validation report from completed evaluator evidence."""
import json
from pathlib import Path
import platform
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from sar.evaluation import mission_success


def main(run_id):
    source=ROOT/'output/runs'/run_id
    evaluation=json.loads((source/'evaluation.json').read_text())
    replay=mission_success(evaluation['controller_result'],len(evaluation['visited']),
                           evaluation['total_targets'],evaluation['home_error_m'],
                           evaluation['home_angle_error_rad'],evaluation['collision_episodes'],
                           evaluation['elapsed_seconds'],600)
    if replay!=evaluation['success']: raise ValueError('Saved result differs from final evaluation criteria')
    analysis=json.loads((ROOT/'output/validation/analysis.json').read_text())
    if analysis['run_id']!=run_id: raise ValueError('Analyze the same run first')
    tests=json.loads((ROOT/'output/validation/tests.json').read_text(encoding='utf-8'))
    if tests['result']!='OK': raise ValueError('Core tests must pass before writing the report')
    history=[]
    for path in sorted((ROOT/'output/runs').glob('*/evaluation.json')):
        if path.parent.name.startswith('calibration'): continue
        e=json.loads(path.read_text())
        history.append(dict(run_id=path.parent.name,success=e['success'],visited=len(e['visited']),
                            elapsed_seconds=e['elapsed_seconds'],collisions=e['collision_episodes'],
                            stop=e.get('controller_result')))
    (ROOT/'output/validation/history.json').write_text(json.dumps(history,indent=2),encoding='utf-8')
    visits='\n'.join(f'| {key} | {value:.3f}초 |' for key,value in evaluation['visited'].items())
    status='성공' if evaluation['success'] else '실패 / 미완료'
    text=f'''# 실행 검증 결과

기본 중간 난이도 맵을 설치된 Webots R2025a에서 실행한 결과: **{status}**.

검증 실행 ID: `{run_id}`. 운영체제: Windows. Python: {platform.python_version()}. NumPy 기반 코어 테스트 {tests['count']}개 통과 (`output/validation/tests.log`에 실제 실행 출력 보존). 현재 보고서는 한 번의 완료된 기본 맵 실행을 증명하며, 다른 맵과 센서 잡음 조건 전반의 성공률을 의미하지 않는다.

| 지표 | 독립 평가기 측정값 |
|---|---:|
| 인정된 목표 방문 | {len(evaluation['visited'])}/{evaluation['total_targets']} |
| 경과 시간 | {evaluation['elapsed_seconds']:.3f}초 / 데모 제한 600초 |
| 실제 주행 거리 | {evaluation['path_length_m']:.3f}m |
| 충돌 발생 횟수 | {evaluation['collision_episodes']} |
| 최소 여유 거리 | {evaluation['minimum_clearance_m']:.3f}m |
| 복귀 위치 오차 | {evaluation['home_error_m']:.3f}m |
| 복귀 방향 오차 | {evaluation['home_angle_error_rad']:.3f}rad |
| 평균 위치 추정 오차, 약 1초 표본 | {analysis['mean_sampled_position_error_m']:.3f}m |
| 최대 위치 추정 오차, 약 1초 표본 | {analysis['maximum_sampled_position_error_m']:.3f}m |

최소 여유 거리는 평가기의 보수적인 원형 로봇 반경 0.23m와 장애물 경계 사이 거리다. 원의 지름/중심 간 거리와 혼동하지 않는다. 표본 위치 오차는 가까운 시각의 정답 궤적과 비교하므로 약 0.1초의 시각 차이가 포함될 수 있다. 로봇 내부의 uncertainty 값은 통계적인 공분산으로 보정되지 않은 휴리스틱이다.

## 각 목표 방문 시각

| 목표 | 평가기 인정 시각 |
|---|---:|
{visits}

## 평가 조건과 정보 분리

성공은 모든 목표의 종류와 위치 확인, 각각에 실제로 접근, 출발점 복귀, 제한 시간 준수, 충돌 0회, 로봇의 정상 완료 이벤트가 동시에 충족되어야 한다. 방문 이벤트의 추정 좌표가 정답 객체에서 0.55m 이내이고 로봇 실제 위치가 객체 중심에서 0.95m 이내일 때 방문을 인정한다. 복귀 실제 위치 오차는 0.35m 미만, 방향 오차는 0.3rad 미만이어야 한다. 이는 공식 대회 배점이 아닌 자체 검증 조건이다. 정상 완료 이벤트를 요구하는 최종 평가 함수는 저장된 실행 결과에도 재적용해 통과를 확인했다.

`scenario_supervisor`만 맵/정답 위치/순찰 경로를 읽는다. 로봇은 센서 관측을 기반으로 위치·지도·목표를 계산한다. 평가기는 로봇에게 정답 위치나 방문 정답을 돌려주지 않는다. 비교 그림의 청록 궤적은 평가기의 정답 기록, 자주색 궤적은 로봇의 추정 기록이다.

## 핵심 검증 범위

- 엔코더 직진과 회전, 자이로 회전 보완, 스캔 정합의 회전 오차 감소.
- Occupancy Grid의 무한대/NaN 처리, 단일 근거리 이상 레이를 미관측으로 유지.
- A*의 장애물 우회 및 대각선 코너 관통 금지.
- 정적 맵에서 출발점과 모든 목표의 접근 영역 사이 경로 존재.
- 실제 RGB 배열과 LiDAR를 연결하는 탐지, 전경 장애물 거리 오연결 거부.
- 여러 관측으로 확인하고 같은 대상 후보를 병합하는 동작.
- 충돌 직전 정지, 다가오는 장애물에 후진 후보 선택, 회전 중 전진 명령 제거.
- 장애물 팽창 띠에서 관측된 빈 공간으로 회복하는 목표 선택.
- 불량 스캔 정지, 시간 초과 실패, 목표 미완료 복귀의 성공 처리 금지.
- 로봇 코드의 Supervisor 및 정답 시나리오 의존 금지.

## 검증 중 발견하고 수정한 문제

1. 수동 지지구가 동적 본체의 충돌 형상에 포함되지 않아 기울어진 차체를 수정했다. 전후 지지구를 본체 boundingObject에 포함했다.
2. 격자 지도에 대한 반복 보정의 양의 피드백을 줄이기 위해 키프레임 기반 point-to-line 정합으로 바꿨다. 이후 신뢰할 수 있는 엔코더 이동량·자이로 회전율보다 스캔 보정이 과도하게 누적되지 않도록 보정 비중을 조정했다. 회전율을 32ms마다 적분해 128ms 제어 주기의 급회전 샘플링 오차를 줄였다.
3. 접근 셀 도착 허용 오차와 방문 허용 거리의 불일치로 정체하는 문제를 수정했다. 복귀에서도 마지막 격자 셀을 제공된 연속 출발 좌표로 연결해 도착 범위의 불일치로 멈추지 않도록 했다.
4. 모서리에서 추정 위치가 지도 팽창 띠에 들어가 경로가 사라지는 경우 센서 안전검사를 유지한 회복 경로를 추가했다.
5. 멀리 있는 표식의 영상에 전경 장애물의 거리가 붙는 경우 시각적 폭과 거리의 일치도를 검사한다.
6. 이동 장애물이 멈춘 로봇에 다가오는 상황을 위해 후진 후보와 이동 군집의 넓은 근거리 관측 범위를 추가했다.
7. 정답 형상과 맞지 않고 양옆 레이보다 매우 짧은 단일 거리값을 미확인으로 처리했다. 센서/렌더링 측의 정확한 발생 원인은 확정하지 않았다.

초기 실행과 중단한 진단 실행의 결과는 `output/validation/history.json`에 남겼다. 실패나 수동 중단을 성공으로 바꾸지 않았다. 현재 완료 결과는 별도 `evaluation.json`에 그대로 보존했다.

## 남아 있는 범위

현재는 2D 실내, 일정 높이 장애물, 색상과 실제 폭을 아는 단순 표식, 완만하게 왕복 이동하는 장애물의 데모다. 연기·낙하·계단·가파른 지형·영상 품질 저하·실제 배터리·무선 통신·실제 구조 대상 운반은 검증하지 않았다. pose graph/loop closure, 공분산 보정, 사람의 의도 추론은 추가 작업이다. 얇은 케이블/가느다란 장애물은 현재 근거리 이상 레이 필터의 가정을 재검토해야 한다.

목표의 실제 폭, 개수, 접근 거리, 시간 제한은 당일 규칙을 확인해야 한다. 실제 폭이 제공되지 않으면 `target_width`를 임의로 추측하지 말고 탐지기를 교체하거나 다중 시점/깊이 일치 검사를 확장한다. Ubuntu 22.04/Python 3.10 실행 절차는 README에 있지만 현재 OS에서 검증하지 않았다.

## 결과 그림

![중간 난이도 시나리오](../output/validation/scenario_map.png)

![실제/추정 주행 궤적](../output/validation/run_analysis.png)

![로봇이 센서로 작성한 지도 - 회색은 미관측](../output/validation/robot_map.png)

Webots 창을 숨긴 batch 모드에서 전체 화면 이미지는 일부 실행에서 크기가 비정상적이었다. 그 화면 캡처는 결과물에 포함하지 않았다. 위 그림은 실제 로그와 지도 배열을 그린 것이며, `final_camera.png`는 실제 RGB 센서 배열이다.
'''
    (ROOT/'docs/검증결과.md').write_text(text,encoding='utf-8')
    print('Validation report created')


if __name__=='__main__': main(sys.argv[1])
