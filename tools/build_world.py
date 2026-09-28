"""Generate a self-contained Webots world from scenario JSON (evaluator-only)."""
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]


def box(name, data, height, color):
    x,y,sx,sy=data
    return f'''DEF {name} Solid {{
  translation {x} {y} {height/2}
  name "{name}"
  children [ Shape {{
    appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 }}
    geometry Box {{ size {sx} {sy} {height} }}
  }} ]
  boundingObject Box {{ size {sx} {sy} {height} }}
}}'''


def build(scenario_path=None, output=None):
    scenario_file=Path(scenario_path or ROOT/'config/scenario.json').resolve()
    scenario=json.loads(scenario_file.read_text(encoding='utf-8'))
    try:
        scenario_reference=scenario_file.relative_to(ROOT).as_posix()
    except ValueError as error:
        raise ValueError('scenario must be inside the project directory') from error
    config=json.loads((ROOT/'config/robot.json').read_text(encoding='utf-8'))
    sx,sy=scenario['size']
    parts=['''#VRML_SIM R2025a utf8
EXTERNPROTO "../protos/RescueBot.proto"
WorldInfo {
  title "AMR Sensor-only Search and Rescue"
  basicTimeStep 32
  coordinateSystem "ENU"
  contactProperties [
    ContactProperties { material1 "wheel" material2 "floor" coulombFriction [ 0.9 ] }
    ContactProperties { material1 "caster" material2 "floor" coulombFriction [ 0 ] }
  ]
}
Viewpoint {
  orientation 0 1 0 1.57079632679
  position 0 0 16
}
Background { skyColor [ 0.08 0.11 0.16 ] }
DirectionalLight { direction -0.2 -0.3 -1 intensity 2 ambientIntensity 0.5 }
''',f'''DEF FLOOR Solid {{
  translation 0 0 -0.05
  name "floor"
  contactMaterial "floor"
  children [ Shape {{
    appearance PBRAppearance {{ baseColor 0.38 0.4 0.42 roughness 1 metalness 0 }}
    geometry Box {{ size {sx} {sy} 0.1 }}
  }} ]
  boundingObject Box {{ size {sx} {sy} 0.1 }}
}}''']
    for i,data in enumerate(scenario['walls']):
        parts.append(box(f'WALL_{i}',data,scenario['wall_height'],'0.56 0.58 0.6'))
    for i,data in enumerate(scenario['debris']):
        parts.append(box(f'DEBRIS_{i}',data,.65,'0.32 0.30 0.28'))
    radius,height=scenario['target_radius'],scenario['target_height']
    for target in scenario['targets']:
        x,y=target['position']; color=' '.join(str(v/255) for v in target['rgb'])
        parts.append(f'''DEF {target['id']} Solid {{
  translation {x} {y} {height/2}
  name "rescue target {target['id']}"
  children [ Shape {{
    appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 }}
    geometry Cylinder {{ radius {radius} height {height} subdivision 24 }}
  }} ]
  boundingObject Cylinder {{ radius {radius} height {height} }}
}}''')
    for person in scenario['pedestrians']:
        x,y=person['start']; radius=scenario['pedestrian_radius']
        parts.append(f'''DEF {person['id']} Solid {{
  translation {x} {y} 0
  name "moving obstacle {person['id']}"
  children [
    Pose {{ translation 0 0 0.4 children [ Shape {{
      appearance PBRAppearance {{ baseColor 0.2 0.2 0.2 roughness 1 metalness 0 }}
      geometry Cylinder {{ radius {radius} height 0.8 }}
    }} ] }}
    Pose {{ translation 0 0 0.94 children [ Shape {{
      appearance PBRAppearance {{ baseColor 0.65 0.65 0.65 roughness 1 metalness 0 }}
      geometry Sphere {{ radius 0.13 }}
    }} ] }}
  ]
  boundingObject Pose {{ translation 0 0 0.4 children [ Cylinder {{ radius {radius} height 0.8 }} ] }}
}}''')
    x,y,yaw=config['initial_pose']
    parts.append(f'DEF RESCUE_ROBOT RescueBot {{ translation {x} {y} 0 rotation 0 0 1 {yaw} }}')
    parts.append(f'''DEF EVALUATOR Robot {{
  name "Scenario evaluator"
  supervisor TRUE
  customData "{scenario_reference}"
  controller "scenario_supervisor"
  children [ Receiver {{ name "mission events" channel 17 }} ]
}}''')
    out=Path(output or ROOT/'worlds/rescue_medium.wbt')
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text('\n\n'.join(parts)+'\n',encoding='utf-8')
    print(f'World generated: {out}')
    return out


if __name__=='__main__':
    build(*sys.argv[1:3])
