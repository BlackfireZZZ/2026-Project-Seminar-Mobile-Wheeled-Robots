"""Generate the two static digit PROTOs and Webots world for a variant.

Run before colcon build: python3 generate_scene.py 02
"""

from pathlib import Path
import sys


SEGMENTS = {
    'a': (0.0, 1.2, 1.42, 0.16),
    'b': (0.65, 0.6, 0.16, 1.2),
    'c': (0.65, -0.6, 0.16, 1.2),
    'd': (0.0, -1.2, 1.42, 0.16),
    'e': (-0.65, -0.6, 0.16, 1.2),
    'f': (-0.65, 0.6, 0.16, 1.2),
    'g': (0.0, 0.0, 1.42, 0.16),
}
DIGITS = {
    '0': 'abcdef', '1': 'bc', '2': 'abged', '3': 'abgcd',
    '4': 'fgbc', '5': 'afgcd', '6': 'afgecd', '7': 'abc',
    '8': 'abcdefg', '9': 'abfgcd',
}
ROOT = Path(__file__).resolve().parent


def proto(digit):
    shapes = []
    collision = []
    for segment in DIGITS[digit]:
        x, y, width, length = SEGMENTS[segment]
        shapes.append(f'''    Transform {{
      translation {x} {y} 0.14
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor 0.42 0.44 0.47 roughness 1 metalness 0 }}
          geometry Box {{ size {width} {length} 0.28 }}
        }}
      ]
    }}''')
        collision.append(f'''      Transform {{
        translation {x} {y} 0.14
        children [ Box {{ size {width} {length} 0.28 }} ]
      }}''')
    return f'''#VRML_SIM R2025a utf8

PROTO Digit{digit} [
  field SFVec3f translation 0 0 0
  field SFString name "digit_{digit}"
]
{{
  Solid {{
    translation IS translation
    name IS name
    children [
{chr(10).join(shapes)}
    ]
    boundingObject Group {{
      children [
{chr(10).join(collision)}
      ]
    }}
  }}
}}
'''


def world(left, right):
    imports = '\n'.join(
        f'EXTERNPROTO "../protos/Digit{digit}.proto"'
        for digit in dict.fromkeys((left, right))
    )
    return f'''#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/devices/robotis/protos/RobotisLds01.proto"
{imports}

WorldInfo {{
}}
Viewpoint {{
  orientation -0.577350269 0.577350269 0.577350269 2.094395102
  position 0 0 14.5
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
RectangleArena {{
  floorSize 8 8
  wallThickness 0.05
  wallHeight 0.35
  floorTileSize 0.8 0.8
}}
Digit{left} {{
  translation -1 0 0
  name "left_{left}"
}}
Digit{right} {{
  translation 1 0 0
  name "right_{right}"
}}
TurtleBot3Burger {{
  translation 2.5 0 0
  rotation 0 0 1 1.57079632679
  controller "<extern>"
  controllerArgs [ "" ]
  extensionSlot [
    Solid {{ name "imu_link" }}
    GPS {{ }}
    InertialUnit {{ name "inertial_unit" }}
    RobotisLds01 {{ }}
  ]
}}
'''


def main():
    if len(sys.argv) != 2 or len(sys.argv[1]) != 2 or any(
        digit not in DIGITS for digit in sys.argv[1]
    ):
        raise SystemExit('Usage: python3 generate_scene.py <two-digit variant>')
    left, right = sys.argv[1]
    for old in (ROOT / 'protos').glob('Digit*.proto'):
        old.unlink()
    for digit in set((left, right)):
        (ROOT / 'protos' / f'Digit{digit}.proto').write_text(proto(digit))
    (ROOT / 'worlds' / 'number_world.wbt').write_text(world(left, right))


if __name__ == '__main__':
    main()
