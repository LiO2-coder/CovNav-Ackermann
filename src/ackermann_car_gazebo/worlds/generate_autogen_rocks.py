# -*- coding: utf-8 -*-

"""Generate small red-rock obstacles for the autogen terrain world."""

from pathlib import Path
import math
import random
from xml.sax.saxutils import escape

from PIL import Image


# 石块总数量
ROCK_COUNT = 300

# 中心密集分布半径，单位：米
DENSE_RADIUS_M = 20.0

# 落在中心密集区域内的石块比例
DENSE_RATIO = 1.0

# 石块长度范围，单位：米，2-10cm
LENGTH_RANGE_M = (0.02, 0.10)

# 石块宽度范围，单位：米，2-10cm
WIDTH_RANGE_M = (0.02, 0.10)

# 石块高度范围，单位：米，1-5cm
HEIGHT_RANGE_M = (0.01, 0.05)

# 石块密度，单位：kg/m^3；数值越大越不容易被撞动，澳洲砂岩/红土岩可取约 2200
ROCK_DENSITY_KG_M3 = 2200.0

# 石块之间是否互相碰撞；动态石块数量多且允许少量重叠，默认关闭以避免启动时弹飞
ROCK_SELF_COLLIDE = False

# 随机种子，保证每次生成结果一致
RANDOM_SEED = 42

# 地形长度和宽度，单位：米，需要与 autogen_terrain/model.sdf 中 heightmap 的 size 保持一致
TERRAIN_SIZE_X_M = 129.0
TERRAIN_SIZE_Y_M = 129.0

# 地形最大高度，单位：米，需要与 autogen_terrain/model.sdf 中 heightmap 的 size 保持一致
TERRAIN_HEIGHT_M = 6.0

# 地形上表面偏移，单位：米；Gazebo 的 heightmap 有厚度，石块要放到上表面而不是下表面
TERRAIN_TOP_OFFSET_M = 3.0

# 从地形边缘向内收缩的安全距离，单位：米，避免石块压到 heightmap 边界
EDGE_MARGIN_M = 0.10

# 石块颜色：澳洲红土/赤褐色
ROCK_AMBIENT = "0.55 0.22 0.10 1"
ROCK_DIFFUSE = "0.72 0.30 0.14 1"
ROCK_SPECULAR = "0.08 0.06 0.04 1"


WORLD_DIR = Path(__file__).resolve().parent
TERRAIN_DIR = WORLD_DIR / "autogen_terrain"
HEIGHTMAP_PATH = TERRAIN_DIR / "materials" / "textures" / "heightmap.png"
OUTPUT_DIR = WORLD_DIR / "autogen_rocks"
MODEL_SDF_PATH = OUTPUT_DIR / "model.sdf"
MODEL_CONFIG_PATH = OUTPUT_DIR / "model.config"


def fmt(value):
    return f"{value:.6f}".rstrip("0").rstrip(".")


def read_heightmap():
    return Image.open(HEIGHTMAP_PATH).convert("L")


def bilinear_sample(image, u, v):
    width, height = image.size
    u = max(0.0, min(width - 1.0, u))
    v = max(0.0, min(height - 1.0, v))

    x0 = int(math.floor(u))
    y0 = int(math.floor(v))
    x1 = min(x0 + 1, width - 1)
    y1 = min(y0 + 1, height - 1)
    tx = u - x0
    ty = v - y0

    p00 = image.getpixel((x0, y0))
    p10 = image.getpixel((x1, y0))
    p01 = image.getpixel((x0, y1))
    p11 = image.getpixel((x1, y1))

    top = p00 * (1.0 - tx) + p10 * tx
    bottom = p01 * (1.0 - tx) + p11 * tx
    return top * (1.0 - ty) + bottom * ty


def terrain_height_at(image, x, y):
    half_x = TERRAIN_SIZE_X_M / 2.0
    half_y = TERRAIN_SIZE_Y_M / 2.0
    width, height = image.size

    u = (x + half_x) / TERRAIN_SIZE_X_M * (width - 1)
    v = (half_y - y) / TERRAIN_SIZE_Y_M * (height - 1)
    gray = bilinear_sample(image, u, v)
    return TERRAIN_TOP_OFFSET_M + gray / 255.0 * TERRAIN_HEIGHT_M


def sample_xy(rng):
    half_x = TERRAIN_SIZE_X_M / 2.0 - EDGE_MARGIN_M
    half_y = TERRAIN_SIZE_Y_M / 2.0 - EDGE_MARGIN_M

    if rng.random() < DENSE_RATIO:
        radius = math.sqrt(rng.random()) * DENSE_RADIUS_M
        theta = rng.uniform(0.0, math.tau)
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        return max(-half_x, min(half_x, x)), max(-half_y, min(half_y, y))

    return rng.uniform(-half_x, half_x), rng.uniform(-half_y, half_y)


def generate_rocks():
    rng = random.Random(RANDOM_SEED)
    heightmap = read_heightmap()
    rocks = []

    for index in range(ROCK_COUNT):
        length = rng.uniform(*LENGTH_RANGE_M)
        width = rng.uniform(*WIDTH_RANGE_M)
        height = rng.uniform(*HEIGHT_RANGE_M)
        x, y = sample_xy(rng)
        yaw = rng.uniform(-math.pi, math.pi)
        z = terrain_height_at(heightmap, x, y) + height / 2.0
        rocks.append(
            {
                "name": f"rock_{index:04d}",
                "pose": (x, y, z, 0.0, 0.0, yaw),
                "size": (length, width, height),
            }
        )

    return rocks


def box_mass_and_inertia(size):
    length, width, height = size
    mass = ROCK_DENSITY_KG_M3 * length * width * height
    ixx = mass * (width * width + height * height) / 12.0
    iyy = mass * (length * length + height * height) / 12.0
    izz = mass * (length * length + width * width) / 12.0
    return mass, ixx, iyy, izz


def write_model_config():
    model_config = """<?xml version='1.0' encoding='ASCII'?>
<model>
  <name>autogen_rocks</name>
  <version>1.0</version>
  <sdf version="1.4">model.sdf</sdf>
  <author>
    <name>Generated</name>
    <email>---</email>
  </author>
  <description>Auto generated small red rock obstacles for the terrain world</description>
</model>
"""
    MODEL_CONFIG_PATH.write_text(model_config, encoding="ascii")


def write_model_sdf(rocks):
    lines = [
        "<?xml version='1.0' encoding='ASCII'?>",
        '<sdf version="1.4">',
        '  <model name="autogen_rocks">',
        "    <static>false</static>",
        f"    <self_collide>{str(ROCK_SELF_COLLIDE).lower()}</self_collide>",
    ]

    for rock in rocks:
        name = escape(rock["name"])
        pose = " ".join(fmt(value) for value in rock["pose"])
        size = " ".join(fmt(value) for value in rock["size"])
        mass, ixx, iyy, izz = box_mass_and_inertia(rock["size"])
        lines.extend(
            [
                f'    <link name="{name}">',
                f"      <pose>{pose}</pose>",
                "      <inertial>",
                f"        <mass>{fmt(mass)}</mass>",
                "        <inertia>",
                f"          <ixx>{fmt(ixx)}</ixx>",
                "          <ixy>0</ixy>",
                "          <ixz>0</ixz>",
                f"          <iyy>{fmt(iyy)}</iyy>",
                "          <iyz>0</iyz>",
                f"          <izz>{fmt(izz)}</izz>",
                "        </inertia>",
                "      </inertial>",
                '      <collision name="collision">',
                "        <geometry>",
                "          <box>",
                f"            <size>{size}</size>",
                "          </box>",
                "        </geometry>",
                "      </collision>",
                '      <visual name="visual">',
                "        <geometry>",
                "          <box>",
                f"            <size>{size}</size>",
                "          </box>",
                "        </geometry>",
                "        <material>",
                f"          <ambient>{ROCK_AMBIENT}</ambient>",
                f"          <diffuse>{ROCK_DIFFUSE}</diffuse>",
                f"          <specular>{ROCK_SPECULAR}</specular>",
                "        </material>",
                "      </visual>",
                "    </link>",
            ]
        )

    lines.extend(
        [
            "  </model>",
            "</sdf>",
            "",
        ]
    )
    MODEL_SDF_PATH.write_text("\n".join(lines), encoding="ascii")


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    rocks = generate_rocks()
    write_model_config()
    write_model_sdf(rocks)
    print(f"Generated {len(rocks)} rocks in {MODEL_SDF_PATH}")


if __name__ == "__main__":
    main()
