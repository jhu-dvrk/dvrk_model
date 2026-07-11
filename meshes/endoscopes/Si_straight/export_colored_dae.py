#!/usr/bin/env python3
"""Generate colored Collada visual meshes from the OpenSCAD STL outputs."""

from pathlib import Path
import struct
from xml.sax.saxutils import escape


ROOT = Path(__file__).resolve().parent

PARTS = {
    "body": ((0.34, 0.38, 0.40, 1.0), "SiEndoscopeCameraBody"),
    "black": ((0.015, 0.015, 0.014, 1.0), "SiEndoscopeBlack"),
    "metal": ((0.72, 0.72, 0.70, 1.0), "SiEndoscopeMetal"),
    "green": ((0.00, 0.62, 0.43, 1.0), "SiEndoscopeGreen"),
    "lens": ((0.005, 0.006, 0.008, 1.0), "SiEndoscopeLens"),
}


def read_binary_stl(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + count * 50
    if len(data) != expected:
        raise RuntimeError(f"{path} is not a binary STL with expected size")

    vertices = []
    triangles = []
    offset = 84
    for _ in range(count):
        offset += 12
        triangle = []
        for _ in range(3):
            triangle.append(len(vertices))
            vertices.append(struct.unpack_from("<fff", data, offset))
            offset += 12
        triangles.append(triangle)
        offset += 2
    return vertices, triangles


def write_dae(part, color, material_name):
    stl_path = ROOT / f"Si_straight_{part}.stl"
    dae_path = ROOT / f"Si_straight_{part}.dae"
    vertices, triangles = read_binary_stl(stl_path)

    positions = " ".join(f"{coordinate:.9g}" for vertex in vertices for coordinate in vertex)
    indices = " ".join(str(index) for triangle in triangles for index in triangle)
    material_id = material_name.replace(" ", "_")
    geometry_id = f"Si_straight_{part}_geometry"
    mesh_name = f"Si_straight_{part}"
    r, g, b, a = color

    dae_path.write_text(f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>dvrk_model OpenSCAD STL-to-DAE generator</authoring_tool></contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_effects>
    <effect id="{material_id}-effect">
      <profile_COMMON>
        <technique sid="common">
          <phong>
            <ambient><color>{r:.6g} {g:.6g} {b:.6g} {a:.6g}</color></ambient>
            <diffuse><color>{r:.6g} {g:.6g} {b:.6g} {a:.6g}</color></diffuse>
          </phong>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="{material_id}" name="{escape(material_name)}">
      <instance_effect url="#{material_id}-effect"/>
    </material>
  </library_materials>
  <library_geometries>
    <geometry id="{geometry_id}" name="{mesh_name}">
      <mesh>
        <source id="{geometry_id}-positions">
          <float_array id="{geometry_id}-positions-array" count="{len(vertices) * 3}">{positions}</float_array>
          <technique_common>
            <accessor source="#{geometry_id}-positions-array" count="{len(vertices)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="{geometry_id}-vertices">
          <input semantic="POSITION" source="#{geometry_id}-positions"/>
        </vertices>
        <triangles material="{material_id}" count="{len(triangles)}">
          <input semantic="VERTEX" source="#{geometry_id}-vertices" offset="0"/>
          <p>{indices}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="{mesh_name}" name="{mesh_name}">
        <instance_geometry url="#{geometry_id}">
          <bind_material>
            <technique_common>
              <instance_material symbol="{material_id}" target="#{material_id}"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
""")
    print(dae_path.name)


def main():
    for part, (color, material_name) in PARTS.items():
        write_dae(part, color, material_name)


if __name__ == "__main__":
    main()
