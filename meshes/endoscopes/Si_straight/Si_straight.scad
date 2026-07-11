/*
  Parametric da Vinci Si straight endoscope + camera head model.

  Units are meters.  The model frame matches ECM_adaptor_link:
    - +Z is the main axis toward the distal endoscope tip.
    - Z=0 is the front of the camera-head body.
    - The camera head extends toward -Z.
    - camera_tip_z is mirrored in urdf/common/endoscopes/Si_straight.urdf.xacro.

  Dimensions are intentionally plain-text and approximate.  The key measured
  features are:
    - camera head: 110 mm long, 85 mm front width, 100 mm rear width, 75 mm high
    - black octagonal camera-head collar: 68 mm wide, 50 mm long
    - black ribbed endoscope sleeve: 63 mm diameter, 70 mm long
    - neck: 35 mm diameter, 21 mm long
    - cone: 17 mm long
    - shaft: 12 mm diameter, 600 mm long

  Render individual meshes with:
    openscad --export-format binstl -D 'part="body"' -o Si_straight_body.stl Si_straight.scad
    openscad --export-format binstl -D 'part="black"' -o Si_straight_black.stl Si_straight.scad
    openscad --export-format binstl -D 'part="metal"' -o Si_straight_metal.stl Si_straight.scad
    openscad --export-format binstl -D 'part="green"' -o Si_straight_green.stl Si_straight.scad
    openscad --export-format binstl -D 'part="lens"' -o Si_straight_lens.stl Si_straight.scad
    openscad --export-format binstl -D 'part="collision"' -o Si_straight_collision.stl Si_straight.scad
*/

part = "all"; // all, body, black, metal, green, lens, collision
$fn = 64;

camera_head_length = 0.110;
camera_head_front_width = 0.085;
camera_head_rear_width = 0.100;
camera_head_height = 0.075;
camera_head_radius = 0.009;

oct_collar_length = 0.050;
oct_collar_radius = 0.034;
oct_collar_bore_radius = 0.023;

ribbed_sleeve_length = 0.070;
ribbed_sleeve_radius = 0.0315;
ribbed_sleeve_core_radius = 0.0285;
rib_count = 12;
rib_width = 0.0052;
rib_depth = 0.0030;
rib_overlap = 0.0015;
part_overlap = 0.0020;

neck_length = 0.021;
neck_radius = 0.0175;
cone_length = 0.017;
shaft_length = 0.600;
shaft_radius = 0.006;

z_camera_body_start = -camera_head_length;
z_camera_body_end = 0.000;
z_oct_start = z_camera_body_end;
z_oct_end = z_oct_start + oct_collar_length;
z_ribbed_start = z_oct_end;
z_ribbed_end = z_ribbed_start + ribbed_sleeve_length;
z_neck_start = z_ribbed_end;
z_neck_end = z_neck_start + neck_length;
z_cone_start = z_neck_end;
z_cone_end = z_cone_start + cone_length;
z_shaft_start = z_cone_end;
camera_tip_z = z_shaft_start + shaft_length;

lens_face_thickness = 0.0015;
lens_radius = 0.0022;
lens_spacing = 0.0046;

module rounded_box(size, radius) {
  hull() {
    for (x = [-size[0] / 2 + radius, size[0] / 2 - radius])
      for (y = [-size[1] / 2 + radius, size[1] / 2 - radius])
        for (z = [-size[2] / 2 + radius, size[2] / 2 - radius])
          translate([x, y, z]) sphere(r = radius, $fn = 24);
  }
}

module z_cylinder(z0, z1, radius) {
  translate([0, 0, z0]) cylinder(h = z1 - z0, r = radius);
}

module z_rounded_box(z0, z1, size_xy, radius) {
  translate([0, 0, (z0 + z1) / 2])
    rounded_box([size_xy[0], size_xy[1], z1 - z0], radius);
}

module tapered_rounded_box_z(z0, z1, front_width, rear_width, height, radius) {
  hull() {
    for (x = [-rear_width / 2 + radius, rear_width / 2 - radius])
      for (y = [-height / 2 + radius, height / 2 - radius])
        translate([x, y, z0 + radius]) sphere(r = radius, $fn = 24);

    for (x = [-front_width / 2 + radius, front_width / 2 - radius])
      for (y = [-height / 2 + radius, height / 2 - radius])
        translate([x, y, z1 - radius]) sphere(r = radius, $fn = 24);
  }
}

module tapered_z_cylinder(z0, z1, r0, r1) {
  translate([0, 0, z0]) cylinder(h = z1 - z0, r1 = r0, r2 = r1);
}

module z_octagonal_ring(z0, z1, outer_radius, inner_radius) {
  difference() {
    rotate([0, 0, 22.5])
      translate([0, 0, z0])
        cylinder(h = z1 - z0, r = outer_radius, $fn = 8);
    translate([0, 0, z0 - 0.001])
      cylinder(h = z1 - z0 + 0.002, r = inner_radius, $fn = 48);
  }
}

module camera_body() {
  // Main grey camera module, wider at the rear and narrower at the collar.
  tapered_rounded_box_z(z_camera_body_start, z_camera_body_end,
                        camera_head_front_width, camera_head_rear_width,
                        camera_head_height, camera_head_radius);
}

module black_parts() {
  // Front camera-head collar that covers the endoscope rear cylinder.
  z_octagonal_ring(z_oct_start, z_oct_end, oct_collar_radius,
                   oct_collar_bore_radius);

  // Main ribbed endoscope sleeve.
  z_cylinder(z_ribbed_start - part_overlap, z_ribbed_end,
             ribbed_sleeve_core_radius);
  for (angle = [0 : 360 / rib_count : 360 - 360 / rib_count]) {
    rotate([0, 0, angle])
      translate([ribbed_sleeve_core_radius + rib_depth / 2 - rib_overlap, 0,
                 (z_ribbed_start + z_ribbed_end - part_overlap) / 2])
        cube([rib_depth + 2 * rib_overlap, rib_width,
              ribbed_sleeve_length + part_overlap], center = true);
  }

  // Top button panel on the camera head.
  translate([0, camera_head_height / 2 + 0.0012, -0.056])
    rounded_box([0.054, 0.004, 0.064], 0.002);
}

module metal_parts() {
  // Internal silver cylinder visible through and after the black collar.
  z_cylinder(z_oct_start + 0.006, z_ribbed_start + 0.006, 0.020);

  // Silver transition after the black ribbed sleeve.
  z_cylinder(z_neck_start, z_neck_end, neck_radius);
  tapered_z_cylinder(z_cone_start, z_cone_end, neck_radius, shaft_radius);
  z_cylinder(z_shaft_start, camera_tip_z, shaft_radius);

}

module green_parts() {
  // Green da Vinci Si camera-head trim rails, simplified from photos.
  for (x = [-0.027, 0.027])
    translate([x, camera_head_height / 2 + 0.0035, -0.058])
      rounded_box([0.010, 0.005, 0.092], 0.002);
  translate([0, camera_head_height / 2 + 0.004, -0.020])
    rounded_box([0.060, 0.004, 0.018], 0.002);
  translate([0, camera_head_height / 2 + 0.004, -0.098])
    rounded_box([0.060, 0.004, 0.014], 0.002);
}

module lens_parts() {
  // Distal stereo face at the endoscope tip.
  translate([0, 0, camera_tip_z])
    cylinder(h = lens_face_thickness, r = shaft_radius * 0.92);
  translate([-lens_spacing / 2, 0, camera_tip_z + lens_face_thickness * 0.2])
    cylinder(h = lens_face_thickness * 0.8, r = lens_radius);
  translate([ lens_spacing / 2, 0, camera_tip_z + lens_face_thickness * 0.2])
    cylinder(h = lens_face_thickness * 0.8, r = lens_radius);

  // Three raised camera-head buttons on the top panel.
  for (x = [-0.017, 0, 0.017])
    translate([x, camera_head_height / 2 + 0.004, -0.060])
      rotate([90, 0, 0])
        cylinder(h = 0.004, r = 0.006, center = true);
}

module collision() {
  camera_body();
  z_cylinder(z_oct_start, z_ribbed_end, ribbed_sleeve_radius);
  z_cylinder(z_neck_start, z_neck_end, neck_radius);
  tapered_z_cylinder(z_cone_start, z_cone_end, neck_radius, shaft_radius);
  z_cylinder(z_shaft_start, camera_tip_z, shaft_radius);
}

if (part == "body") {
  camera_body();
} else if (part == "black") {
  black_parts();
} else if (part == "metal") {
  metal_parts();
} else if (part == "green") {
  green_parts();
} else if (part == "lens") {
  lens_parts();
} else if (part == "collision") {
  collision();
} else {
  camera_body();
  black_parts();
  metal_parts();
  green_parts();
  lens_parts();
}
