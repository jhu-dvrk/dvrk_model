# dVRK Model Complete Testing Guide

Copy and paste each command line-by-line to test all configurations.
Press `Ctrl+C` to stop each test before running the next one.

---

## Single PSM Arms - Classic Generation

```bash
ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Classic instrument:=400006
ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Classic instrument:=400049

ros2 launch dvrk_model arm.launch.py arm:=PSM2 generation:=Classic instrument:=400006
ros2 launch dvrk_model arm.launch.py arm:=PSM2 generation:=Classic instrument:=400049

ros2 launch dvrk_model arm.launch.py arm:=PSM3 generation:=Classic instrument:=400006
ros2 launch dvrk_model arm.launch.py arm:=PSM3 generation:=Classic instrument:=400049
```

### Cross-generation tool compatibility on Classic arms

```bash
ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Classic instrument:=420006
ros2 launch dvrk_model arm.launch.py arm:=PSM2 generation:=Classic instrument:=420006
ros2 launch dvrk_model arm.launch.py arm:=PSM3 generation:=Classic instrument:=420006
```

---

## Single PSM Arms - Si Generation

```bash
ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Si instrument:=420006
ros2 launch dvrk_model arm.launch.py arm:=PSM2 generation:=Si instrument:=420006
ros2 launch dvrk_model arm.launch.py arm:=PSM3 generation:=Si instrument:=420006
```

---

## ECM Arms

```bash
ros2 launch dvrk_model arm.launch.py arm:=ECM generation:=Classic
ros2 launch dvrk_model arm.launch.py arm:=ECM generation:=Si instrument:=SF0826001
```

---

## MTM Arms - Classic Only

```bash
ros2 launch dvrk_model arm.launch.py arm:=MTML generation:=Classic
ros2 launch dvrk_model arm.launch.py arm:=MTMR generation:=Classic
```

---

## Patient Cart - Full System (PSM1, PSM2, PSM3, ECM with SUJ)

```bash
ros2 launch dvrk_model patient_cart.launch.py generation:=Classic
ros2 launch dvrk_model patient_cart.launch.py generation:=Si
```

---

## Surgeon Console - Full System (MTML + MTMR)

```bash
ros2 launch dvrk_model surgeon_console.launch.py
```

---

## Simulated vs Physical Mode Tests

### Simulated Mode (includes dVRK robot node)

```bash
ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Classic simulated:=True
ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Si simulated:=True
```

### Physical Visualization Only

```bash
ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Classic simulated:=False
ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Si simulated:=False
```

---

## Virtual PSM Xacro Checks

Virtual PSM entry points are not exposed through `generation:=Virtual` in ROS 2 launch files.
Validate using direct xacro expansion:

```bash
ros2 run xacro xacro urdf/Virtual/PSM1.urdf.xacro instrument:=400006 > /tmp/virtual_psm1_400006.urdf
ros2 run xacro xacro urdf/Virtual/PSM1.urdf.xacro instrument:=420006 > /tmp/virtual_psm1_420006.urdf
ros2 run xacro xacro urdf/Virtual/PSM2.urdf.xacro instrument:=400049 > /tmp/virtual_psm2_400049.urdf
ros2 run xacro xacro urdf/Virtual/PSM3.urdf.xacro instrument:=420006 > /tmp/virtual_psm3_420006.urdf
```

---

## Instrument TF/Link Validation (One-by-One)

Use the checker below to verify each instrument chain for:
- naming scheme (`PSM1_..._link`),
- expected parent/child stage joints,
- full connectivity from `adaptor_link` to jaw and tool-tip links.

Single instrument:

```bash
python3 scripts/verify_instrument_tf.py --instrument 420006
python3 scripts/verify_instrument_tf.py --instrument 400172
```

All instruments in `instruments.yaml`:

```bash
python3 scripts/verify_instrument_tf.py --all
```

## Instrument DH and Asset Validation

The DH checker compares every instrument in `instruments.yaml` with the
corresponding JSON in the `dvrk_config` ROS package.  It also checks the
instrument Xacro stages and referenced meshes:

```bash
python3 scripts/verify_instrument_dh.py
python3 scripts/verify_instrument_dh.py -v
python3 scripts/verify_instrument_dh.py --instrument 420006
```

When checking an uninstalled source tree, provide the tool directory
explicitly:

```bash
python3 scripts/verify_instrument_dh.py \
  --dvrk-tool-dir /path/to/sawIntuitiveResearchKit/share/tool
```

Missing production tip meshes are reported as warnings when the configured
instrument uses the repository's intentional `tip_placeholder` model.

## Instrument Visual Validation

The visual checker expands the selected PSM Xacro, applies mesh scales and
visual origins, and validates wrist/jaw mesh bounds against a sorted
7 cm x 1 cm x 1 cm envelope. Wrist visual bounds must also contain their link
reference origin:

~~~~bash
python3 scripts/verify_instrument_visual.py --instrument 420006
python3 scripts/verify_instrument_visual.py --all
~~~~
