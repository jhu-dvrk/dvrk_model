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
