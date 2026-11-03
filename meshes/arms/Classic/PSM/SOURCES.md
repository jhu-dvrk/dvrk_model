# Classic PSM CAD sources

Created on: 2026-11-03

The STL files listed below were copied without geometry changes from
[zixingjiang/dvrk_description](https://github.com/zixingjiang/dvrk_description),
commit `2a29763608fd619082dec112f07014256cc62c22`.
Upstream credits JHU dVRK's `dvrk_model` and WPI-AIM's `dvrk_env` for resources.
The upstream copyright and MIT license are reproduced below.

| Local file | Upstream file |
| --- | --- |
| `PSM_base.stl` | `meshes/classic/psm/base.stl` |
| `PSM_yaw.stl` | `meshes/classic/psm/roll.stl` |
| `PSM_pitch_2.stl` | `meshes/classic/psm/pitch_parallel_back.stl` |
| `PSM_pitch_3.stl` | `meshes/classic/psm/pitch_parallel_front.stl` |
| `PSM_pitch_4.stl` | `meshes/classic/psm/pitch_parallel_bottom.stl` |
| `PSM_pitch_5.stl` | `meshes/classic/psm/pitch_parallel_top.stl` |
| `PSM_insertion.stl` | `meshes/classic/psm/pitch_parallel_end.stl` |
| `PSM_adapter.stl` | `meshes/classic/psm/insertion.stl` |

`PSM_insertion.stl` contains the fixed end carrier and cannula. It follows
arm yaw and pitch. `PSM_adapter.stl` follows the insertion carriage.
The original instrument housing, shaft, wrist and tip meshes remain separate.

`urdf/Classic/PSM_base.urdf.xacro` aligns the upstream CAD frames to the
existing dVRK RCM frame (90 degrees about Z). The passive linkage frames use
the upstream geometry; the actuated joint transforms, limits, and EE frames
are retained from dvrk_model. The old DAE assets remain available for legacy
models.

## Upstream license

MIT License

Copyright (c) 2026 Zixing Jiang

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

