# OneRobotics A1 for Genesis World

Standalone OneRobotics A1 integration for [Genesis World](https://github.com/Genesis-Embodied-AI/genesis-world), following an external-project layout similar to [onerobotics-a1-mjlab](https://github.com/T1Amoo/onerobotics-a1-mjlab).

This project is independently maintained; it is **not** part of the Genesis curated asset library and is not endorsed by the Genesis maintainers.

## Supported models

| Model | DoF | Source asset |
| --- | ---: | --- |
| Right arm | 7 | `a1_r.urdf` and 8 STL meshes |
| Left arm | 7 | `a1_l.urdf` and 8 STL meshes |
| Bimanual stand | 14 | `a1_bimanual_stand.urdf` and 17 STL meshes |

All three models, 33 referenced STL files, and their attribution notice are bundled under `src/onerobotics_a1_genesis/assets/onerobotics_a1/`. No grippers are included.

## Installation

Requires Python 3.10–3.13. Install a compatible version of [PyTorch](https://pytorch.org/get-started/locally/) for your system before installing Genesis and this integration. The following works in a Conda environment; `uv` is not required.

```bash
git clone https://github.com/Chaysse/onerobotics-a1-genesis.git
cd onerobotics-a1-genesis
python -m pip install -e .
```

This installs `genesis-world` as a dependency. For a preconfigured Genesis environment, `python -m pip install -e . --no-deps` avoids reinstalling it.

## Quick start

Check the three sets of URDF/mesh files (this is **not** a simulation test):

```bash
a1-genesis-demo --robot right --check-assets
a1-genesis-demo --robot left --check-assets
a1-genesis-demo --robot bimanual --check-assets
```

View the models in Genesis:

```bash
a1-genesis-demo --robot right --demo view -v
a1-genesis-demo --robot left --demo view -v
a1-genesis-demo --robot bimanual --demo view -v
```

Run joint control and inverse kinematics:

```bash
a1-genesis-demo --robot right --demo control -v
a1-genesis-demo --robot right --demo ik -v
a1-genesis-demo --robot left --demo control -v
a1-genesis-demo --robot left --demo ik -v
a1-genesis-demo --robot bimanual --demo control -v
a1-genesis-demo --robot bimanual --demo ik --arm both -v
```

Equivalent entry point from the repository root (after installing the package):

```bash
python examples/a1_genesis_demo.py --robot right --demo view -v
```

Without `-v`, the simulation runs without the graphical viewer. See `a1-genesis-demo --help` for more options. The bimanual model supports `--no-plane` and `--robot-z` for ground placement diagnostics.

## Use the model in your own Genesis code

```python
import genesis as gs
from onerobotics_a1_genesis import urdf_path

gs.init(backend=gs.cpu)
scene = gs.Scene(show_viewer=False)
robot = scene.add_entity(
    gs.morphs.URDF(file=str(urdf_path("right")), fixed=True),
)
scene.build()
```

`urdf_path` also accepts `"left"` and `"bimanual"`. This interface returns a filesystem path to packaged data without modifying Genesis.

## Validation and limitations

See [VALIDATION.md](VALIDATION.md) for the existing fork's reported Genesis version, model-loading results, joint-control observations, and IK results. The independent-package relocation should be rechecked locally before publishing a compatibility claim for the packaged distribution.

The independent arms retain upstream temporary Link7 inertial values; the bimanual URDF retains zero effort/velocity placeholders. The demonstration overrides runtime motor force limits for control only. These demonstrations are not proof of hardware fidelity or a production-ready controller.

## Licensing and provenance

The example code is distributed under the root [Apache-2.0 LICENSE](LICENSE). The OneRobotics URDF/STL assets are licensed separately under **CC BY 4.0**; see [ASSET_LICENSE.md](ASSET_LICENSE.md) and [ATTRIBUTION.md](src/onerobotics_a1_genesis/assets/onerobotics_a1/ATTRIBUTION.md).

Upstream asset source: https://github.com/katazen/onerobot_h1

Pinned source revision: `3db6f614c8443bd9396fcd9304cf64a519f98f2a`

Earlier Genesis upstream asset submission (declined due to the repository asset-library policy): [Issue #3523](https://github.com/Genesis-Embodied-AI/genesis-world/issues/3523) / [PR #3525](https://github.com/Genesis-Embodied-AI/genesis-world/pull/3525).
