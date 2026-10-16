# OneRobotics A1 for Genesis World

OneRobotics A1 robot integration for [Genesis World](https://github.com/Genesis-Embodied-AI/genesis-world), providing URDF robot models, simulation examples, joint control, and inverse kinematics.

The project supports three robot configurations: a 7-DoF right arm, a 7-DoF left arm, and a 14-DoF bimanual system with a fixed stand.

## Supported models

| Model | DoF | Source asset |
| --- | ---: | --- |
| Right arm | 7 | `a1_r.urdf` and 8 STL meshes |
| Left arm | 7 | `a1_l.urdf` and 8 STL meshes |
| Bimanual stand | 14 | `a1_bimanual_stand.urdf` and 17 STL meshes |

All three models, 33 referenced STL files, and their attribution notice are bundled under `src/onerobotics_a1_genesis/assets/onerobotics_a1/`. No grippers are included.

## Installation

Requires Python 3.10–3.13. Install a compatible version of [PyTorch](https://pytorch.org/get-started/locally/) for your system before installing Genesis and this integration. The following works in a Conda environment; 

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

The integration supports both CPU and GPU backends in Genesis World.

### GPU example

```python
import genesis as gs
from onerobotics_a1_genesis import urdf_path

gs.init(backend=gs.gpu, precision="32")

scene = gs.Scene(show_viewer=False)

robot = scene.add_entity(
    gs.morphs.URDF(
        file=str(urdf_path("right")),
        fixed=True,
    )
)

scene.build()

for _ in range(1000):
    scene.step()
```

### CPU backend

To run on CPU, replace:

```python
gs.init(backend=gs.gpu, precision="32")
```

with:

```python
gs.init(backend=gs.cpu, precision="32")
```

### Available robot models

- `urdf_path("right")`: A1 right arm (7 DoF)
- `urdf_path("left")`: A1 left arm (7 DoF)
- `urdf_path("bimanual")`: A1 bimanual stand (14 DoF)

The `urdf_path()` function returns the filesystem path
to the packaged URDF model without modifying Genesis.

GPU execution requires a compatible GPU, drivers, and
Genesis runtime dependencies.
```

## Validation and limitations

See [VALIDATION.md](VALIDATION.md) for recorded local Genesis validation results, including URDF loading, joint control,and inverse kinematics.The standalone Python package should be validated after installation before claiming compatibility for the packaged distribution. The independent-package relocation should be rechecked locally before publishing a compatibility claim for the packaged distribution.

The independent arms retain upstream temporary Link7 inertial values; the bimanual URDF retains zero effort/velocity placeholders. The demonstration overrides runtime motor force limits for control only. These demonstrations are not proof of hardware fidelity or a production-ready controller.

## Licensing

The Python integration and example code are licensed under [Apache License 2.0](LICENSE).
The OneRobotics A1 URDF models and STL meshes are licensed separately under **Creative Commons Attribution 4.0 International(CC BY 4.0)**.See [ASSET_LICENSE.md](ASSET_LICENSE.md) and[ATTRIBUTION.md](src/onerobotics_a1_genesis/assets/onerobotics_a1/ATTRIBUTION.md)for asset licensing and attribution details.

Original asset repository: https://github.com/katazen/onerobot_h1
