#!/usr/bin/env python3
"""Load and demonstrate OneRobotics A1 robots in Genesis.

This is a self-contained example; no changes to the Franka examples are needed.
The original A1 URDF and STL files are read, never modified.
Control constants are copied from the upstream model_parameters.yaml; that
metadata YAML is not bundled with this minimal asset contribution.

Example entry point in the independent repository:
    examples/a1_genesis_demo.py

The assets should be at:
    src/onerobotics_a1_genesis/assets/onerobotics_a1/

Examples (from the independent repository root, after installation):
    python examples/a1_genesis_demo.py --robot right --demo view -v
    python examples/a1_genesis_demo.py --robot left --demo control -v
    python examples/a1_genesis_demo.py --robot bimanual --demo ik --arm both -v
    python examples/a1_genesis_demo.py --robot right --demo all -v
    python examples/a1_genesis_demo.py --robot bimanual --check-assets
    python examples/a1_genesis_demo.py --robot bimanual --demo view -v
    python examples/a1_genesis_demo.py --robot bimanual --demo view -v --no-plane
    python examples/a1_genesis_demo.py --robot bimanual --demo view -v --robot-z 0.3

The ``view`` and ``ik`` demos use kinematic visualization. The ``control``
demo uses Genesis's physical PD/velocity/force controls. In particular, the
bimanual source URDF has effort="0" placeholders; this script overrides the
imported actuator force limits at runtime using the *peak* joint torques in
the upstream OneRobotics model_parameters.yaml. This does not fix or edit
the URDF itself.

This example is not evidence of model fidelity or a validated controller.
Run it on a recorded Genesis version before reporting
that loading, control, or inverse kinematics have passed.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np


MODEL_FILES = {
    "right": Path("a1_r.urdf"),
    "left": Path("a1_l.urdf"),
    "bimanual": Path("bimanual_stand/a1_bimanual_stand.urdf"),
}

# These values are from upstream OneRobotics model_parameters.yaml; this file
# does not load or require that YAML at runtime.
# Each arm uses the same per-joint settings; the limits are joint-output peaks
# rather than continuous torque guarantees. No thermal/peak-duration model is
# implied by setting a static simulation force range.
ARM_KP = np.array([150.0, 150.0, 150.0, 150.0, 40.0, 40.0, 40.0])
ARM_KV = np.array([4.0, 4.0, 4.0, 4.0, 1.0, 1.0, 1.0])
ARM_PEAK_TORQUE = np.array([26.859, 26.859, 26.859, 5.975, 5.975, 5.975, 5.975])
SIM_DT = 0.005  # 200 Hz, consistent with the supplied model_parameters.yaml.


def find_assets_dir(assets_dir: Path | None, robot_kind: str) -> Path:
    """Locate A1 assets when the example is run from root or examples/rigid."""
    rel = Path("src/onerobotics_a1_genesis/assets/onerobotics_a1")
    if assets_dir is not None:
        candidates = [assets_dir.expanduser()]
    else:
        script = Path(__file__).resolve()
        candidates = [root / rel for root in (Path.cwd(), script.parent, *script.parents)]
        candidates.append(script.parent / "assets" / "onerobotics_a1")

    # A user may keep only one of the three configurations in a local checkout.
    for candidate in dict.fromkeys(p.resolve() for p in candidates):
        if (candidate / MODEL_FILES[robot_kind]).is_file():
            return candidate

    checked = "\n".join(f"  {p.resolve()}" for p in candidates)
    raise FileNotFoundError(
        f"Cannot find the {robot_kind} A1 URDF. Checked these asset directories:\n"
        f"{checked}\nUse --assets-dir PATH if the assets are stored elsewhere."
    )


def check_assets(model_path: Path) -> int:
    """Check the URDF is readable and its locally referenced meshes exist."""
    root = ET.parse(model_path).getroot()
    if root.tag != "robot":
        raise ValueError(f"Not a URDF <robot> document: {model_path}")

    meshes = [element.attrib["filename"] for element in root.iter("mesh")]
    if not meshes:
        raise ValueError(f"No mesh references found in {model_path}")

    missing = []
    for mesh in set(meshes):
        # The supplied A1 models intentionally use URDF-relative paths.
        if "://" in mesh or mesh.startswith("package:"):
            raise ValueError(f"Non-local A1 mesh path requires special resolution: {mesh}")
        if not (model_path.parent / mesh).is_file():
            missing.append(mesh)
    if missing:
        raise FileNotFoundError("Missing referenced A1 meshes:\n  " + "\n  ".join(sorted(missing)))

    joints = [joint for joint in root.findall("joint") if joint.get("type") in ("revolute", "continuous")]
    print(f"[ASSETS] {model_path}: {len(joints)} movable joints, {len(set(meshes))} mesh files present.")
    return len(joints)


def auto_bimanual_ground_z(urdf_path: Path, clearance: float = 0.005) -> float:
    """Place the bimanual stand's base_link visual mesh just above z=0.

    The URDF origin is not necessarily the physical bottom of the stand.
    Only the stand base mesh is used: using the lowest point of *all* arm
    links would make the stand float if a zero-pose arm extends downward.
    This adjusts the scene pose, never the original URDF or STL data.
    """
    try:
        import trimesh
    except ImportError as exc:
        raise RuntimeError("Automatic ground alignment needs the trimesh package") from exc

    root = ET.parse(urdf_path).getroot()
    base = root.find("./link[@name='base_link']")
    if base is None:
        raise ValueError(f"Missing base_link in {urdf_path}")
    visual = base.find("visual")
    if visual is None:
        raise ValueError(f"Missing base_link visual geometry in {urdf_path}")
    mesh_tag = visual.find("geometry/mesh")
    if mesh_tag is None:
        raise ValueError(f"Missing base_link visual mesh in {urdf_path}")
    mesh_path = (urdf_path.parent / mesh_tag.attrib["filename"]).resolve()
    mesh = trimesh.load_mesh(str(mesh_path), process=False)
    if not hasattr(mesh, "vertices") or len(mesh.vertices) == 0:
        raise ValueError(f"Empty or invalid base mesh: {mesh_path}")

    scale = np.fromstring(mesh_tag.get("scale", "1 1 1"), sep=" ")
    if scale.size != 3:
        raise ValueError(f"Invalid mesh scale for {mesh_path}: {scale}")
    origin_tag = visual.find("origin")
    origin_xyz = np.zeros(3)
    origin_rpy = np.zeros(3)
    if origin_tag is not None:
        origin_xyz = np.fromstring(origin_tag.get("xyz", "0 0 0"), sep=" ")
        origin_rpy = np.fromstring(origin_tag.get("rpy", "0 0 0"), sep=" ")
    if origin_xyz.size != 3 or origin_rpy.size != 3:
        raise ValueError(f"Invalid base_link visual origin in {urdf_path}")

    # URDF rpy is a fixed-axis XYZ rotation; this follows the usual URDF
    # origin transform, using trimesh's 'sxyz' convention.
    transform = trimesh.transformations.euler_matrix(*origin_rpy, axes="sxyz")
    vertices = np.asarray(mesh.vertices, dtype=float) * scale
    z_values = vertices @ transform[2, :3] + origin_xyz[2]
    lowest_z = float(z_values.min())
    raise_by = clearance - lowest_z
    print(
        f"[GROUND] Stand base mesh lowest local Z={lowest_z:.4f} m; "
        f"placing robot base at world Z={raise_by:.4f} m "
        f"(clearance={clearance:.3f} m)."
    )
    return raise_by


def arm_names(robot_kind: str, arm: str) -> list[str]:
    if robot_kind == "bimanual":
        prefix = "r" if arm == "right" else "l"
        return [f"joint_{prefix}{i}" for i in range(1, 8)]
    suffix = "a1_r" if robot_kind == "right" else "a1_l"
    return [f"joint{i}-{suffix}" for i in range(1, 8)]


def arms_in(robot_kind: str) -> tuple[str, ...]:
    return ("right", "left") if robot_kind == "bimanual" else (robot_kind,)


def joint_groups(robot, robot_kind: str) -> dict[str, list[int]]:
    groups = {}
    for arm in arms_in(robot_kind):
        indices = []
        for joint_name in arm_names(robot_kind, arm):
            try:
                joint = robot.get_joint(joint_name)
            except Exception as exc:
                actual = [joint.name for joint in robot.joints]
                raise RuntimeError(f"Missing joint {joint_name!r}; imported joints: {actual}") from exc
            if len(joint.dofs_idx_local) != 1:
                raise RuntimeError(f"Expected exactly 1 DoF for {joint_name!r}")
            indices.append(int(joint.dofs_idx_local[0]))
        groups[arm] = indices

    selected = [idx for indices in groups.values() for idx in indices]
    expected = 14 if robot_kind == "bimanual" else 7
    if robot.n_dofs != expected or len(selected) != expected or len(set(selected)) != expected:
        raise RuntimeError(f"Expected {expected} independent DoFs; got {robot.n_dofs}, mapping={groups}")
    return groups


def as_numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def apply_actuator_metadata(robot, groups: dict[str, list[int]]) -> None:
    """Apply the supplied PD gains and documented peak torque caps in Genesis."""
    for arm, dofs in groups.items():
        robot.set_dofs_kp(ARM_KP, dofs_idx_local=dofs)
        robot.set_dofs_kv(ARM_KV, dofs_idx_local=dofs)
        # Includes a runtime workaround for bimanual effort="0" URDF limits.
        robot.set_dofs_force_range(-ARM_PEAK_TORQUE, ARM_PEAK_TORQUE, dofs_idx_local=dofs)
        print(f"[CONTROL] {arm}: kp/kv and peak torque caps set on DoFs {dofs}.")


def control_targets(robot, groups: dict[str, list[int]]) -> list[tuple[str, np.ndarray]]:
    home = np.zeros(robot.n_dofs, dtype=float)
    pose_a = home.copy()
    pose_b = home.copy()
    for arm, idx in groups.items():
        sign = 1.0 if arm == "right" else -1.0
        # Joint 2 is bounded on opposite sides for left/right arms.
        pose_a[idx[0]] = sign * 0.24
        pose_a[idx[1]] = -sign * 0.12
        pose_b[idx[0]] = sign * 0.10
        pose_b[idx[1]] = -sign * 0.07
        pose_b[idx[2]] = 0.12

    # Never command outside the joint position limits parsed by Genesis.
    for label, pose in (("home", home), ("pose A", pose_a), ("pose B", pose_b)):
        for arm, idx in groups.items():
            low, high = robot.get_dofs_limit(dofs_idx_local=idx)
            q = pose[idx]
            lower, upper = as_numpy(low).reshape(-1), as_numpy(high).reshape(-1)
            if np.any(q < lower) or np.any(q > upper):
                raise ValueError(f"{label} is outside {arm} joint limits: {q}; [{lower}, {upper}]")
    return [("pose A", pose_a), ("pose B", pose_b), ("home", home)]


def run_view(scene, robot, steps: int) -> None:
    print("\n[VIEW] Displaying OneRobotics A1 (fixed base)...")
    robot.set_qpos(np.zeros(robot.n_dofs), zero_velocity=True)
    for _ in range(steps):
        scene.step()
    print("[VIEW] Scene completed.")


def joint_report(robot, label: str, target: np.ndarray, indices: list[int]) -> None:
    measured = as_numpy(robot.get_dofs_position(dofs_idx_local=indices)).reshape(-1)
    err = measured - target[indices]
    rmse = float(np.sqrt(np.mean(err**2)))
    print(f"  {label}: target={np.round(target[indices], 3)}, "
          f"actual={np.round(measured, 3)}, RMSE={rmse:.5f} rad")


def run_control(scene, robot, groups: dict[str, list[int]], steps: int) -> None:
    """Position, velocity, and torque demos inspired by control_franka.py."""
    print("\n[CONTROL] Genesis physical joint control")
    apply_actuator_metadata(robot, groups)
    selected = [idx for ids in groups.values() for idx in ids]
    home = np.zeros(robot.n_dofs)
    targets = control_targets(robot, groups)
    robot.set_qpos(home, zero_velocity=True)

    for label, target in targets:
        robot.control_dofs_position(target[selected], dofs_idx_local=selected)
        for _ in range(steps):
            scene.step()
        joint_report(robot, label, target, selected)

    # Velocity and torque examples use only joint 1 of each active arm.
    first_joint = [ids[0] for ids in groups.values()]
    vel = np.array([0.10 if arm == "right" else -0.10 for arm in groups])
    robot.control_dofs_position(home[selected], dofs_idx_local=selected)
    robot.control_dofs_velocity(vel, dofs_idx_local=first_joint)
    for _ in range(min(steps, 80)):
        scene.step()
    observed = as_numpy(robot.get_dofs_velocity(dofs_idx_local=first_joint)).reshape(-1)
    print(f"[CONTROL] Velocity target={vel}, measured={np.round(observed, 4)} rad/s")

    # This is only a short API demo, not a calibrated torque-control test.
    small_torque = np.array([0.5 if arm == "right" else -0.5 for arm in groups])
    robot.control_dofs_force(small_torque, dofs_idx_local=first_joint)
    for _ in range(min(steps, 20)):
        scene.step()
    measured_torque = as_numpy(robot.get_dofs_control_force(dofs_idx_local=first_joint)).reshape(-1)
    print(f"[CONTROL] Torque target={small_torque}, control output={np.round(measured_torque, 4)} Nm")

    robot.control_dofs_position(home[selected], dofs_idx_local=selected)
    for _ in range(steps):
        scene.step()
    joint_report(robot, "final home", home, selected)
    print("[CONTROL] Done. Inspect reported errors; commands are not proof of tracking accuracy.")


def run_ik(scene, robot, robot_kind: str, groups: dict[str, list[int]],
           arm: str, steps: int, marker, show_viewer: bool) -> None:
    """A1-specific FK-derived IK demonstration inspired by ik_franka.py.

    ``set_qpos`` is for visualizing solved configurations; this is not a
    physics-based Cartesian controller. In bimanual mode only the target arm
    DoFs are passed to the IK solver.
    """
    requested = arms_in(robot_kind) if arm == "both" else (arm,)
    if robot_kind != "bimanual":
        requested = arms_in(robot_kind)
    home = np.zeros(robot.n_dofs)
    print("\n[IK] Computing Genesis IK against reachable, A1-derived targets")
    for active_arm in requested:
        idx = groups[active_arm]
        if robot_kind == "bimanual":
            ee_name = "Link_r7" if active_arm == "right" else "Link_l7"
        else:
            ee_name = "Link7"
        try:
            ee_link = robot.get_link(ee_name)
        except Exception as exc:
            raise RuntimeError(f"Cannot locate end-effector link {ee_name!r}") from exc
        print(f"[IK] arm={active_arm}, end-effector={ee_name}, DoFs={idx}")

        robot.set_qpos(home, zero_velocity=True)
        previous = home.copy()
        largest_error = 0.0
        for i in range(steps):
            phase = 2.0 * np.pi * i / steps
            ref = home.copy()
            sign = 1.0 if active_arm == "right" else -1.0
            ref[idx[0]] = sign * 0.16 * np.sin(phase)
            ref[idx[1]] = -sign * 0.10 * (1.0 - np.cos(phase))
            ref[idx[2]] = 0.08 * np.sin(phase)

            # First sample a reachable pose with this same Genesis model.
            robot.set_qpos(ref, zero_velocity=True)
            goal_pos = as_numpy(ee_link.get_pos()).reshape(-1).copy()
            goal_quat = as_numpy(ee_link.get_quat()).reshape(-1).copy()
            if marker is not None:
                marker.set_qpos(np.concatenate((goal_pos, goal_quat)))

            # Solve IK from the previous configuration, controlling one arm.
            robot.set_qpos(previous, zero_velocity=True)
            solution, _ = robot.inverse_kinematics(
                link=ee_link,
                pos=goal_pos,
                quat=goal_quat,
                init_qpos=previous,
                dofs_idx_local=idx,
                respect_joint_limit=True,
                return_error=True,
            )
            robot.set_qpos(solution, zero_velocity=True)
            previous = as_numpy(robot.get_qpos()).reshape(-1).copy()
            reached = as_numpy(ee_link.get_pos()).reshape(-1)
            error = float(np.linalg.norm(reached - goal_pos))
            largest_error = max(largest_error, error)
            if show_viewer:
                scene.visualizer.update(force=True)
            if i % max(steps // 5, 1) == 0 or i == steps - 1:
                print(f"  IK step={i:4d}, position error={error:.6f} m")
        print(f"[IK] {active_arm}: maximum observed position error={largest_error:.6f} m")
    print("[IK] Done (kinematic visualization, not dynamic trajectory tracking).")


def main() -> int:
    parser = argparse.ArgumentParser(description="OneRobotics A1 Genesis example: view, control and IK")
    parser.add_argument("--robot", choices=tuple(MODEL_FILES), default="right")
    parser.add_argument("--demo", choices=("view", "control", "ik", "all"), default="view")
    parser.add_argument("--arm", choices=("right", "left", "both"), default="both",
                        help="Which arm to use for bimanual IK; defaults to both")
    parser.add_argument("--assets-dir", type=Path, help="Override A1 asset directory")
    parser.add_argument("--steps", type=int, default=300, help="Frames per demo stage; default 300")
    parser.add_argument("--gpu", action="store_true", help="Run Genesis with the GPU backend")
    parser.add_argument("-v", "--vis", action="store_true", help="Show GUI viewer (official examples style)")
    parser.add_argument("--headless", action="store_true", help="Explicitly disable GUI viewer")
    parser.add_argument(
        "--robot-z", type=float, default=None,
        help="Override robot base height in meters; for bimanual, default is automatic stand-floor alignment",
    )
    parser.add_argument(
        "--no-plane", action="store_true",
        help="Do not add the ground plane (useful to inspect model geometry without occlusion)",
    )
    parser.add_argument("--check-assets", action="store_true",
                        help="Check selected URDF mesh references without starting Genesis")
    args = parser.parse_args()

    if args.steps < 1:
        parser.error("--steps must be at least 1")
    if args.vis and args.headless:
        parser.error("--vis and --headless cannot be used together")
    if args.robot != "bimanual" and args.arm not in ("both", args.robot):
        parser.error("--arm must match --robot unless --robot bimanual")

    try:
        assets = find_assets_dir(args.assets_dir, args.robot)
        urdf_path = (assets / MODEL_FILES[args.robot]).resolve()
        expected = 14 if args.robot == "bimanual" else 7
        dofs = check_assets(urdf_path)
        if dofs != expected:
            raise ValueError(f"Expected {expected} revolute joints, found {dofs} in {urdf_path}")
    except (OSError, ET.ParseError, ValueError) as exc:
        parser.error(str(exc))

    if args.check_assets:
        print("[ASSETS] File presence checks passed; Genesis import has NOT been tested.")
        return 0

    try:
        import genesis as gs
    except ImportError as exc:
        print("Genesis is unavailable. Install the checkout with `python -m pip install -e .`.",
              file=sys.stderr)
        raise SystemExit(2) from exc

    gs.init(backend=gs.gpu if args.gpu else gs.cpu, precision="32")
    viewer = args.vis and not args.headless
    print(f"[A1] Robot={args.robot}, demo={args.demo}, URDF={urdf_path}")
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=SIM_DT),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(2.4, -2.4, 1.8),
            camera_lookat=(0.0, 0.0, 0.6),
            camera_fov=40,
        ),
        show_viewer=viewer,
    )
    if not args.no_plane:
        scene.add_entity(gs.morphs.Plane())

    # The bimanual URDF's fixed base_link origin may not be located at the
    # support surface of the stand. Align the stand's own mesh to the z=0
    # ground rather than assuming its mesh begins at the URDF origin.
    if args.robot_z is not None:
        robot_z = args.robot_z
        print(f"[GROUND] Using explicit robot base Z={robot_z:.4f} m.")
    elif args.robot == "bimanual":
        robot_z = auto_bimanual_ground_z(urdf_path)
    else:
        robot_z = 0.0

    # Use a rigid dynamic model only when testing actual joint control.
    material = (gs.materials.Rigid() if args.demo in ("control", "all")
                else gs.materials.Kinematic())
    robot = scene.add_entity(
        morph=gs.morphs.URDF(file=str(urdf_path), fixed=True, pos=(0.0, 0.0, robot_z)),
        material=material,
    )
    marker = None
    if args.demo in ("ik", "all"):
        marker = scene.add_entity(
            gs.morphs.Mesh(file="meshes/axis.obj", scale=0.10),
            material=gs.materials.Kinematic(),
        )

    scene.build()
    groups = joint_groups(robot, args.robot)
    print(f"[A1] Genesis loaded {robot.n_dofs} DoFs: {groups}")

    if args.demo == "all":
        # Prevent gravity-induced dropping during the view part of a dynamic demo.
        apply_actuator_metadata(robot, groups)
        all_indices = [i for dofs in groups.values() for i in dofs]
        robot.control_dofs_position(np.zeros(len(all_indices)), dofs_idx_local=all_indices)

    if args.demo in ("view", "all"):
        run_view(scene, robot, args.steps)
    if args.demo in ("control", "all"):
        run_control(scene, robot, groups, args.steps)
    if args.demo in ("ik", "all"):
        run_ik(scene, robot, args.robot, groups, args.arm, args.steps, marker, viewer)

    print("\n[A1] Demo finished. Verify results before claiming Genesis compatibility.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
