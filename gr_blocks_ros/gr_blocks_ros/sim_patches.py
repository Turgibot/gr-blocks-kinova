"""Optional tweaks of the generated robot description for the simulation (no vendored file is edited)."""

FINGERS = ("left_inner_finger", "right_inner_finger")  # real (non-lumped) links that carry the pad collisions


def patch_pads(urdf: str, mu: str = "", kp: str = "", kd: str = "") -> str:
    """Add Gazebo friction / contact settings to the finger links (and so to the pads). Empty strings leave a value alone."""
    if not (mu or kp or kd):
        return urdf
    parts = []
    if mu:
        parts.append(f"<mu1>{mu}</mu1><mu2>{mu}</mu2>")
    if kp:
        parts.append(f"<kp>{kp}</kp>")
    if kd:
        parts.append(f"<kd>{kd}</kd>")
    extra = "".join(f'<gazebo reference="{link}">{"".join(parts)}</gazebo>\n' for link in FINGERS)
    i = urdf.rindex("</robot>")
    return urdf[:i] + extra + urdf[i:]


LETTERS = "ABEFHLMORSTW"
ATTACH_PARENT = "bracelet_link"  # real link that carries the gripper body (the gripper base is lumped into it)


def attach_topic(letter: str) -> str:
    return f"/gr_blocks/attach/{letter}"


def detach_topic(letter: str) -> str:
    return f"/gr_blocks/detach/{letter}"


def add_attach_plugins(urdf: str, letters=LETTERS) -> str:
    """One Gazebo DetachableJoint per cube: an empty message on attach_topic welds the cube to the gripper at its
    current relative pose, one on detach_topic releases it. Sim-only 'rigid grasp' used when the physical grasp is
    not good enough (see docs/PHASE2_NOTES.md). The cube models must exist in the world before the robot is spawned."""
    plugins = "".join(
        "<gazebo><plugin filename=\"gz-sim-detachable-joint-system\" name=\"gz::sim::systems::DetachableJoint\">"
        f"<parent_link>{ATTACH_PARENT}</parent_link><child_model>block_{l}</child_model><child_link>link</child_link>"
        f"<attach_topic>{attach_topic(l)}</attach_topic><detach_topic>{detach_topic(l)}</detach_topic>"
        "<suppress_child_warning>true</suppress_child_warning></plugin></gazebo>\n"
        for l in letters
    )
    i = urdf.rindex("</robot>")
    return urdf[:i] + plugins + urdf[i:]


def controllers_with_gain(yaml_path: str, gain: str) -> str:
    """Copy of the controllers file with gz_ros2_control's position_proportional_gain set (velocity = gain * position error,
    its default 0.1 1/s holds loaded joints very softly). Empty gain: the original file is returned unchanged."""
    if not gain:
        return yaml_path
    import tempfile
    from pathlib import Path

    text = Path(yaml_path).read_text() + f"\ngz_ros_control:\n  ros__parameters:\n    position_proportional_gain: {float(gain)}\n"
    out = Path(tempfile.gettempdir()) / f"gr_blocks_ros_controllers_gain_{float(gain)}.yaml"
    out.write_text(text)
    return str(out)
