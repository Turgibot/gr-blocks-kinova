"""Search an arm configuration whose wrist camera sees all 12 cubes (offline: URDF kinematics + pinhole projection).

Camera (Gazebo convention): looks along +x of camera_color_frame, +y = image left, +z = image up.
Intrinsics from the URDF: fx = fy = 554.25, cx = 320.5, cy = 240.5, 640 x 480.
"""
import itertools
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

URDF = sys.argv[1]
FX, CX, CY, W, H = 554.25469, 320.5, 240.5, 640, 480
JOINTS = [f"joint_{i}" for i in range(1, 8)]
HOME = np.array([0, 0.26, 3.14, -2.27, 0, 0.96, 1.57])
CUBE = 0.045
TOWER_X, TOWER_YS, LEVELS = 0.80, (0.12, 0.0, -0.12), 4          # base frame (table top z = 0)

root = ET.parse(URDF).getroot()
J = {j.find("child").get("link"): j for j in root.findall("joint")}
LIM = {}
for j in root.findall("joint"):
    lim = j.find("limit")
    if j.get("name") in JOINTS:
        LIM[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper"))) if j.get("type") == "revolute" and lim is not None else (-2 * np.pi, 2 * np.pi)


def fk(q, link):
    qd = dict(zip(JOINTS, q))
    M = np.eye(4)
    while link in J:
        j = J[link]
        o = j.find("origin")
        A = np.eye(4)
        if o is not None:
            A[:3, :3] = R.from_euler("xyz", [float(v) for v in o.get("rpy", "0 0 0").split()]).as_matrix()
            A[:3, 3] = [float(v) for v in o.get("xyz", "0 0 0").split()]
        if j.get("name") in qd:
            ax = np.array([float(v) for v in j.find("axis").get("xyz").split()])
            B = np.eye(4)
            B[:3, :3] = R.from_rotvec(ax * qd[j.get("name")]).as_matrix()
            A = A @ B
        M = A @ M
        link = j.find("parent").get("link")
    return M


def look_rot(cam, target):
    """Camera rotation: x towards target, y horizontal (image left), z up-ish."""
    x = (target - cam) / np.linalg.norm(target - cam)
    y = np.cross([0, 0, 1.0], x)
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


def ik(cam, Rc, q0):
    def res(q):
        T = fk(q, "camera_color_frame")
        return np.concatenate([10 * (T[:3, 3] - cam), 2 * R.from_matrix(Rc @ T[:3, :3].T).as_rotvec(), 0.02 * (q - HOME)])
    lo = [LIM[j][0] for j in JOINTS]
    hi = [LIM[j][1] for j in JOINTS]
    s = least_squares(res, np.clip(q0, lo, hi), bounds=(lo, hi))
    T = fk(s.x, "camera_color_frame")
    return s.x, np.linalg.norm(T[:3, 3] - cam), np.linalg.norm(R.from_matrix(Rc @ T[:3, :3].T).as_rotvec())


def project(T, pts):
    p = (np.linalg.inv(T) @ np.c_[pts, np.ones(len(pts))].T)[:3].T  # camera frame
    d = p[:, 0]
    return np.c_[CX - FX * p[:, 1] / d, CY - FX * p[:, 2] / d], d


corners = np.array([[TOWER_X + sx * CUBE / 2, y + sy * CUBE / 2, k * CUBE + sz * CUBE / 2 + CUBE / 2]
                    for y in TOWER_YS for k in range(LEVELS) for sx, sy, sz in itertools.product((-1, 1), repeat=3)])
links = [l for l in J if any(s in l for s in ("shoulder", "arm", "forearm", "wrist", "bracelet", "knuckle", "finger", "robotiq"))]

results = []
for cx_, cz_, tz in itertools.product(np.arange(0.20, 0.66, 0.05), np.arange(0.35, 0.86, 0.05), (0.06, 0.09, 0.12)):
    cam, target = np.array([cx_, 0.0, cz_]), np.array([TOWER_X, 0.0, tz])
    Rc = look_rot(cam, target)
    best = None
    for q0 in (HOME, np.array([0, 0.6, 3.14, -1.8, 0, 1.2, 1.57]), np.array([0, -0.3, 3.14, -2.0, 0, 0.6, 1.57])):
        q, ep, eo = ik(cam, Rc, q0)
        if best is None or ep + eo < best[1] + best[2]:
            best = (q, ep, eo)
    q, ep, eo = best
    if ep > 0.003 or eo > 0.01:
        continue
    T = fk(q, "camera_color_frame")
    uv, d = project(T, corners)
    margin = min(uv[:, 0].min(), W - uv[:, 0].max(), uv[:, 1].min(), H - uv[:, 1].max())
    # fingertips: pad boxes (both pads, 8 corners each) must not overlap the cubes' image box
    pads = []
    for side in ("left", "right"):
        P = fk(q, f"{side}_inner_finger_pad")
        pads += [(P @ np.array([sx * 0.015, sy * 0.035, sz * 0.00375, 1]))[:3] for sx, sy, sz in itertools.product((-1, 1), repeat=3)]
    puv, pd = project(T, np.array(pads))
    pin = puv[(pd > 0) & (puv[:, 0] > 0) & (puv[:, 0] < W) & (puv[:, 1] > 0) & (puv[:, 1] < H)]
    cube_box = (uv[:, 0].min(), uv[:, 0].max(), uv[:, 1].min(), uv[:, 1].max())
    pads_over_cubes = bool(len(pin)) and puv[:, 1].min() < cube_box[3] and puv[:, 0].max() > cube_box[0] and puv[:, 0].min() < cube_box[1]
    lowest = min(fk(q, l)[2, 3] for l in links)
    fill = (cube_box[1] - cube_box[0]) * (cube_box[3] - cube_box[2]) / (W * H)
    results.append(dict(cam=cam, tz=tz, q=q, margin=margin, fill=fill, pads_in=len(pin), pads_over=pads_over_cubes,
                        lowest=lowest, dist=d.min(), box=cube_box))

ok = [r for r in results if r["margin"] >= 25 and not r["pads_over"] and r["lowest"] > 0.10 and r["dist"] > 0.25]
print(f"{len(results)} reachable camera poses, {len(ok)} see all cubes (margin >= 25 px, pads not over cubes, arm > 10 cm above table)")
for r in sorted(ok, key=lambda r: -r["fill"])[:8]:
    print(f"cam {np.round(r['cam'], 2)} -> target z {r['tz']:.2f}  fill {r['fill']:.2f}  margin {r['margin']:.0f}px  pads in image {r['pads_in']}/16  "
          f"lowest link z {r['lowest']:.2f}  nearest cube {r['dist']:.2f} m  box u {r['box'][0]:.0f}-{r['box'][1]:.0f} v {r['box'][2]:.0f}-{r['box'][3]:.0f}")
    print("    q =", np.round(r["q"], 3).tolist())
