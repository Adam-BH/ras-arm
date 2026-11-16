"""Overhead camera + OpenCV: find red and blue cubes anywhere on the table (same idea as the ICRA arm's
vision pipeline and the web twin's vision sort, feature/vision-sort).

    frame = camera.capture()                    # synthetic RGB-D from PyBullet (CPU renderer)
    cubes = detect(frame, camera)               # HSV masks → blobs → every pixel unprojected with the depth
                                                # buffer → top face → centre (mm), yaw (deg), size

Each blob pixel goes back to 3-D through the depth buffer (inverse projection × view), so the result is in
robot mm whatever the camera pose. Only the points of the top face are kept (highest 3 mm of the blob):
the side faces a perspective camera sees would otherwise pull the centre toward the image centre.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np
import pybullet as p

from .config import SIM

MM = 0.001

# OpenCV hue is 0…180. The grey table, the orange arm and the purple sensor pad stay out.
HSV = {"RED": [((0, 110, 50), (8, 255, 255)), ((172, 110, 50), (180, 255, 255))],
       "BLUE": [((100, 110, 50), (128, 255, 255))]}
MIN_PX = 40
TOP_BAND_MM = 3.0
SIDE_MM = (18.0, 32.0)          # a 25 mm cube's top face, seen through the renderer
ZONE = {"x": (120.0, 290.0), "y": (-160.0, 160.0)}       # loose cubes are looked for here


@dataclass
class Frame:
    rgb: np.ndarray             # H × W × 3 uint8
    depth: np.ndarray           # H × W float32, OpenGL depth buffer (0 near … 1 far)


@dataclass
class Detection:
    colour: str
    x: float
    y: float
    z_top: float
    yaw: float                  # deg, −45 … 45 (a square repeats every 90°)
    size: tuple[float, float]   # top face, mm
    px: tuple[int, int]         # blob centre in the image
    reason: str = ""            # "" = accepted, else why it was rejected

    @property
    def ok(self) -> bool:
        return not self.reason


class OverheadCamera:
    def __init__(self, cid: int, cfg: dict | None = None):
        c = cfg or SIM.camera
        self.cid, self.w, self.h = cid, int(c["width"]), int(c["height"])
        eye = [c["x"] * MM, c["y"] * MM, c["z"] * MM]
        self.eye = eye
        # image up = robot +x, so image right = robot −y
        self.view = p.computeViewMatrix(eye, [eye[0], eye[1], 0.0], [1, 0, 0], physicsClientId=cid)
        self.proj = p.computeProjectionMatrixFOV(c["fov_deg"], self.w / self.h, 0.2, 0.8, physicsClientId=cid)
        V = np.array(self.view).reshape(4, 4, order="F")       # PyBullet matrices are column-major
        P = np.array(self.proj).reshape(4, 4, order="F")
        self.inv = np.linalg.inv(P @ V)
        self.fov = c["fov_deg"]

    def capture(self) -> Frame:
        img = p.getCameraImage(self.w, self.h, self.view, self.proj, renderer=p.ER_TINY_RENDERER,
                               lightDirection=[0.4, 0.3, 1.0], shadow=0, flags=p.ER_NO_SEGMENTATION_MASK,
                               physicsClientId=self.cid)
        rgb = np.reshape(np.asarray(img[2], dtype=np.uint8), (self.h, self.w, 4))[:, :, :3].copy()
        depth = np.reshape(np.asarray(img[3], dtype=np.float32), (self.h, self.w))
        return Frame(rgb, depth)

    def unproject(self, u: np.ndarray, v: np.ndarray, d: np.ndarray) -> np.ndarray:
        """pixel columns u, rows v, depth-buffer values d → N × 3 world points (mm)"""
        ndc = np.stack([(2 * u + 1) / self.w - 1, 1 - (2 * v + 1) / self.h, 2 * d - 1, np.ones_like(d)])
        w = self.inv @ ndc
        return (w[:3] / w[3]).T / MM

    def project(self, x: float, y: float, z: float) -> tuple[float, float]:
        """world mm → pixel (u, v), for drawing"""
        V = np.array(self.view).reshape(4, 4, order="F")
        P = np.array(self.proj).reshape(4, 4, order="F")
        c = P @ V @ np.array([x * MM, y * MM, z * MM, 1.0])
        c = c[:3] / c[3]
        return (c[0] + 1) * self.w / 2 - 0.5, (1 - c[1]) * self.h / 2 - 0.5


def masks(frame: Frame) -> dict[str, np.ndarray]:
    hsv = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2HSV)
    out = {}
    for colour, ranges in HSV.items():
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        out[colour] = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return out


def detect(frame: Frame, cam: OverheadCamera, zone: dict = ZONE) -> list[Detection]:
    found: list[Detection] = []
    for colour, m in masks(frame).items():
        n, labels, stats, cents = cv2.connectedComponentsWithStats(m, connectivity=4)
        for k in range(1, n):
            if stats[k, cv2.CC_STAT_AREA] < MIN_PX:
                continue
            v, u = np.nonzero(labels == k)
            pts = cam.unproject(u.astype(np.float64), v.astype(np.float64), frame.depth[v, u].astype(np.float64))
            z_top = float(np.percentile(pts[:, 2], 95))
            top = pts[pts[:, 2] > z_top - TOP_BAND_MM]
            cx, cy = (float(c) for c in top[:, :2].mean(axis=0))
            (_, _), (sw, sh), ang = cv2.minAreaRect(top[:, :2].astype(np.float32))
            yaw = (ang + 45) % 90 - 45                         # a square: yaw only matters mod 90°
            side_lo, side_hi = sorted((sw, sh))
            reason = ""
            if side_lo < 0.5 or side_hi / side_lo > 1.5:
                reason = "shape"
            elif side_lo < SIDE_MM[0]:
                reason = "small"
            elif side_hi > SIDE_MM[1]:
                reason = "large"                               # e.g. two cubes touching
            elif not (zone["x"][0] <= cx <= zone["x"][1] and zone["y"][0] <= cy <= zone["y"][1]):
                reason = "zone"
            found.append(Detection(colour, cx, cy, z_top, float(yaw), (float(sw), float(sh)),
                                   (int(cents[k][0]), int(cents[k][1])), reason))
    return found


def annotate(frame: Frame, cam: OverheadCamera, dets: list[Detection], title: str = "") -> np.ndarray:
    """BGR debug image: the frame with every blob, its centre, yaw and verdict"""
    img = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
    zx, zy = ZONE["x"], ZONE["y"]
    corners = [cam.project(x, y, 0) for x, y in ((zx[0], zy[0]), (zx[1], zy[0]), (zx[1], zy[1]), (zx[0], zy[1]))]
    cv2.polylines(img, [np.array(corners, np.int32)], True, (90, 200, 90), 1)
    for d in dets:
        colour = (0, 230, 0) if d.ok else (0, 160, 255)
        u, v = cam.project(d.x, d.y, d.z_top)
        r = 16
        dx, dy = r * math.cos(math.radians(d.yaw)), r * math.sin(math.radians(d.yaw))
        u2, v2 = cam.project(d.x + dx, d.y + dy, d.z_top)
        cv2.circle(img, (round(u), round(v)), 3, colour, -1)
        cv2.line(img, (round(u), round(v)), (round(u2), round(v2)), colour, 1)
        label = f"{d.colour[0]} {d.x:.0f},{d.y:.0f} {d.yaw:+.0f}" + (f" [{d.reason}]" if d.reason else "")
        cv2.putText(img, label, (round(u) + 6, round(v) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, colour, 1, cv2.LINE_AA)
    if title:
        cv2.putText(img, title, (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (240, 240, 240), 1, cv2.LINE_AA)
    return img


def mask_view(frame: Frame) -> np.ndarray:
    """BGR: the dimmed frame with the red / blue classes painted in"""
    img = (cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR) * 0.35).astype(np.uint8)
    m = masks(frame)
    img[m["RED"] > 0] = (40, 40, 230)
    img[m["BLUE"] > 0] = (230, 120, 40)
    return img
