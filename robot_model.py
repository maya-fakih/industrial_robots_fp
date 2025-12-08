import math
import numpy as np
from typing import List, Dict, Tuple

# ------------------------
# DH + Offset Functions
# ------------------------
def dh_A(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct ],
        [ st,  ct*ca, -ct*sa, a*st ],
        [  0,     sa,     ca,    d ],
        [  0,      0,      0,    1 ]
    ], dtype=float)

def offset_to_matrix(off: Dict) -> np.ndarray:
    if off.get("is_identity", False):
        return np.eye(4, dtype=float)
    tx, ty, tz = off.get("tx", 0.0), off.get("ty", 0.0), off.get("tz", 0.0)
    rx, ry, rz = off.get("rx", 0.0), off.get("ry", 0.0), off.get("rz", 0.0)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]],dtype=float)
    Ry = np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]],dtype=float)
    Rz = np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]],dtype=float)
    R = Rz @ Ry @ Rx
    T = np.eye(4,dtype=float)
    T[0:3,0:3] = R
    T[0:3,3] = [tx,ty,tz]
    return T

# ------------------------
# Robot Model
# ------------------------
class RobotModel:
    def __init__(self):
        self.dh_table = [
            {"theta": None, "alpha": math.pi/2, "a": 0.0,   "d": 0.085},
            {"theta": None, "alpha": 0.0,       "a": 0.105, "d": 0.0},
            {"theta": None, "alpha": 0.0,       "a": 0.10,  "d": 0.0},
            {"theta": None, "alpha": math.pi/2, "a": 0.0,   "d": 0.0},
        ]
        self.n_joints = len(self.dh_table)
        self.offsets = [
            {"is_identity": True, "tx":0,"ty":0,"tz":0},
            {"is_identity": True, "tx":0,"ty":0,"tz":0},
            {"is_identity": True, "tx":0,"ty":0,"tz":0},
            {"is_identity": False, "tx":0,"ty":0,"tz":-0.10}
        ]

    def compute(self, thetas: List[float]) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        if len(thetas) != self.n_joints:
            raise ValueError(f"Expected {self.n_joints} joint angles")
        T_dh_cum = np.eye(4)
        T_dh_frames = [T_dh_cum.copy()]
        T_actual_frames = [T_dh_cum.copy()]
        for i,row in enumerate(self.dh_table):
            theta_rad = math.radians(thetas[i])
            A = dh_A(row["a"], row["d"], row["alpha"], theta_rad)
            T_dh_cum = T_dh_cum @ A
            O = offset_to_matrix(self.offsets[i])
            T_actual = T_dh_cum @ O
            T_dh_frames.append(T_dh_cum.copy())
            T_actual_frames.append(T_actual.copy())
        return T_dh_frames, T_actual_frames

    def fk_dh(self,thetas): return self.compute(thetas)[0]
    def fk_actual(self,thetas): return self.compute(thetas)[1]
