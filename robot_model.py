import numpy as np

class RobotModel:
    """
    RobotModel holds:
    - The DH table (static)
    - The physical servo offsets (height offsets, link offsets)
    - Functions A() and T()
    - Returns transformed frames including the physical offsets
    """

    def __init__(self):
        # -------------------------
        # 1) DH TABLE (as you provided)
        # -------------------------
        # Columns: [theta, alpha, a, d]
        # theta is variable → we store placeholders

        self.dh = np.array([
            [0.0,   np.deg2rad(90),   0.0,    8.5],   # Link 1
            [0.0,   0.0,               10.5,   0.0],   # Link 2
            [0.0,   0.0,               10.0,   0.0],   # Link 3
            [0.0,   np.deg2rad(90),    0.0,    0.0],   # Link 4
            [0.0,   0.0,               4.0+2.5,0.0],   # Link 5
        ], dtype=float)

        # ----------------------------------
        # 2) PHYSICAL OFFSETS (REAL SERVO POS)
        # ----------------------------------
        # These DO NOT affect DH math.
        # Only affect drawing.

        self.servo_offsets = [
            np.array([0, 0, 8.0]),   # Servo 1 real height above DH frame
            np.array([0, 0, 7.5]),   # Servo 2 positioned along link
            np.array([0, 0, 7.5]),   # Servo 3
            np.array([0, 0, 7.5]),   # Servo 4 physical shift
            np.array([0, 0, 0.0]),   # Servo 5 (just example)
        ]

        self.joint_angles = np.zeros(len(self.dh))

    # ---------------------------
    #  SINGLE TRANSFORMATION A_i
    # ---------------------------
    def A(self, theta, alpha, a, d):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,      sa,     ca,    d],
            [0,       0,      0,    1]
        ])

    # ---------------------------
    #  FULL TRANSFORMATION 0→n
    # ---------------------------
    def T_all(self):
        T = np.eye(4)
        frames = [T]

        for i in range(len(self.dh)):
            theta = self.joint_angles[i]
            _, alpha, a, d = self.dh[i]

            A_i = self.A(theta, alpha, a, d)
            T = T @ A_i
            frames.append(T)

        return frames

    # ---------------------------
    #  FRAMES FOR DRAWING WITH OFFSETS
    # ---------------------------
    def T_with_offsets(self):
        frames = self.T_all()
        real_frames = []

        for i, T in enumerate(frames[:-1]):
            offset = np.eye(4)
            offset[:3, 3] = self.servo_offsets[i]
            real_frames.append(T @ offset)

        real_frames.append(frames[-1])
        return real_frames
