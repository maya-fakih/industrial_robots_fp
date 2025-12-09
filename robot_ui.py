import tkinter as tk 
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from robot_model import RobotModel
import serial  # For Arduino communication

# Theme colors
PRIMARY_DEEP = "#4B217A"
ACCENT = "#7C4DA8"
VIOLET_SERVO = "#B98FE0"
PASTEL_BG = "#F3EAF8"
TEXT_BG = "#FFFFFF"

class RobotUI:
    def __init__(self, model: RobotModel, serial_port, baudrate=115200):
        self.model = model
        self.root = tk.Tk()
        self.root.title("Robot Arm Simulator")
        self.root.configure(bg=PASTEL_BG)
        self.root.geometry("1200x700")

        # ----- Serial to Arduino -----
        self.ser = serial.Serial(serial_port, baudrate, timeout=1)
        self.ser.write(b"MODE,PY\n")  # tell Arduino to switch to Python control

        #update mode on close
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        # ----- Arduino offsets (joints 1-4) -----
        self.joint_offsets = [95, 0, 200, 270, 0, 0]  # joints 5-6 manual

        # ----- Frames layout -----
        self.left_frame = tk.Frame(self.root, bg=PASTEL_BG)
        self.center_frame = tk.Frame(self.root, bg=PASTEL_BG)
        self.right_frame = tk.Frame(self.root, bg=PASTEL_BG)

        self.left_frame.pack(side="left", fill="y", padx=8, pady=8)
        self.center_frame.pack(side="left", fill="both", expand=True, padx=8, pady=8)
        self.right_frame.pack(side="right", fill="y", padx=8, pady=8)

        # ----- Left: Transforms -----
        lbl = tk.Label(self.left_frame, text="Transforms (T_0i)", bg=PASTEL_BG, fg=PRIMARY_DEEP)
        lbl.pack(anchor="nw")
        self.text_box = tk.Text(self.left_frame, width=36, height=40, bg=TEXT_BG)
        self.text_box.pack(side="left", fill="y")
        scrollbar = tk.Scrollbar(self.left_frame, command=self.text_box.yview)
        scrollbar.pack(side="right", fill="y")
        self.text_box.config(yscrollcommand=scrollbar.set)

        # ----- Center: 3D plot -----
        fig = Figure(figsize=(6,6), dpi=100)
        self.ax = fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor(PASTEL_BG)
        self.canvas = FigureCanvasTkAgg(fig, master=self.center_frame)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill="both", expand=True)
        self.ax.view_init(elev=25, azim=-60)

        # ----- Right: Controls -----
        self.show_frames = tk.BooleanVar(value=False)
        self.toggle_btn = tk.Button(self.right_frame, text="Show Frames", bg=PRIMARY_DEEP, fg="white",
                                    command=self._toggle_frames)
        self.toggle_btn.pack(pady=(6,12), ipadx=8, ipady=6)

        self.ee_var = tk.StringVar(value="End effector: (0.000, 0.000, 0.000)")
        self.ee_label = tk.Label(self.right_frame, textvariable=self.ee_var, bg=PASTEL_BG, fg=PRIMARY_DEEP)
        self.ee_label.pack(pady=(0,12))

        self.controls_container = tk.Frame(self.right_frame, bg=PASTEL_BG)
        self.controls_container.pack(pady=6)

        # Joint controls
        self.joint_val_labels = []
        for i in range(len(self.model.dh)):
            row = tk.Frame(self.controls_container, bg=PASTEL_BG)
            row.pack(pady=4)

            minus = tk.Button(row, text="-", bg=ACCENT, fg="white", width=3,
                              command=lambda j=i: self._adjust_joint(j, -1))
            minus.pack(side="left", padx=(0,6))

            lbl = tk.Label(row, text="0.0°", width=10, bg=PASTEL_BG)
            lbl.pack(side="left")
            self.joint_val_labels.append(lbl)

            plus = tk.Button(row, text="+", bg=ACCENT, fg="white", width=3,
                             command=lambda j=i: self._adjust_joint(j, +1))
            plus.pack(side="left", padx=(6,0))

        reset_btn = tk.Button(self.right_frame, text="Reset joints", bg=PRIMARY_DEEP, fg="white",
                              command=self._reset_joints)
        reset_btn.pack(pady=(12,6), ipadx=6, ipady=4)

        # Initial draw
        self._update_ui_from_model()
        self._redraw()

    def _on_close(self):
        try:
            self.ser.write(b"MODE,BT\n")  # switch back to manual mode
            self.ser.close()
        except:
            print("Could not close serial port.",e)
        self.root.destroy()
    # ----- UI actions -----
    def _toggle_frames(self):
        v = self.show_frames.get()
        self.show_frames.set(not v)
        self.toggle_btn.configure(text="Hide Frames" if not v else "Show Frames")
        self._redraw()

    def _adjust_joint(self, idx, delta_deg):
        cur_deg = np.degrees(self.model.joint_angles[idx])
        new_deg = cur_deg + delta_deg
        self.model.joint_angles[idx] = np.radians(new_deg)
        self._update_ui_from_model()
        self._redraw()
        self._send_to_robot()  # immediate update to Arduino

    def _reset_joints(self):
        defaults = [0,90,-90,-90,0,0]
        self.model.joint_angles = np.radians(defaults)
        self._update_ui_from_model()
        self._redraw()
        self._send_to_robot()  # reset also sent

    def _update_ui_from_model(self):
        for i, lbl in enumerate(self.joint_val_labels):
            lbl.configure(text=f"{np.degrees(self.model.joint_angles[i]):.1f}°")

        self.text_box.delete('1.0', tk.END)
        frames = self.model.T_all()
        for i, T in enumerate(frames[1:], start=1):
            s = f"T_0{i}:\n" + "\n".join(["\t" + "\t".join([f"{v: .4f}" for v in row]) for row in T]) + "\n\n"
            self.text_box.insert(tk.END, s)

        ee = frames[-1] @ np.array([0,0,0,1])
        self.ee_var.set(f"End effector: ({ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f})")

    # ----- Drawing helpers -----
    def _draw_cylinder(self, T, radius=0.02, height=0.04, res_u=16, res_v=8, color=VIOLET_SERVO):
        u = np.linspace(0, 2*np.pi, res_u)
        v = np.linspace(-height/2, height/2, res_v)
        x = radius * np.outer(np.cos(u), np.ones_like(v))
        y = radius * np.outer(np.sin(u), np.ones_like(v))
        z = np.outer(np.ones_like(u), v)
        pts = np.vstack([x.flatten(), y.flatten(), z.flatten(), np.ones(x.size)])
        pts_world = (T @ pts).reshape((4, x.shape[0], x.shape[1]))
        X, Y, Z = pts_world[0], pts_world[1], pts_world[2]
        self.ax.plot_surface(X, Y, Z, shade=True, rcount=4, ccount=4, color=color)

    def _redraw(self):
        self.ax.cla()
        self.ax.set_box_aspect([1,1,1])
        self.ax.set_facecolor(PASTEL_BG)
        frames = self.model.T_all()
        origins = [f @ np.array([0,0,0,1]) for f in frames]
        xs, ys, zs = [p[0] for p in origins], [p[1] for p in origins], [p[2] for p in origins]
        self.ax.plot(xs, ys, zs, linewidth=3, color=PRIMARY_DEEP)

        if self.show_frames.get():
            for T in frames:
                origin = T @ np.array([0,0,0,1])
                x_axis = T @ np.array([2.5,0,0,1])
                y_axis = T @ np.array([0,2.5,0,1])
                z_axis = T @ np.array([0,0,2.5,1])
                self.ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], color='r', linewidth=2)
                self.ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], color='g', linewidth=2)
                self.ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], color='b', linewidth=2)

        joint_radius, joint_height = 0.8, 1.2
        for T in frames[:-1]:
            self._draw_cylinder(T, radius=joint_radius, height=joint_height, color="#D4B5F0")

        ee = origins[-1]
        self.ax.scatter([ee[0]], [ee[1]], [ee[2]], color=ACCENT, s=40)
        max_extent = max(max(abs(min(xs)), abs(max(xs))), max(abs(min(ys)), abs(max(ys))), max(zs))
        plot_range = max_extent * 0.65
        self.ax.set_xlim(-plot_range, plot_range)
        self.ax.set_ylim(-plot_range, plot_range)
        self.ax.set_zlim(0, plot_range * 2)
        self.canvas.draw()

    # ----- Send angles to Arduino immediately -----
    def _send_to_robot(self):
        # Convert model angles to degrees
        angles_deg = [np.degrees(a) for a in self.model.joint_angles]

        # Apply offsets only to joints 0–3 (first 4)
        first_four = [angles_deg[i] + self.joint_offsets[i] for i in range(4)]

        # Always append 0, 0 for joints 5–6
        last_two = [0, 0]

        # Build final message
        all_six = first_four + last_two

        message = "D," + ",".join([str(int(a)) for a in all_six]) + "\n"
        self.ser.write(message.encode())


    def run(self):
        self.root.mainloop()