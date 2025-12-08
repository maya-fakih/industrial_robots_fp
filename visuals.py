import tkinter as tk
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from robot_model import RobotModel

LINK_COLORS = ["red","green","blue","purple","orange"]

def draw_frame(ax,T,name,L=0.05):
    o = T[:3,3]
    x = o + T[:3,0]*L
    y = o + T[:3,1]*L
    z = o + T[:3,2]*L
    ax.plot([o[0],x[0]],[o[1],x[1]],[o[2],x[2]],color="red",linewidth=2)
    ax.plot([o[0],y[0]],[o[1],y[1]],[o[2],y[2]],color="green",linewidth=2)
    ax.plot([o[0],z[0]],[o[1],z[1]],[o[2],z[2]],color="blue",linewidth=2)
    ax.text(o[0],o[1],o[2]," "+name,fontsize=8,color="black")

def angle_clip(v): return max(min(v,180.0),-180.0)

class RobotVisualizerApp:
    def __init__(self, root):
        self.root = root
        root.title("4-DOF Robot Visualizer")
        self.robot_model = RobotModel()

        # MAIN container
        main = tk.Frame(root); main.pack(fill=tk.BOTH,expand=True)
        # LEFT: matrices
        left_panel = tk.Frame(main); left_panel.pack(side=tk.LEFT,padx=8,pady=8,fill=tk.Y)
        tk.Label(left_panel,text="T matrices & EE position",font=("Arial",10,"bold")).pack(anchor="w")
        tbox_frame = tk.Frame(left_panel); tbox_frame.pack(fill=tk.Y,expand=True)
        self.scrollbar = tk.Scrollbar(tbox_frame,orient=tk.VERTICAL)
        self.text_box = tk.Text(tbox_frame,width=40,height=30,font=("Courier",9),yscrollcommand=self.scrollbar.set)
        self.scrollbar.config(command=self.text_box.yview)
        self.text_box.pack(side=tk.LEFT,fill=tk.Y,expand=True)
        self.scrollbar.pack(side=tk.RIGHT,fill=tk.Y)
        # CENTER: 3D plot
        import matplotlib
        matplotlib.use("TkAgg")
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure
        center_panel = tk.Frame(main); center_panel.pack(side=tk.LEFT,padx=8,pady=8,fill=tk.BOTH,expand=True)
        self.fig = Figure(figsize=(6.5,6.0))
        self.ax = self.fig.add_subplot(111,projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig,master=center_panel)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH,expand=True)
        # RIGHT: controls
        right_panel = tk.Frame(main); right_panel.pack(side=tk.LEFT,padx=12,pady=8,fill=tk.Y)
        self.angle_vars=[]; self.entries=[]
        for i in range(4):
            row=tk.Frame(right_panel); row.pack(anchor="w",pady=6)
            tk.Label(row,text=f"Joint {i+1} angle (°):",width=18,anchor="w").grid(row=0,column=0)
            v=tk.DoubleVar(value=0.0); self.angle_vars.append(v)
            ent=tk.Entry(row,textvariable=v,width=7); ent.grid(row=0,column=1,padx=4); self.entries.append(ent)
            ent.bind("<Return>",lambda e,j=i:self.on_entry_set(j))
            tk.Button(row,text="+",bg="#a3e6ff",width=3,command=lambda j=i:self.increment(j,+5)).grid(row=0,column=2,padx=2)
            tk.Button(row,text="-",bg="#ffb3b3",width=3,command=lambda j=i:self.increment(j,-5)).grid(row=0,column=3,padx=2)
        tk.Button(right_panel,text="Toggle Joint Frames",bg="#d5c4ff",width=20,command=self.toggle_frames).pack(pady=(8,10))
        tk.Button(right_panel,text="Reset Pose",width=20,command=self.reset_pose).pack(pady=(4,0))

        self.show_frames=False
        self.default_pose=[75.0,0.0,-25.0,0.0]
        self.joint_angles=[0.0]*4
        self.reset_pose()

    def increment(self,j,d):
        new=angle_clip(self.angle_vars[j].get()+d)
        self.angle_vars[j].set(new)
        self.joint_angles[j]=new
        self.update()

    def on_entry_set(self,j):
        try: v=float(self.entries[j].get())
        except: v=self.angle_vars[j].get()
        v=angle_clip(v)
        self.angle_vars[j].set(v)
        self.joint_angles[j]=v
        self.update()

    def toggle_frames(self):
        self.show_frames = not self.show_frames
        self.update()

    def reset_pose(self):
        for i in range(4):
            self.angle_vars[i].set(self.default_pose[i])
            self.joint_angles[i]=self.default_pose[i]
        if hasattr(self,"ax"):
            self.ax.view_init(elev=20,azim=-60)
        self.update()

    # --- Cylinder drawing helpers ---
    def _draw_cylinder(self, center, axis, radius=0.05, height=0.12, color='plum', resolution=20):
        axis = axis / np.linalg.norm(axis)
        theta = np.linspace(0, 2*np.pi, resolution)
        z_local = np.linspace(-height/2, height/2, 2)
        theta_grid, z_grid = np.meshgrid(theta, z_local)
        x_local = radius * np.cos(theta_grid)
        y_local = radius * np.sin(theta_grid)
        z_vec = np.array([0,0,1])
        R = self._rotation_matrix_from_vectors(z_vec, axis)
        points = np.stack([x_local.flatten(), y_local.flatten(), z_grid.flatten()])
        points_transformed = R @ points
        x_world = points_transformed[0,:].reshape(x_local.shape) + center[0]
        y_world = points_transformed[1,:].reshape(y_local.shape) + center[1]
        z_world = points_transformed[2,:].reshape(z_grid.shape) + center[2]
        self.ax.plot_surface(x_world, y_world, z_world, color=color, alpha=0.7, shade=True)
        # Caps
        for z_cap in [-height/2, height/2]:
            cap_points = []
            for t in theta:
                pt_local = np.array([radius*np.cos(t), radius*np.sin(t), z_cap])
                pt_world = R @ pt_local + center
                cap_points.append(pt_world)
            cap = Poly3DCollection([cap_points], facecolor=color, edgecolor='purple', alpha=0.7)
            self.ax.add_collection3d(cap)

    def _rotation_matrix_from_vectors(self, vec1, vec2):
        a = vec1 / np.linalg.norm(vec1)
        b = vec2 / np.linalg.norm(vec2)
        v = np.cross(a,b)
        c = np.dot(a,b)
        if np.allclose(v,0):
            if c>0: return np.eye(3)
            perp = np.array([1,0,0]) if abs(a[0])<0.9 else np.array([0,1,0])
            perp = perp - np.dot(perp,a)*a
            perp = perp/np.linalg.norm(perp)
            return 2*np.outer(perp,perp)-np.eye(3)
        vx = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
        R = np.eye(3)+vx+vx@vx*(1/(1+c))
        return R

    # --- Update visualization ---
    def update(self):
        joints = [self.angle_vars[i].get() for i in range(4)]
        frames = self.robot_model.fk_actual(joints)
        self.ax.cla()
        pts = [frames[i][:3,3] for i in range(len(frames))]
        P = np.array(pts)
        # Links
        for i in range(3):
            p1=P[i]; p2=P[i+1]
            self.ax.plot([p1[0],p2[0]], [p1[1],p2[1]], [p1[2],p2[2]], color=LINK_COLORS[i], linewidth=4)
        # Cylinders
        for i in range(4):
            center = frames[i][:3,3]
            z_axis = frames[i][:3,2]
            self._draw_cylinder(center, z_axis, radius=0.05, height=0.12, color='plum')
        # Frames
        draw_frame(self.ax,frames[0],"Base",L=0.06)
        draw_frame(self.ax,frames[-1],"EE",L=0.06)
        if self.show_frames:
            for i in range(1,4):
                draw_frame(self.ax,frames[i],f"F{i}",L=0.045)
        margin=0.12
        xmin,ymin,zmin=np.min(P,axis=0)-margin
        xmax,ymax,zmax=np.max(P,axis=0)+margin
        self.ax.set_xlim(xmin,xmax); self.ax.set_ylim(ymin,ymax); self.ax.set_zlim(max(0,zmin),zmax+margin)
        self.ax.set_title("4-DOF Robot")
        try: self.ax.set_box_aspect([1,1,0.8])
        except: pass
        # matrices
        self.text_box.delete("1.0",tk.END)
        for i in range(1,4):
            T=frames[i]; self.text_box.insert(tk.END,f"T0{i} =\n{np.round(T,4)}\n\n")
        ee=frames[-1][:3,3]
        self.text_box.insert(tk.END,f"End Effector Position:\nX={ee[0]:.4f}  Y={ee[1]:.4f}  Z={ee[2]:.4f}\n")
        self.text_box.see("1.0")
        self.canvas.draw()

def main():
    root=tk.Tk()
    app=RobotVisualizerApp(root)
    root.mainloop()

if __name__=="__main__":
    main()
