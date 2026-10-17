import socket
import struct
import threading
import time
import numpy as np
import cv2
import open3d as o3d
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PIL import Image, ImageTk

# ==========================================
# Camera & Depth Configuration
# ==========================================
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
BUFFER_SIZE = 65535

# Sensor Intrinsics (Android DepthStreamer 180x240 rotated)
WIDTH = 180
HEIGHT = 240
FX = 203.1
FY = 203.7
CX = 90.0
CY = 120.0

class ObjectScannerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("3D Object Depth Scanner (Smart 3D Alignment)")
        self.root.geometry("1300x760")
        self.root.configure(bg="#1e1e2e")

        # Scanning & Motion Tracking state
        self.is_scanning = False
        self.is_running = True
        self.use_icp_alignment = True # Auto 3D camera pose tracking
        self.latest_depth_frame = None
        self.lock = threading.Lock()

        # Thread-Safe Point Cloud Buffer (Stored as NumPy arrays to avoid C++ Open3D thread crashes)
        self.accumulated_pts = np.empty((0, 3), dtype=np.float32)
        self.accumulated_cols = np.empty((0, 3), dtype=np.float32)
        self.last_frame_pts = None
        self.scan_point_count = 0

        # Pre-calculate coordinate grid
        u_grid, v_grid = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
        self.u_flat = u_grid.flatten()
        self.v_flat = v_grid.flatten()

        self._build_ui()

        # Start UDP receiver thread
        self.udp_thread = threading.Thread(target=self._udp_receiver_loop, daemon=True)
        self.udp_thread.start()

        # Start UI render loop
        self.root.after(30, self._update_ui_loop)

    def _build_ui(self):
        # Header / Title
        header = tk.Frame(self.root, bg="#181825")
        header.pack(fill=tk.X, pady=5)
        title_label = tk.Label(
            header, 
            text="📷 3D Depth Camera Scanner with Auto Motion Alignment", 
            font=("Segoe UI", 16, "bold"), 
            fg="#cdd6f4", 
            bg="#181825"
        )
        title_label.pack(side=tk.LEFT, padx=15)

        # Main Layout
        main_container = tk.Frame(self.root, bg="#1e1e2e")
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left Container for Dual Feeds
        left_container = tk.Frame(main_container, bg="#181825", bd=2, relief=tk.GROOVE)
        left_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        v_headers = tk.Frame(left_container, bg="#181825")
        v_headers.pack(fill=tk.X, pady=5)
        tk.Label(v_headers, text="2D Depth Camera Feed", font=("Segoe UI", 11, "bold"), fg="#89b4fa", bg="#181825").pack(side=tk.LEFT, expand=True)
        tk.Label(v_headers, text="Real-Time Aligned 3D Model", font=("Segoe UI", 11, "bold"), fg="#a6e3a1", bg="#181825").pack(side=tk.RIGHT, expand=True)

        feeds_frame = tk.Frame(left_container, bg="#11111b")
        feeds_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.video_canvas = tk.Label(feeds_frame, bg="#11111b")
        self.video_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2, pady=2)

        self.pcd_canvas = tk.Label(feeds_frame, bg="#11111b")
        self.pcd_canvas.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=2, pady=2)

        # Right Column: Controls Panel
        right_panel = tk.Frame(main_container, bg="#181825", width=340, bd=2, relief=tk.GROOVE)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=5)
        right_panel.pack_propagate(False)

        panel_title = tk.Label(right_panel, text="Scanner Controls", font=("Segoe UI", 14, "bold"), fg="#cdd6f4", bg="#181825")
        panel_title.pack(pady=10)

        # Scan Button
        self.btn_scan = tk.Button(
            right_panel,
            text="▶ START SCANNING OBJECT",
            font=("Segoe UI", 11, "bold"),
            bg="#a6e3a1",
            fg="#11111b",
            activebackground="#94e2d5",
            height=2,
            relief=tk.RAISED,
            command=self.toggle_scan
        )
        self.btn_scan.pack(fill=tk.X, padx=15, pady=8)

        # Action Buttons
        self.btn_save = tk.Button(
            right_panel,
            text="💾 Save 3D Model (.ply)",
            font=("Segoe UI", 10, "bold"),
            bg="#89b4fa",
            fg="#11111b",
            command=self.save_mesh
        )
        self.btn_save.pack(fill=tk.X, padx=15, pady=4)

        self.btn_view_3d = tk.Button(
            right_panel,
            text="👁️ Open Standalone 3D Viewer",
            font=("Segoe UI", 10, "bold"),
            bg="#cba6f7",
            fg="#11111b",
            command=self.open_3d_viewer
        )
        self.btn_view_3d.pack(fill=tk.X, padx=15, pady=4)

        self.btn_clear = tk.Button(
            right_panel,
            text="🗑️ Clear Scan Buffer",
            font=("Segoe UI", 10),
            bg="#f38ba8",
            fg="#11111b",
            command=self.clear_scan
        )
        self.btn_clear.pack(fill=tk.X, padx=15, pady=4)

        # Motion Alignment Toggle
        ttk.Separator(right_panel, orient="horizontal").pack(fill=tk.X, padx=15, pady=10)

        self.var_icp = tk.BooleanVar(value=True)
        chk_icp = tk.Checkbutton(
            right_panel,
            text="Auto-Align Camera Motion (ICP)",
            variable=self.var_icp,
            font=("Segoe UI", 10, "bold"),
            fg="#a6e3a1",
            bg="#181825",
            selectcolor="#11111b",
            activebackground="#181825",
            activeforeground="#a6e3a1",
            command=self._update_icp_setting
        )
        chk_icp.pack(anchor="w", padx=15, pady=3)

        tk.Label(
            right_panel, 
            text="Aligns 3D shapes automatically as you move camera around the object.", 
            font=("Segoe UI", 8), 
            fg="#a6adc8", 
            bg="#181825",
            wraplength=300,
            justify=tk.LEFT
        ).pack(anchor="w", padx=15)

        # Distance Filter Sliders
        ttk.Separator(right_panel, orient="horizontal").pack(fill=tk.X, padx=15, pady=10)
        filter_label = tk.Label(right_panel, text="Object Distance Filter (Meters)", font=("Segoe UI", 10, "bold"), fg="#cdd6f4", bg="#181825")
        filter_label.pack(anchor="w", padx=15)

        tk.Label(right_panel, text="Min Distance (Near Clip):", font=("Segoe UI", 9), fg="#a6adc8", bg="#181825").pack(anchor="w", padx=15, pady=(3, 0))
        self.slider_min_dist = tk.Scale(right_panel, from_=0.1, to=2.0, resolution=0.05, orient=tk.HORIZONTAL, bg="#181825", fg="#cdd6f4", highlightthickness=0)
        self.slider_min_dist.set(0.2)
        self.slider_min_dist.pack(fill=tk.X, padx=15)

        tk.Label(right_panel, text="Max Distance (Far Clip):", font=("Segoe UI", 9), fg="#a6adc8", bg="#181825").pack(anchor="w", padx=15, pady=(3, 0))
        self.slider_max_dist = tk.Scale(right_panel, from_=0.3, to=4.0, resolution=0.05, orient=tk.HORIZONTAL, bg="#181825", fg="#cdd6f4", highlightthickness=0)
        self.slider_max_dist.set(1.2)
        self.slider_max_dist.pack(fill=tk.X, padx=15)

        tk.Label(right_panel, text="Voxel Downsample Size (mm):", font=("Segoe UI", 9), fg="#a6adc8", bg="#181825").pack(anchor="w", padx=15, pady=(3, 0))
        self.slider_voxel = tk.Scale(right_panel, from_=2, to=20, resolution=1, orient=tk.HORIZONTAL, bg="#181825", fg="#cdd6f4", highlightthickness=0)
        self.slider_voxel.set(5)
        self.slider_voxel.pack(fill=tk.X, padx=15)

        # 3D View Angle
        tk.Label(right_panel, text="Real-Time 3D View Angle:", font=("Segoe UI", 9), fg="#a6adc8", bg="#181825").pack(anchor="w", padx=15, pady=(3, 0))
        self.slider_angle = tk.Scale(right_panel, from_=0, to=360, resolution=5, orient=tk.HORIZONTAL, bg="#181825", fg="#cdd6f4", highlightthickness=0)
        self.slider_angle.set(45)
        self.slider_angle.pack(fill=tk.X, padx=15)

        # Status Panel
        ttk.Separator(right_panel, orient="horizontal").pack(fill=tk.X, padx=15, pady=10)
        self.lbl_status = tk.Label(right_panel, text="Status: IDLE", font=("Segoe UI", 11, "bold"), fg="#fab387", bg="#181825")
        self.lbl_status.pack(pady=3)

        self.lbl_stats = tk.Label(right_panel, text="Points Captured: 0\n3D Model: 0 points", font=("Segoe UI", 9), fg="#a6adc8", bg="#181825", justify=tk.CENTER)
        self.lbl_stats.pack(pady=3)

    def _update_icp_setting(self):
        self.use_icp_alignment = self.var_icp.get()

    def toggle_scan(self):
        self.is_scanning = not self.is_scanning
        if self.is_scanning:
            self.btn_scan.config(text="⏹ STOP SCANNING", bg="#f38ba8", fg="#11111b")
            self.lbl_status.config(text="Status: 🔴 SCANNING ACTIVE", fg="#f38ba8")
        else:
            self.btn_scan.config(text="▶ START SCANNING OBJECT", bg="#a6e3a1", fg="#11111b")
            self.lbl_status.config(text="Status: 🟢 SCAN STOPPED", fg="#a6e3a1")

    def clear_scan(self):
        with self.lock:
            self.accumulated_pts = np.empty((0, 3), dtype=np.float32)
            self.accumulated_cols = np.empty((0, 3), dtype=np.float32)
            self.last_frame_pts = None
            self.scan_point_count = 0
        self.lbl_stats.config(text="Points Captured: 0\n3D Model: 0 points")
        self.lbl_status.config(text="Status: Buffer Cleared", fg="#fab387")

    def _udp_receiver_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((UDP_IP, UDP_PORT))
            sock.settimeout(0.1)
        except Exception as e:
            print(f"Socket Bind Warning: {e}")
            self.lbl_status.config(text=f"Port 5005 in use by another app", fg="#f38ba8")
            return

        frame_buffer = np.zeros((180, 240), dtype=np.uint16)

        while self.is_running:
            try:
                data, _ = sock.recvfrom(BUFFER_SIZE)
                if len(data) >= 16:
                    w, h, y_offset, y_count = struct.unpack('<IIII', data[:16])
                    if len(data) - 16 == w * y_count * 2:
                        depth_raw = np.frombuffer(data, dtype=np.uint16, offset=16)
                        distance = np.bitwise_and(depth_raw, 0x1FFF).reshape((y_count, w))
                        frame_buffer[y_offset:y_offset+y_count, :] = distance

                        if y_offset + y_count >= h:
                            rotated = np.rot90(frame_buffer, k=-1)
                            with self.lock:
                                self.latest_depth_frame = rotated.copy()

                            if self.is_scanning:
                                self._process_scan_frame(rotated)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"UDP Error: {e}")

        sock.close()

    def _process_scan_frame(self, frame):
        min_d = self.slider_min_dist.get()
        max_d = self.slider_max_dist.get()
        voxel_m = self.slider_voxel.get() / 1000.0

        z = frame.flatten().astype(np.float32) / 1000.0
        valid_mask = (z >= min_d) & (z <= max_d)

        if not np.any(valid_mask):
            return

        z_valid = z[valid_mask]
        u_valid = self.u_flat[valid_mask]
        v_valid = self.v_flat[valid_mask]

        x = (u_valid - CX) * z_valid / FX
        y = (v_valid - CY) * z_valid / FY

        points = np.column_stack((x, -y, -z_valid)).astype(np.float32)

        z_norm = np.clip((z_valid - min_d) / (max_d - min_d + 1e-5), 0, 1)
        colors = np.column_stack((1.0 - z_norm, 0.6 * np.ones_like(z_norm), z_norm)).astype(np.float32)

        # Downsample frame points using simple voxel quantization in NumPy
        if voxel_m > 0.001:
            quantized = np.round(points / voxel_m)
            _, unique_idx = np.unique(quantized, axis=0, return_index=True)
            points = points[unique_idx]
            colors = colors[unique_idx]

        with self.lock:
            if len(self.accumulated_pts) == 0:
                self.accumulated_pts = points
                self.accumulated_cols = colors
                self.last_frame_pts = points
            else:
                if self.use_icp_alignment and self.last_frame_pts is not None and len(self.last_frame_pts) > 30 and len(points) > 30:
                    try:
                        # Construct temporary Open3D clouds solely inside lock for ICP
                        src_cloud = o3d.geometry.PointCloud()
                        src_cloud.points = o3d.utility.Vector3dVector(points)

                        tgt_cloud = o3d.geometry.PointCloud()
                        tgt_cloud.points = o3d.utility.Vector3dVector(self.accumulated_pts)

                        reg = o3d.pipelines.registration.registration_icp(
                            src_cloud, tgt_cloud, 0.06, np.identity(4),
                            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=25)
                        )
                        src_cloud.transform(reg.transformation)
                        points = np.asarray(src_cloud.points, dtype=np.float32)
                    except Exception:
                        pass

                # Append aligned points
                self.accumulated_pts = np.vstack((self.accumulated_pts, points))
                self.accumulated_cols = np.vstack((self.accumulated_cols, colors))
                self.last_frame_pts = points

                # Voxel downsample global buffer in NumPy
                if voxel_m > 0.001 and len(self.accumulated_pts) > 100:
                    quantized = np.round(self.accumulated_pts / voxel_m)
                    _, unique_idx = np.unique(quantized, axis=0, return_index=True)
                    self.accumulated_pts = self.accumulated_pts[unique_idx]
                    self.accumulated_cols = self.accumulated_cols[unique_idx]

            self.scan_point_count += len(points)

    def _render_3d_pcd_projection(self, points, colors, w=440, h=550):
        """ Thread-safe 100% NumPy rendering of 3D points """
        canvas = np.zeros((h, w, 3), dtype=np.uint8) + 18

        if len(points) == 0:
            cv2.putText(canvas, "3D Aligned Model preview", (w//2 - 100, h//2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 140), 1)
            cv2.putText(canvas, "Press START SCANNING to begin", (w//2 - 110, h//2 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 140), 1)
            return canvas

        center = np.mean(points, axis=0)
        pts_centered = points - center

        yaw_rad = np.radians(self.slider_angle.get())
        pitch_rad = np.radians(20)

        cos_y, sin_y = np.cos(yaw_rad), np.sin(yaw_rad)
        R_y = np.array([[cos_y, 0, sin_y], [0, 1, 0], [-sin_y, 0, cos_y]])

        cos_p, sin_p = np.cos(pitch_rad), np.sin(pitch_rad)
        R_p = np.array([[1, 0, 0], [0, cos_p, -sin_p], [0, sin_p, cos_p]])

        rotated = pts_centered @ R_y.T @ R_p.T

        max_dist = np.max(np.abs(rotated)) + 1e-5
        scale = min(w, h) * 0.42 / max_dist

        px = (rotated[:, 0] * scale + w / 2).astype(np.int32)
        py = (-rotated[:, 1] * scale + h / 2).astype(np.int32)

        valid = (px >= 0) & (px < w) & (py >= 0) & (py < h)
        px_v, py_v = px[valid], py[valid]

        if len(colors) == len(points):
            cols_v = (colors[valid] * 255).astype(np.uint8)
        else:
            cols_v = np.tile([0, 255, 128], (len(px_v), 1))

        # Fast Vectorized Canvas Assignment
        canvas[py_v, px_v] = cols_v[:, ::-1]

        align_mode_str = "ICP Auto-Aligned" if self.use_icp_alignment else "Raw Overlay"
        cv2.putText(canvas, f"3D Model ({align_mode_str}): {len(points):,} pts", (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 180), 1)
        return canvas

    def _update_ui_loop(self):
        with self.lock:
            frame = self.latest_depth_frame.copy() if self.latest_depth_frame is not None else None
            pts_copy = self.accumulated_pts.copy()
            cols_copy = self.accumulated_cols.copy()
            num_points = len(pts_copy)

        if frame is not None:
            min_d = self.slider_min_dist.get() * 1000.0
            max_d = self.slider_max_dist.get() * 1000.0

            valid_mask = (frame >= min_d) & (frame <= max_d)
            norm = np.zeros_like(frame, dtype=np.uint8)

            if np.any(valid_mask):
                clipped = np.clip(frame[valid_mask], min_d, max_d)
                scaled = ((clipped - min_d) / (max_d - min_d + 1e-5) * 255).astype(np.uint8)
                norm[valid_mask] = 255 - scaled

            color_depth = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
            color_depth[~valid_mask] = [30, 30, 30]

            scan_text = "🔴 SCANNING ACTIVE" if self.is_scanning else "PREVIEW"
            color_text = (0, 0, 255) if self.is_scanning else (0, 255, 0)
            cv2.putText(color_depth, scan_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_text, 2)

            disp = cv2.resize(color_depth, (440, 550), interpolation=cv2.INTER_NEAREST)
            disp_rgb = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)

            img1 = Image.fromarray(disp_rgb)
            imgtk1 = ImageTk.PhotoImage(image=img1)
            self.video_canvas.imgtk = imgtk1
            self.video_canvas.configure(image=imgtk1)

        pcd_img = self._render_3d_pcd_projection(pts_copy, cols_copy, w=440, h=550)
        pcd_rgb = cv2.cvtColor(pcd_img, cv2.COLOR_BGR2RGB)

        img2 = Image.fromarray(pcd_rgb)
        imgtk2 = ImageTk.PhotoImage(image=img2)
        self.pcd_canvas.imgtk = imgtk2
        self.pcd_canvas.configure(image=imgtk2)

        if self.is_scanning:
            self.lbl_stats.config(text=f"Total Captured: {self.scan_point_count:,}\n3D Model: {num_points:,} pts")

        if self.is_running:
            self.root.after(30, self._update_ui_loop)

    def open_3d_viewer(self):
        with self.lock:
            if len(self.accumulated_pts) == 0:
                messagebox.showwarning("Empty Scan", "No 3D scan data collected yet.")
                return
            pts = self.accumulated_pts.copy()
            cols = self.accumulated_cols.copy()

        def _show_3d():
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            pcd.colors = o3d.utility.Vector3dVector(cols)
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            inlier_cloud = pcd.select_by_index(ind)
            o3d.visualization.draw_geometries([inlier_cloud], window_name="Full Interactive 3D Model", width=900, height=700)

        threading.Thread(target=_show_3d, daemon=True).start()

    def save_mesh(self):
        with self.lock:
            if len(self.accumulated_pts) < 50:
                messagebox.showerror("Scan Error", "Not enough 3D points captured to save.")
                return
            pts = self.accumulated_pts.copy()
            cols = self.accumulated_cols.copy()

        filepath = filedialog.asksaveasfilename(
            defaultextension=".ply",
            filetypes=[("Polygon File Format", "*.ply"), ("Wavefront OBJ", "*.obj"), ("PCD File", "*.pcd")],
            title="Save 3D Scanned Object"
        )
        if not filepath:
            return

        try:
            pcd_to_save = o3d.geometry.PointCloud()
            pcd_to_save.points = o3d.utility.Vector3dVector(pts)
            pcd_to_save.colors = o3d.utility.Vector3dVector(cols)

            cl, ind = pcd_to_save.remove_statistical_outlier(nb_neighbors=25, std_ratio=1.5)
            clean_pcd = pcd_to_save.select_by_index(ind)
            clean_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))

            o3d.io.write_point_cloud(filepath, clean_pcd)
            messagebox.showinfo("Saved Successfully", f"3D Scanned object saved to:\n{filepath}\n\nPoints: {len(clean_pcd.points):,}")
            self.lbl_status.config(text=f"Status: Saved -> {filepath.split('/')[-1]}", fg="#89b4fa")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save 3D object: {e}")

    def on_close(self):
        self.is_running = False
        self.root.destroy()

def main():
    root = tk.Tk()
    app = ObjectScannerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
