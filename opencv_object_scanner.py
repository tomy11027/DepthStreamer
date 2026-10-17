import socket
import struct
import numpy as np
import cv2
import open3d as o3d
import time

# UDP Configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
BUFFER_SIZE = 65535

# Camera Intrinsics
WIDTH = 180
HEIGHT = 240
FX = 203.1
FY = 203.7
CX = 90.0
CY = 120.0

# Global Scan State
is_scanning = False
accumulated_pcd = o3d.geometry.PointCloud()

# Pre-calculate coordinate grid
u_coords, v_coords = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
u_coords = u_coords.flatten()
v_coords = v_coords.flatten()

# Distance Clipping (in meters) - adjust to isolate object from background
MIN_DEPTH_M = 0.2
MAX_DEPTH_M = 1.2
VOXEL_SIZE_M = 0.005 # 5mm voxel merging

def on_mouse_click(event, x, y, flags, param):
    global is_scanning, accumulated_pcd
    if event == cv2.EVENT_LBUTTONDOWN:
        # Check if button clicked (Button box area)
        # Button 1: Start/Stop Scan (x: 20-220, y: 550-600)
        if 20 <= x <= 220 and 550 <= y <= 600:
            is_scanning = not is_scanning
            print(f"Scanning Toggled: {'ON' if is_scanning else 'OFF'}")
        
        # Button 2: Save Object (x: 240-380, y: 550-600)
        elif 240 <= x <= 380 and 550 <= y <= 600:
            if len(accumulated_pcd.points) > 0:
                filename = f"scanned_object_{int(time.time())}.ply"
                cl, ind = accumulated_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
                clean_pcd = accumulated_pcd.select_by_index(ind)
                clean_pcd.estimate_normals()
                o3d.io.write_point_cloud(filename, clean_pcd)
                print(f"✅ Saved 3D scanned object to: {filename}")
            else:
                print("⚠️ Cannot save empty scan! Start scanning first.")

        # Button 3: Clear (x: 400-460, y: 550-600)
        elif 400 <= x <= 460 and 550 <= y <= 600:
            accumulated_pcd.clear()
            print("🗑️ Scan cleared.")

def main():
    global is_scanning, accumulated_pcd

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.02)

    window_name = "3D Object Scanner (OpenCV GUI)"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, on_mouse_click)

    print("==================================================")
    print(" 📷 Depth Camera 3D Object Scanner Running")
    print("  - Click '[START SCANNING]' button or press 'S' to start/stop")
    print("  - Click '[SAVE 3D]' button or press 'W' to save .ply file")
    print("  - Press 'C' to clear scan buffer")
    print("  - Press 'Q' to quit")
    print("==================================================")

    frame_buffer = np.zeros((180, 240), dtype=np.uint16)

    while True:
        try:
            try:
                data, _ = sock.recvfrom(BUFFER_SIZE)
                if len(data) >= 16:
                    w, h, y_offset, y_count = struct.unpack('<IIII', data[:16])
                    if len(data) - 16 == w * y_count * 2:
                        depth_raw = np.frombuffer(data, dtype=np.uint16, offset=16)
                        distance = np.bitwise_and(depth_raw, 0x1FFF).reshape((y_count, w))
                        frame_buffer[y_offset:y_offset+y_count, :] = distance

                        # When frame complete
                        if y_offset + y_count >= h:
                            rotated = np.rot90(frame_buffer, k=-1)
                            z = rotated.flatten().astype(np.float32) / 1000.0

                            # Distance filtering
                            valid_mask = (z >= MIN_DEPTH_M) & (z <= MAX_DEPTH_M)
                            
                            # If scanning, aggregate points
                            if is_scanning and np.any(valid_mask):
                                z_val = z[valid_mask]
                                u_val = u_coords[valid_mask]
                                v_val = v_coords[valid_mask]

                                x = (u_val - CX) * z_val / FX
                                y = (v_val - CY) * z_val / FY
                                points = np.column_stack((x, -y, -z_val))

                                z_norm = np.clip((z_val - MIN_DEPTH_M) / (MAX_DEPTH_M - MIN_DEPTH_M), 0, 1)
                                colors = np.column_stack((1.0 - z_norm, np.zeros_like(z_norm), z_norm))

                                frame_pcd = o3d.geometry.PointCloud()
                                frame_pcd.points = o3d.utility.Vector3dVector(points)
                                frame_pcd.colors = o3d.utility.Vector3dVector(colors)

                                accumulated_pcd += frame_pcd
                                accumulated_pcd = accumulated_pcd.voxel_down_sample(voxel_size=VOXEL_SIZE_M)

                            # Prepare 2D visualization
                            norm_depth = np.zeros_like(rotated, dtype=np.uint8)
                            if np.any(valid_mask):
                                clipped = np.clip(rotated[valid_mask], MIN_DEPTH_M * 1000, MAX_DEPTH_M * 1000)
                                scaled = ((clipped - MIN_DEPTH_M * 1000) / ((MAX_DEPTH_M - MIN_DEPTH_M) * 1000) * 255).astype(np.uint8)
                                norm_depth[valid_mask] = 255 - scaled

                            color_depth = cv2.applyColorMap(norm_depth, cv2.COLORMAP_JET)
                            color_depth[~valid_mask] = [20, 20, 20]

                            # Upscale image & build UI layout
                            disp = cv2.resize(color_depth, (480, 520), interpolation=cv2.INTER_NEAREST)

                            # Bottom Control Bar Area (Height: 100px)
                            bottom_panel = np.zeros((100, 480, 3), dtype=np.uint8) + 35

                            # Draw Buttons
                            # Button 1: Start/Stop
                            btn_color = (0, 180, 0) if not is_scanning else (0, 0, 220)
                            btn_text = "START SCANNING" if not is_scanning else "STOP SCANNING"
                            cv2.rectangle(bottom_panel, (20, 30), (220, 80), btn_color, -1)
                            cv2.putText(bottom_panel, btn_text, (35, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

                            # Button 2: Save
                            cv2.rectangle(bottom_panel, (240, 30), (380, 80), (180, 100, 0), -1)
                            cv2.putText(bottom_panel, "SAVE 3D", (270, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

                            # Button 3: Clear
                            cv2.rectangle(bottom_panel, (400, 30), (460, 80), (80, 80, 80), -1)
                            cv2.putText(bottom_panel, "CLEAR", (408, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

                            # Text stats overlay
                            pts_count = len(accumulated_pcd.points)
                            cv2.putText(bottom_panel, f"Accumulated 3D Points: {pts_count:,}", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

                            # Combine video and bottom controls panel
                            combined_ui = np.vstack((disp, bottom_panel))

                            cv2.imshow(window_name, combined_ui)

            except socket.timeout:
                pass

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                is_scanning = not is_scanning
                print(f"Scanning Toggled: {'ON' if is_scanning else 'OFF'}")
            elif key == ord('w'):
                if len(accumulated_pcd.points) > 0:
                    filename = f"scanned_object_{int(time.time())}.ply"
                    o3d.io.write_point_cloud(filename, accumulated_pcd)
                    print(f"✅ Saved: {filename}")
            elif key == ord('c'):
                accumulated_pcd.clear()
                print("Clear")

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")

    sock.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
