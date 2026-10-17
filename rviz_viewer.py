import socket
import struct
import argparse
import numpy as np
import open3d as o3d

# Default Camera Intrinsics (Calculated from FoV 61 deg / 47.82 deg)
# The Android ToF sensor is 240x180 landscape.
# When held in portrait orientation, rotated 90 deg clockwise gives 180x240.
DEFAULT_WIDTH = 180
DEFAULT_HEIGHT = 240
DEFAULT_FX = 203.1
DEFAULT_FY = 203.7
DEFAULT_CX = 90.0
DEFAULT_CY = 120.0

def parse_args():
    parser = argparse.ArgumentParser(
        description="DepthStreamer 3D Point Cloud Visualizer (RViz-Style) using Open3D."
    )
    parser.add_argument("--ip", default="0.0.0.0", help="UDP IP to bind and listen on (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=5005, help="UDP port to listen on (default: 5005)")
    parser.add_argument("--min-range", type=float, default=0.05, help="Min distance in meters (default: 0.05 m)")
    parser.add_argument("--max-range", type=float, default=5.0, help="Max distance in meters (default: 5.0 m)")
    parser.add_argument("--fx", type=float, default=DEFAULT_FX, help=f"Focal length X (default: {DEFAULT_FX})")
    parser.add_argument("--fy", type=float, default=DEFAULT_FY, help=f"Focal length Y (default: {DEFAULT_FY})")
    parser.add_argument("--cx", type=float, default=DEFAULT_CX, help=f"Principal point X (default: {DEFAULT_CX})")
    parser.add_argument("--cy", type=float, default=DEFAULT_CY, help=f"Principal point Y (default: {DEFAULT_CY})")
    return parser.parse_args()

def main():
    args = parse_args()
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    sock.settimeout(0.005)
    
    print("=" * 60)
    print(f"[*] DepthStreamer 3D RViz Viewer listening on {args.ip}:{args.port}")
    print(f"    Depth Range: {args.min_range:.2f}m to {args.max_range:.2f}m")
    print("    Controls: Left Click + Drag = Rotate | Right Click + Drag = Pan")
    print("              Mouse Wheel = Zoom | Close window to exit")
    print("=" * 60)
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="DepthStreamer - Realtime 3D Point Cloud", width=1024, height=768)
    
    # Coordinate frame / axes for orientation reference
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
    vis.add_geometry(axis)
    
    pcd = o3d.geometry.PointCloud()
    point_cloud_added = False
    
    # Pre-calculate UV grid
    u_coords, v_coords = np.meshgrid(np.arange(DEFAULT_WIDTH), np.arange(DEFAULT_HEIGHT))
    u_coords = u_coords.flatten()
    v_coords = v_coords.flatten()
    
    BUFFER_SIZE = 65535
    frame_buffer = np.zeros((180, 240), dtype=np.uint16)
    
    while True:
        try:
            try:
                data, addr = sock.recvfrom(BUFFER_SIZE)
                
                if len(data) >= 16:
                    w, h, y_offset, y_count = struct.unpack('<IIII', data[:16])
                    
                    if len(data) - 16 == w * y_count * 2:
                        depth_array_raw = np.frombuffer(data, dtype=np.uint16, offset=16)
                        distance_array = np.bitwise_and(depth_array_raw, 0x1FFF).reshape((y_count, w))
                        
                        frame_buffer[y_offset:y_offset+y_count, :] = distance_array
                        
                        # When bottom chunk arrives, project into 3D
                        if y_offset + y_count >= h:
                            rotated_frame = np.rot90(frame_buffer, k=-1)
                            z = rotated_frame.flatten().astype(np.float32) / 1000.0
                            
                            valid_mask = (z > args.min_range) & (z < args.max_range)
                            z_valid = z[valid_mask]
                            u_valid = u_coords[valid_mask]
                            v_valid = v_coords[valid_mask]
                            
                            # Pinhole Camera Model projection to 3D Cartesian coordinates
                            x = (u_valid - args.cx) * z_valid / args.fx
                            y = (v_valid - args.cy) * z_valid / args.fy
                            
                            # Open3D coordinate system adjustment:
                            # In standard computer vision, Z is forward, X is right, Y is down.
                            # We invert Y and Z so it displays naturally in Open3D camera view.
                            points = np.column_stack((x, -y, -z_valid))
                            
                            # Heatmap color scale: Red (Near) -> Blue (Far)
                            norm_span = max(0.1, args.max_range - args.min_range)
                            z_norm = np.clip((z_valid - args.min_range) / norm_span, 0.0, 1.0)
                            colors = np.column_stack(((1.0 - z_norm), np.zeros_like(z_norm), z_norm))
                            
                            pcd.points = o3d.utility.Vector3dVector(points)
                            pcd.colors = o3d.utility.Vector3dVector(colors)
                            
                            if not point_cloud_added:
                                vis.add_geometry(pcd)
                                point_cloud_added = True
                            else:
                                vis.update_geometry(pcd)
            except socket.timeout:
                pass
                
            if not vis.poll_events():
                print("\nVisualizer closed by user.")
                break
            vis.update_renderer()
            
        except KeyboardInterrupt:
            print("\nShutting down 3D viewer...")
            break
        except Exception as e:
            print(f"[Error] {e}")
            
    sock.close()
    vis.destroy_window()

if __name__ == "__main__":
    main()
