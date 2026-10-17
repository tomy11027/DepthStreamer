import socket
import struct
import argparse
import numpy as np
import cv2

def parse_args():
    parser = argparse.ArgumentParser(
        description="DepthStreamer 2D Visualizer - Stream raw ToF laser depth from Android to PC over UDP."
    )
    parser.add_argument(
        "--ip",
        type=str,
        default="0.0.0.0",
        help="UDP IP to bind and listen on (default: 0.0.0.0, all interfaces)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5005,
        help="UDP port to listen on (default: 5005)",
    )
    parser.add_argument(
        "--max-depth",
        type=float,
        default=3000.0,
        help="Maximum depth distance in mm for stable heatmap colormap scaling (default: 3000 mm)",
    )
    parser.add_argument(
        "--scale",
        type=int,
        default=4,
        help="Upscale factor for display window (default: 4)",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress printing raw distance matrices in terminal",
    )
    return parser.parse_args()

def main():
    args = parse_args()
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    
    print("=" * 60)
    print(f"[*] DepthStreamer Receiver listening on {args.ip}:{args.port}")
    print(f"    Max Depth Range: {args.max_depth:.0f} mm")
    print(f"    Window Upscale:  {args.scale}x")
    print("    Press 'q' in the OpenCV window or Ctrl+C in terminal to exit.")
    print("=" * 60)
    
    # Pre-allocate buffer for the largest possible UDP packet
    BUFFER_SIZE = 65535
    
    # We accumulate chunks to form a full frame
    frame_buffer = None
    confidence_buffer = None
    
    while True:
        try:
            # Receive UDP packet
            data, addr = sock.recvfrom(BUFFER_SIZE)
            
            # A valid packet must at least have 16 bytes for header
            if len(data) < 16:
                continue
                
            # Parse header (Little Endian, 32-bit unsigned integers)
            width, height, y_offset, y_count = struct.unpack('<IIII', data[:16])
            
            expected_data_size = width * y_count * 2
            if len(data) - 16 != expected_data_size:
                print(f"[Warning] Size mismatch for chunk: expected {expected_data_size}, got {len(data)-16}")
                continue
                
            # Initialize persistent buffer if it doesn't exist or size changed
            if frame_buffer is None or frame_buffer.shape != (height, width):
                frame_buffer = np.zeros((height, width), dtype=np.uint16)
                confidence_buffer = np.zeros((height, width), dtype=np.uint16)
                print(f"[*] Initialized depth frame buffer: {width}x{height} (DEPTH16)")
                
            # Parse chunk depth pixels
            depth_array_raw = np.frombuffer(data, dtype=np.uint16, offset=16)
            
            # Extract distance (lower 13 bits in mm) and confidence (upper 3 bits: 0-7)
            distance_array = np.bitwise_and(depth_array_raw, 0x1FFF).reshape((y_count, width))
            confidence_array = np.right_shift(depth_array_raw, 13).reshape((y_count, width))
            
            # Write chunk to persistent full frame buffer
            frame_buffer[y_offset:y_offset+y_count, :] = distance_array
            confidence_buffer[y_offset:y_offset+y_count, :] = confidence_array
            
            # ----------------------------------------------------
            # Visualizing the Depth Data (Heatmap / Night Vision)
            # ----------------------------------------------------
            valid_mask = frame_buffer > 0
            
            normalized_depth = np.zeros_like(frame_buffer, dtype=np.uint8)
            color_depth = np.zeros((height, width, 3), dtype=np.uint8)
            
            if np.any(valid_mask):
                # Clip to stable max depth
                clipped = np.clip(frame_buffer[valid_mask], 0, args.max_depth)
                
                # Scale to 0-255
                scaled = (clipped / args.max_depth * 255).astype(np.uint8)
                
                # Invert scale: Near = Red (255 in JET), Far = Blue (0 in JET)
                normalized_depth[valid_mask] = 255 - scaled
                
                # Apply JET colormap
                color_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)
                
                # Invalid pixels (0 distance / out of range) remain black
                color_depth[~valid_mask] = [0, 0, 0]
            
            # Android physical camera sensors are mounted in landscape.
            # Rotate 90 degrees clockwise to match portrait phone orientation.
            color_depth = cv2.rotate(color_depth, cv2.ROTATE_90_CLOCKWISE)
            
            # Center pixel metrics
            center_x, center_y = width // 2, height // 2
            center_mm = frame_buffer[center_y, center_x]
            center_conf = confidence_buffer[center_y, center_x]
            
            # Print frame details when bottom chunk arrives
            if (y_offset + y_count >= height) and not args.quiet:
                print(f"Frame received: Center Distance: {center_mm:4d} mm | Confidence: {center_conf}/7")
            
            # Draw telemetry on displayed image
            cv2.putText(
                color_depth,
                f"Center: {center_mm} mm (Conf: {center_conf}/7)",
                (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                color_depth,
                f"Range: 0 - {int(args.max_depth)} mm",
                (8, 38),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (200, 200, 200),
                1,
                cv2.LINE_AA,
            )
            
            # Scale up for display
            display_w = height * args.scale
            display_h = width * args.scale
            scaled_depth = cv2.resize(color_depth, (display_w, display_h), interpolation=cv2.INTER_NEAREST)
            
            cv2.imshow("DepthStreamer - Android ToF Depth Map", scaled_depth)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\nExiting per user request...")
                break
                
        except KeyboardInterrupt:
            print("\nShutting down receiver...")
            break
        except Exception as e:
            print(f"[Error] Processing packet: {e}")
            
    sock.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
