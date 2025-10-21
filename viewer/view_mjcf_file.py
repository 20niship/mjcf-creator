#!/usr/bin/env python3
"""
MJCF File Viewer using MuJoCo

This script provides an interactive viewer for MJCF files with camera control and simulation.
"""

import sys
import os
import argparse
from pathlib import Path

try:
    import mujoco
    import mujoco.viewer
    MUJOCO_AVAILABLE = True
except ImportError:
    print("Error: MuJoCo Python package not available.")
    print("Install with: pip install mujoco")
    print("Or use uv: uv pip install mujoco")
    MUJOCO_AVAILABLE = False
    sys.exit(1)


def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="View MJCF files with MuJoCo interactive viewer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s model.xml
  %(prog)s path/to/scene.mjcf
  %(prog)s ../examples/output/simple_boxes_scene.xml

Controls:
  - Left mouse button: Rotate view
  - Right mouse button: Move view
  - Scroll wheel: Zoom
  - Double-click: Select body
  - Ctrl+P: Pause/Resume simulation
  - Backspace: Reset simulation
  - Esc: Exit viewer
        """
    )
    parser.add_argument(
        "mjcf_file",
        type=str,
        help="Path to MJCF/XML file to visualize"
    )
    parser.add_argument(
        "--no-simulate",
        action="store_true",
        help="Start with simulation paused"
    )
    parser.add_argument(
        "--timestep",
        type=float,
        default=None,
        help="Override simulation timestep (seconds)"
    )
    
    return parser.parse_args()


def load_model(mjcf_path):
    """
    Load MJCF model from file.
    
    Args:
        mjcf_path: Path to MJCF file
        
    Returns:
        MuJoCo model object
        
    Raises:
        FileNotFoundError: If file doesn't exist
        Exception: If model loading fails
    """
    mjcf_path = Path(mjcf_path)
    
    if not mjcf_path.exists():
        raise FileNotFoundError(f"MJCF file not found: {mjcf_path}")
    
    if not mjcf_path.is_file():
        raise ValueError(f"Path is not a file: {mjcf_path}")
    
    # Check file extension
    valid_extensions = ['.xml', '.mjcf']
    if mjcf_path.suffix.lower() not in valid_extensions:
        print(f"Warning: File extension '{mjcf_path.suffix}' may not be valid. Expected: {valid_extensions}")
    
    print(f"Loading MJCF model from: {mjcf_path}")
    
    try:
        # Load the model
        model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        print(f"✓ Model loaded successfully")
        print(f"  - Bodies: {model.nbody}")
        print(f"  - Geoms: {model.ngeom}")
        print(f"  - Joints: {model.njnt}")
        print(f"  - Actuators: {model.nu}")
        print(f"  - Timestep: {model.opt.timestep}s")
        
        return model
        
    except Exception as e:
        print(f"✗ Failed to load model: {e}")
        raise


def main():
    """Main entry point."""
    if not MUJOCO_AVAILABLE:
        return 1
    
    args = parse_arguments()
    
    try:
        # Load the model
        model = load_model(args.mjcf_file)
        
        # Override timestep if requested
        if args.timestep is not None:
            model.opt.timestep = args.timestep
            print(f"Timestep overridden to: {args.timestep}s")
        
        # Create simulation data
        data = mujoco.MjData(model)
        
        print("\n" + "="*60)
        print("Starting MuJoCo Viewer")
        print("="*60)
        print("\nViewer Controls:")
        print("  Left mouse: Rotate view")
        print("  Right mouse: Move view")
        print("  Scroll wheel: Zoom in/out")
        print("  Double-click: Select body")
        print("  Ctrl+P: Pause/Resume simulation")
        print("  Backspace: Reset simulation")
        print("  Esc or Ctrl+Q: Exit viewer")
        print("\nCamera Selection:")
        print("  Use '[' and ']' keys to switch between cameras")
        print("  Press '0' for free camera")
        print("="*60 + "\n")
        
        # Launch the interactive viewer
        # The viewer will run until the user closes it
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set initial simulation state
            if args.no_simulate:
                viewer.is_running = False
                print("Simulation started in paused mode")
            
            # Main simulation loop
            while viewer.is_running():
                # Advance the simulation
                mujoco.mj_step(model, data)
                
                # Sync the viewer with the simulation state
                viewer.sync()
        
        print("\nViewer closed.")
        return 0
        
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
