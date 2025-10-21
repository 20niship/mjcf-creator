#!/usr/bin/env python3
"""
Script to validate MJCF files by loading them with MuJoCo.
This script will attempt to load all MJCF files in a directory and report any errors.
"""

import os
import sys
import argparse
import glob
from pathlib import Path

try:
    import mujoco

    MUJOCO_AVAILABLE = True
except ImportError:
    print(
        "Warning: MuJoCo Python package not available. Install with 'pip install mujoco'"
    )
    MUJOCO_AVAILABLE = False


def validate_mjcf_file(filepath):
    """
    Validate a single MJCF file by attempting to load it with MuJoCo.

    Args:
        filepath: Path to the MJCF file

    Returns:
        tuple: (success: bool, error_message: str or None)
    """
    if not MUJOCO_AVAILABLE:
        return False, "MuJoCo not available"

    try:
        # Attempt to load the model
        model = mujoco.MjModel.from_xml_path(str(filepath))

        # Create a data object to test simulation readiness
        data = mujoco.MjData(model)

        # Perform a basic simulation step to ensure the model is valid
        mujoco.mj_step(model, data)

        return True, None

    except Exception as e:
        return False, str(e)


def validate_directory(directory):
    """
    Validate all MJCF files in a directory.

    Args:
        directory: Path to directory containing MJCF files

    Returns:
        dict: Results of validation {filename: (success, error)}
    """
    results = {}

    # Find all XML files in the directory
    xml_files = glob.glob(os.path.join(directory, "*.xml"))

    if not xml_files:
        print(f"No XML files found in {directory}")
        return results

    print(f"Found {len(xml_files)} XML files to validate...")

    for filepath in xml_files:
        filename = os.path.basename(filepath)
        print(f"Validating {filename}...", end=" ")

        success, error = validate_mjcf_file(filepath)
        results[filename] = (success, error)

        if success:
            print("✓ VALID")
        else:
            print(f"✗ INVALID: {error}")

    return results


def print_summary(results):
    """Print a summary of validation results."""
    if not results:
        print("No files were validated.")
        return

    total = len(results)
    valid = sum(1 for success, _ in results.values() if success)
    invalid = total - valid

    print(f"\nValidation Summary:")
    print(f"==================")
    print(f"Total files: {total}")
    print(f"Valid files: {valid}")
    print(f"Invalid files: {invalid}")

    if invalid > 0:
        print(f"\nInvalid files:")
        for filename, (success, error) in results.items():
            if not success:
                print(f"  - {filename}: {error}")

    return invalid == 0


def main():
    parser = argparse.ArgumentParser(description="Validate MJCF files using MuJoCo")
    parser.add_argument(
        "directory",
        nargs="?",
        default=".",
        help="Directory containing MJCF files (default: current directory)",
    )
    parser.add_argument(
        "--recursive",
        "-r",
        action="store_true",
        help="Search recursively in subdirectories",
    )

    args = parser.parse_args()

    if not os.path.exists(args.directory):
        print(f"Error: Directory '{args.directory}' does not exist")
        sys.exit(1)

    if not MUJOCO_AVAILABLE:
        print("Error: MuJoCo Python package is required for validation")
        print("Install with: pip install mujoco")
        sys.exit(1)

    print(f"MJCF File Validation")
    print(f"===================")
    print(f"Directory: {args.directory}")
    print(f"MuJoCo version: {mujoco.mj_versionString()}")
    print()

    if args.recursive:
        # Find all XML files recursively
        results = {}
        for root, dirs, files in os.walk(args.directory):
            xml_files = [f for f in files if f.endswith(".xml")]
            if xml_files:
                print(f"Validating files in {root}:")
                dir_results = validate_directory(root)
                results.update(dir_results)
                print()
    else:
        results = validate_directory(args.directory)

    success = print_summary(results)

    # Exit with error code if any files failed validation
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

