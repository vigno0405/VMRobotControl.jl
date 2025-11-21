#!/usr/bin/env python3
"""
Merge original URDF with Gazebo additions
Usage: python3 merge_urdf_gazebo.py input.urdf output.urdf
"""

import sys
import os

def merge_urdf(input_file, gazebo_additions_file, output_file):
    """
    Merge original URDF with Gazebo additions
    """
    # Read original URDF
    with open(input_file, 'r') as f:
        urdf_content = f.read()
    
    # Read Gazebo additions
    with open(gazebo_additions_file, 'r') as f:
        gazebo_content = f.read()
    
    # Check if URDF ends with </robot>
    if not urdf_content.strip().endswith('</robot>'):
        print("ERROR: Input URDF doesn't end with </robot> tag")
        return False
    
    # Remove closing </robot> tag
    urdf_content = urdf_content.rsplit('</robot>', 1)[0]
    
    # Merge
    merged_content = urdf_content + gazebo_content
    
    # Write output
    with open(output_file, 'w') as f:
        f.write(merged_content)
    
    print(f"Successfully merged URDF to: {output_file}")
    print(f"Original size: {len(urdf_content)} bytes")
    print(f"Gazebo additions: {len(gazebo_content)} bytes")
    print(f"Merged size: {len(merged_content)} bytes")
    
    return True

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 merge_urdf_gazebo.py <input_urdf> <output_urdf>")
        print("   Gazebo additions will be read from gazebo_additions.xml")
        sys.argv.exit(1)
    
    input_urdf = sys.argv[1]
    output_urdf = sys.argv[2]
    gazebo_additions = "gazebo_additions.xml"
    
    # Check files exist
    if not os.path.exists(input_urdf):
        print(f"ERROR: Input URDF not found: {input_urdf}")
        sys.exit(1)
    
    if not os.path.exists(gazebo_additions):
        print(f"ERROR: Gazebo additions file not found: {gazebo_additions}")
        print("Run create_gazebo_urdf.py first to generate it")
        sys.exit(1)
    
    # Merge
    success = merge_urdf(input_urdf, gazebo_additions, output_urdf)
    
    if success:
        print("\nNext steps:")
        print("1. Review the generated URDF")
        print("2. Update mesh file paths if needed")
        print("3. Place in your ROS 2 workspace")
        print("4. Update launch file with correct path")
        print("5. Build and launch!")
    else:
        sys.exit(1)