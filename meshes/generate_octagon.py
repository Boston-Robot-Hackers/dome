#!/usr/bin/env python3
"""
Generate an octagonal plate mesh in STL format for dome2 robot.
"""
import math
import struct

def write_stl_binary(filename, triangles):
    """Write triangles to binary STL file."""
    with open(filename, 'wb') as f:
        # Header (80 bytes)
        f.write(b' ' * 80)
        # Number of triangles
        f.write(struct.pack('<I', len(triangles)))

        # Write each triangle
        for triangle in triangles:
            normal, vertices = triangle
            # Normal vector
            f.write(struct.pack('<fff', *normal))
            # Three vertices
            for vertex in vertices:
                f.write(struct.pack('<fff', *vertex))
            # Attribute byte count
            f.write(struct.pack('<H', 0))

def generate_octagon_plate(diameter, thickness):
    """
    Generate an octagonal plate mesh.

    Args:
        diameter: Diameter of the octagon (inscribed circle)
        thickness: Thickness of the plate

    Returns:
        List of triangles, each as (normal, [v1, v2, v3])
    """
    radius = diameter / 2.0
    half_thickness = thickness / 2.0

    # Generate octagon vertices (8 points around a circle)
    num_sides = 8
    top_vertices = []
    bottom_vertices = []

    for i in range(num_sides):
        angle = 2 * math.pi * i / num_sides
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        top_vertices.append((x, y, half_thickness))
        bottom_vertices.append((x, y, -half_thickness))

    triangles = []

    # Top face (octagon divided into triangles from center)
    center_top = (0, 0, half_thickness)
    normal_top = (0, 0, 1)
    for i in range(num_sides):
        next_i = (i + 1) % num_sides
        triangles.append((normal_top, [center_top, top_vertices[i], top_vertices[next_i]]))

    # Bottom face (octagon divided into triangles from center)
    center_bottom = (0, 0, -half_thickness)
    normal_bottom = (0, 0, -1)
    for i in range(num_sides):
        next_i = (i + 1) % num_sides
        # Reverse winding order for bottom face
        triangles.append((normal_bottom, [center_bottom, bottom_vertices[next_i], bottom_vertices[i]]))

    # Side faces (8 rectangular faces, each made of 2 triangles)
    for i in range(num_sides):
        next_i = (i + 1) % num_sides

        # Calculate outward normal for this side
        angle = 2 * math.pi * (i + 0.5) / num_sides
        normal = (math.cos(angle), math.sin(angle), 0)

        # Two triangles per side face
        v1 = bottom_vertices[i]
        v2 = top_vertices[i]
        v3 = top_vertices[next_i]
        v4 = bottom_vertices[next_i]

        triangles.append((normal, [v1, v2, v3]))
        triangles.append((normal, [v1, v3, v4]))

    return triangles

def main():
    # Parameters for dome2 octagonal plate
    diameter = 0.300  # 30cm
    thickness = 0.003  # 3mm

    print(f"Generating octagonal plate mesh:")
    print(f"  Diameter: {diameter}m")
    print(f"  Thickness: {thickness}m")

    triangles = generate_octagon_plate(diameter, thickness)

    output_file = "octagon_plate.stl"
    write_stl_binary(output_file, triangles)

    print(f"Generated {output_file} with {len(triangles)} triangles")

if __name__ == "__main__":
    main()
