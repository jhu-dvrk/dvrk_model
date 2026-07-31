import trimesh
import numpy as np
import sys
import os

def analyze_mesh(file_path):
    print(f"Analyzing {file_path}...")
    loaded = trimesh.load(file_path)
    
    # Handle Scene objects (some OBJs load as scenes)
    if isinstance(loaded, trimesh.Scene):
        if len(loaded.geometry) == 0:
            print("  Error: Scene is empty")
            return
        # Merge all meshes in the scene into one for analysis
        mesh = trimesh.util.concatenate([g for g in loaded.geometry.values() if isinstance(g, trimesh.Trimesh)])
    else:
        mesh = loaded

    # 1. Basic properties
    verts = mesh.vertices
    print(f"  Faces: {len(mesh.faces)}")
    print(f"  Bounding Box: {mesh.bounds}")
    print(f"  Centroid: {mesh.centroid}")
    
    # 2. Try to find cylindrical primitive
    print("\nAttempting to find primitives...")
    # Attempting to use decomposition to find cylinders
    try:
        # Mesh decomposition can split complex shapes into primitives
        # Parts with cylindrical/other shapes
        prim = trimesh.decomposition.protrusions(mesh)
        print(f"  Decomposition found {len(prim)} protrusion(s).")
    except Exception as e:
        print(f"  Primitive detection failed: {e}")

    # 3. Manual Extraction via Face Clusters (e.g. cylinder walls)
    # The user says "centered and orthogonal to the axis"
    # Find cylindrical walls by checking for constant distance to one of the main axes
    # (assuming the holes are roughly along X or Y, with the tool shaft along Z)
    
    # Analyze point cloud (vertices)
    verts = mesh.vertices
    # Check for constant radius around some axis
    # (x^2 + y^2 = r^2 for axis Z)
    # (x^2 + z^2 = r^2 for axis Y)
    # (y^2 + z^2 = r^2 for axis X)
    
    axes = ['X', 'Y', 'Z']
    for i, axis in enumerate(axes):
        # other indices
        idx1 = (i + 1) % 3
        idx2 = (i + 2) % 3
        
        radii = np.sqrt(verts[:, idx1]**2 + verts[:, idx2]**2)
        # Check standard deviation of radii - a small deviation suggests a cylindrical segment
        # (This is a simplification, but helpful for 'centered' holes)
        
        # We look for a *peak* in the radius histogram
        hist, bins = np.histogram(radii, bins=100)
        peak_idx = np.argmax(hist)
        peak_radius = bins[peak_idx]
        
        # If many points are at similar radius, it could be a cylinder centered at the origin
        if hist[peak_idx] > len(verts) * 0.1: # Threshold: 10% of vertices at same radius
             # Calculate variance for those points
             mask = np.abs(radii - peak_radius) < (bins[1]-bins[0])
             radius_std = np.std(radii[mask])
             print(f"  Axis {axis}: Found potential cylinder at R={peak_radius:.6f} with {hist[peak_idx]} vertices (std={radius_std:.6e})")

    # 4. Group faces into areas of similar normal (potential cylinder walls)
    print("\n[CYLINDER DETECTION]")
    try:
         # trimesh triangles_center and face_normals are useful here
         
         for axis_idx, axis_name in enumerate(['X', 'Y', 'Z']):
             axis_vec = np.zeros(3)
             axis_vec[axis_idx] = 1
             
             # Dot product with axis; if perpendicular, the absolute dot product is ~0.
             # (A cylindrical wall around Z has normals with Z-component near 0).
             perpendicular_mask = np.abs(mesh.face_normals[:, axis_idx]) < 0.1
             face_centers = mesh.triangles_center[perpendicular_mask]
             
             # Compute radii relative to the axis
             i1, i2 = (axis_idx + 1) % 3, (axis_idx + 2) % 3
             rads = np.sqrt(face_centers[:, i1]**2 + face_centers[:, i2]**2)

             if len(rads) > 0:
                 # consensus radius via histogram
                 hist, bins = np.histogram(rads, bins=100)
                 # Find peaks that have significant face counts
                 peaks = np.where(hist > 20)[0]
                 
                 for peak in peaks:
                     consensus_rad = bins[peak]
                     consensus_mask = np.abs(rads - consensus_rad) < (bins[1]-bins[0])
                     
                     final_points = face_centers[consensus_mask]
                     if len(final_points) > 20: 
                         other_dims_mean = np.mean(final_points[:, [i1, i2]], axis=0)
                         z_coord = final_points[:, axis_idx]
                         z_range = [np.min(z_coord), np.max(z_coord)]
                         z_center = np.mean(z_range)
                         print(f"  Axis {axis_name}: FOUND cylinder wall at R={consensus_rad:.6f}")
                         print(f"    Center (other dims): {other_dims_mean}")
                         print(f"    Center along Axis ({axis_name}): {z_center:.6f}")
                         print(f"    Length along Axis: {np.max(z_coord) - np.min(z_coord):.6f}")

    except Exception as e:
        print(f"  Detection error: {e}")

    # 4. Use trimesh's 'primitive' attribute if available
    # Sometimes loading tries to identify them
    if hasattr(mesh, 'primitive'):
        print(f"  Detected primitive type: {type(mesh.primitive)}")

    # 5. Manual Cylinder Detection via Face Grouping
    # Split the mesh into connected components of similar curvature/normals
    # This is a bit complex for a quick script but trimesh has some tools for it.
    
    # Let's try to find the "largest" hole or cylinder by looking at normal distributions
    # Most surgical instruments have holes along X, Y, or Z.
    normals = mesh.face_normals
    unique_normals, counts = np.unique(np.round(normals, 2), axis=0, return_counts=True)
    # print(f"Top 5 normals: {unique_normals[np.argsort(-counts)[:5]]}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python extract_geometry.py <path_to_obj>")
        return
    
    analyze_mesh(sys.argv[1])

if __name__ == "__main__":
    main()