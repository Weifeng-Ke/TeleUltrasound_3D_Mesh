import open3d as o3d
import os
import numpy as np

# cloud = o3d.io.read_point_cloud("[pointcloudname].ply") #TO-DO Add way to read pointcloud files automatically

cloud = o3d.io.read_point_cloud("Pointcloud_top_down.ply")

if cloud.is_empty():
    print("Load Failed")
else:
    print("Load Success")

    cloud_tree = o3d.geometry.KDTreeFlann(cloud) #
    distances = []

    for point in cloud.points:
        _, idx, dists = cloud_tree.search_knn_vector_3d(point, knn=2)  # knn=2 to get the first neighbor
        distances.append(np.sqrt(dists[1]))  # Distance to nearest neighbor

    avg_spacing = np.mean(distances) # Gets average spacing between points in the mesh

    cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=avg_spacing*3, max_nn=30)) # Creates normals for mesh based on average spacing
    
    print("Normal Success")
    
    cloud.orient_normals_consistent_tangent_plane(k=5) # Ensures all normals are facing the same direction
    
    print("orient success")


    # Tries to find the center of all the points and mark it as the "interior"
    # Aligns all normals so that they face away from the centroid
    # **Note** Unreliable due to noise/random points, the issue of flipped normals might have to be fixed by rendering them double sided
    centroid = np.mean(np.asarray(cloud.points), axis = 0)

    normals = np.asarray(cloud.normals)
    vectors_to_centroid = np.asarray(cloud.points) - centroid
    dot_products = np.einsum("ij,ij->i", normals, vectors_to_centroid)

    if np.mean(dot_products) < 0:
        cloud.normals = o3d.utility.Vector3dVector(-normals)


    print("centroid success")

    # From testing, seems to be the best set of radii for minimizing holes. More radii has minimal effect on mesh creation speed. 
    radii = o3d.utility.DoubleVector([avg_spacing * 3, avg_spacing * 3.5, avg_spacing *4, avg_spacing * 4.5, avg_spacing * 5]) #, avg_spacing * 5.5, avg_spacing *6, avg_spacing * 20])

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(cloud, o3d.utility.DoubleVector(radii))

    print("Beginning Check Mesh")

    if mesh.is_empty():
        print("Empty Mesh")
    else:
        if len(mesh.triangles) != 0:
            success = o3d.io.write_triangle_mesh("generated_mesh.ply", mesh)  # To-Do: Determine where to output mesh files
            if success:
                print("Mesh Saved")
            else:
                print("Failed to save")
            o3d.visualization.draw_geometries([mesh], window_name="Gen Mesh") # Open3D Mesh Visualizer, remove in release
        else:
            print("Mesh has no triangles")
    