import open3d as o3d
import os
import numpy as np

cloud = o3d.io.read_point_cloud("[pointcloudname].ply") #TO-DO Add way to read pointcloud files automatically

if cloud.is_empty():
    print("Load Failed")
else:
    print("Load Success")

    cloud_tree = o3d.geometry.KDTreeFlann(cloud)
    distances = []

    for point in cloud.points:
        _, idx, dists = cloud_tree.search_knn_vector_3d(point, knn=2)  # knn=2 to get the first neighbor
        distances.append(np.sqrt(dists[1]))  # Distance to nearest neighbor

    avg_spacing = np.mean(distances)

    cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=avg_spacing*3, max_nn=30))
    cloud.orient_normals_consistent_tangent_plane(k=100)
    print("Normal Success")

    radii = o3d.utility.DoubleVector([avg_spacing * 2, avg_spacing * 2.5, avg_spacing * 3])

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(cloud, o3d.utility.DoubleVector(radii))

    print("Beginning Check Mesh")

    if mesh.is_empty():
        print("Empty Mesh")
    else:
        if len(mesh.triangles) != 0:
            success = o3d.io.write_triangle_mesh("generated_mesh.ply", mesh)
            if success:
                print("Mesh Saved")
            else:
                print("Failed to save")
            o3d.visualization.draw_geometries([mesh], window_name="Gen Mesh")
        else:
            print("Mesh has no triangles")
    