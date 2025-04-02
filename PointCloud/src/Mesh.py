import open3d as o3d
import os
import numpy as np
import multiprocessing
import sys



def processMesh(arr):


    array = np.array(arr)

    print(array[:,-3:])

    # if __name__ == '__main__':
    print("Current Working Directory:", os.getcwd())
    cloud = o3d.geometry.PointCloud()

    cloud.points = o3d.utility.Vector3dVector(array[:,:3])
    cloud.colors = o3d.utility.Vector3dVector(array[:,-3:]) #todo, extract rgb values from the float

    # cloud = o3d.io.read_point_cloud("PointCloud\\build\\Filtered_Pointcloud.ply", remove_nan_points=True, remove_infinite_points=True)


    if cloud.is_empty():
        print("Load Failed")
    else:
        print("Load Success")

        cloud_tree = o3d.geometry.KDTreeFlann(cloud) 
        distances = []

        for point in cloud.points:
            _, idx, dists = cloud_tree.search_knn_vector_3d(point, knn=2)  # knn=2 to get the first neighbor
            distances.append(np.sqrt(dists[1]))  # Distance to nearest neighbor

        avg_spacing = np.mean(distances) # Gets average spacing between points in the mesh

        print("Obtained Spacing")

        cloud = cloud.voxel_down_sample(voxel_size = avg_spacing*2)

        # Code for running normal gen in parallel
        # cloud = parallel_normal_estimation(cloud, avg_spacing)
        cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=avg_spacing*3, max_nn=30)) # Creates normals for mesh based on average spacing
        
        print("Normals Success")

        # cloud.orient_normals_to_align_with_direction([0,0,1])
        # cloud.orient_normals_towards_camera_location([0,0,10])
        cloud.orient_normals_consistent_tangent_plane(k=5) # Ensures all normals are facing the same direction
        
        print("Orient Success")

        # centroid = np.mean(np.asarray(cloud.points), axis = 0)

        # normals = np.asarray(cloud.normals)
        # vectors_to_centroid = np.asarray(cloud.points) - centroid
        # dot_prod = np.einsum("ij,ij->i", normals, vectors_to_centroid)

        # if np.mean(dot_prod) < 0 :
        #     cloud.normals = o3d.utility.Vector3dVector(-normals)

        # print("Centroid Success")


        radii = o3d.utility.DoubleVector([avg_spacing * 3, avg_spacing * 3.5, avg_spacing * 4, avg_spacing*4.5, avg_spacing*5]) 

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
                # o3d.visualization.draw_geometries([mesh], window_name="Gen Mesh") # Open3D Mesh Visualizer, remove in release
            else:
                print("Mesh has no triangles")



# Used for parallelization, currently not working
def estimate_normals_chunk(chunk, avg_spacing):
    print("Entered Estimate Norms")

    temp_cloud = o3d.geometry.PointCloud()
    temp_cloud.points = o3d.utility.Vector3dVector(chunk)

    print(temp_cloud)

    temp_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=avg_spacing*3, max_nn=30))

    temp_cloud.orient_normals_consistent_tangent_plane(k=5)

    return np.asarray(temp_cloud.normals)

def parallel_normal_estimation(cloud, avg_spacing, chunk_size = 10000):
    print("Entered Parallel")

    num_chunks = len(cloud.points) // chunk_size + 1
    chunks = []
    for i in range(num_chunks):
        if (i + 1) * chunk_size > len(cloud.points):
            chunks.append(np.asarray(cloud.points)[i * chunk_size: len(cloud.points)])
        else:
            chunks.append(np.asarray(cloud.points)[i * chunk_size: (i + 1) * chunk_size])

    with multiprocessing.Pool(processes=multiprocessing.cpu_count()) as pool:
        results = pool.starmap(estimate_normals_chunk, [(chunk, avg_spacing) for chunk in chunks])

    all_normals = np.vstack(results)
    cloud.normals = o3d.utility.Vector3dVector(all_normals)
    return cloud


