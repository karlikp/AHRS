import os.path
import sys
import time

import numpy as np
from pypointmatcher import pointmatcher as pm
from sklearn.neighbors import NearestNeighbors

sys.path.append(os.path.join(os.path.dirname(__file__), 'lib'))
from lib.libpointmatcher_original.examples.python.utils import parse_rotation, parse_translation

sys.path.append(os.path.join(os.path.dirname(__file__), 'module'))
from module.IMU_calibrator import IMU_calibrator

def calculate_normals(points, k=10):
    """
    Oblicza normalne dla chmury punktów 3D.

    :param points: numpy array o wymiarach (n, 3) - chmura punktów (x, y, z)
    :param k: liczba najbliższych sąsiadów
    :return: lista punktów z normalnymi (x, y, z, nx, ny, nz)
    """
    # Tworzenie KD-Tree do wyszukiwania najbliższych sąsiadów
    neighbors = NearestNeighbors(n_neighbors=k).fit(points)
    normals = []

    for i, point in enumerate(points):
        # Znajdź sąsiadów punktu
        distances, indices = neighbors.kneighbors([point])
        neighbors_points = points[indices[0]]

        # Oblicz PCA (rozklad SVD)
        centroid = np.mean(neighbors_points, axis=0)
        centered_points = neighbors_points - centroid
        _, _, vh = np.linalg.svd(centered_points, full_matrices=False)

        # Wektor normalny to ostatni wektor własny (najmniejsza wartość własna)
        normal = vh[-1]
        
        # Orientacja normalnych (opcjonalne): skierowanie w stronę sensora
        if np.dot(normal, point - centroid) < 0:
            normal = -normal

        # Dodanie normalnych do punktu
        normals.append([point[0], point[1], point[2], normal[0], normal[1], normal[2]])

    return normals

def synchronize_point_cloud_lengths(formatted_ref, formatted_data):
    """
    Synchronizuje długości dwóch chmur punktów, usuwając nadmiarowe elementy z dłuższej listy.
    
    :param formatted_ref: numpy array - chmura punktów referencyjnych
    :param formatted_data: numpy array - chmura punktów do dopasowania
    :return: tuple (numpy array, numpy array) - zsynchronizowane chmury punktów
    """
    len_ref = len(formatted_ref)
    len_data = len(formatted_data)

    # Jeśli długości są takie same, zwróć bez zmian
    if len_ref == len_data:
        return formatted_ref, formatted_data

    # Usuń nadmiarowe elementy z dłuższej listy
    if len_ref > len_data:
        formatted_ref = formatted_ref[:len_data]
    else:
        formatted_data = formatted_data[:len_ref]

    return formatted_ref, formatted_data

def icp_process(data_manager):
    
    #static calibration for IMU
    imu_calibrator = IMU_calibrator()
    imu_calibrator.calibrate_imu_once(data_manager)
    
    # Code example for ICP taking 2 points clouds (2D or 3D) relatively close
    # and computing the transformation between them.
    #
    # This code is more complete than icp_simple. It can load parameter files and
    # has more options.


    #from pypointmatcher import pointmatcher as pm
    #from utils import parse_translation, parse_rotation

    PM = pm.PointMatcher
    DP = PM.DataPoints
    
    raw_ref = data_manager.get_current_cloud()
    formatted_ref = np.array(raw_ref)
    formatted_ref = formatted_ref[:2050]
    
    # Dodaj wiersz jedynek dla współrzędnych homogenicznych
    features_raw = np.vstack((formatted_ref.T, np.ones((1, formatted_ref.shape[0]), dtype=np.float32)))

    # Zdefiniuj etykiety cech
    feature_labels_raw = [  
        PM.DataPoints.Label("x", 1),
        PM.DataPoints.Label("y", 1),
        PM.DataPoints.Label("z", 1),
        PM.DataPoints.Label("pad", 1)
        ]       
    
    feature_labels = PM.DataPoints.Labels(feature_labels_raw)
    # Stwórz obiekt DataPoints
    ref = DP(features_raw, feature_labels)

    # Save transformation matrix in three different files:
    #     - BASEFILENAME_inti_transfo.txt
    #     - BASEFILENAME_icp_transfo.txt
    #     - BASEFILENAME_complete_transfo.txt
    #       (default: false)
    is_transfo_saved = False

    # Be more verbose (info logging to the console)
    is_verbose = True

    # Load the config from a YAML file (default: default.yaml)
    # Leave empty to set the ICP default configuration
    config_file = "../data/default.yaml"

    # Path of output directory (default: tests/icp/)
    # The output directory must already exist
    # Leave empty to save in the current directory
    output_base_directory = "tests/icp/"

    # Name of output files (default: test)
    output_base_file = "test"

    # Toggle to switch between 2D and 3D point clouds
    is_3D = True

    # Add an initial 3D translation before applying ICP (default: 0,0,0)
    # Add an initial 2D translation before applying ICP (default: 0,0)
    init_translation = "0,0,0" if is_3D else "0,0"
    # Add an initial 3D rotation before applying ICP (default: 1,0,0;0,1,0;0,0,1)
    # Add an initial 2D rotation before applying ICP (default: 1,0;0,1)
    init_rotation = "1,0,0;0,1,0;0,0,1" if is_3D else "1,0;0,1"
    
    time.sleep(0.5)

    while True:
        
        raw_cloud = data_manager.get_current_cloud()
        np_raw_cloud = np.array(raw_cloud)
        
        # formatted_data = np.array(calculate_normals(np_raw_cloud, k=10)) 
        formatted_data = np.array(np_raw_cloud) 
        
        ref, formatted_data = synchronize_point_cloud_lengths(ref, formatted_data)
        
        print(f"ref amount: {len(ref)}")
        print(f"data amoung: {len(formatted_data)}")
        # Dodaj wiersz jedynek dla współrzędnych homogenicznych
        features_data = np.vstack((formatted_data.T, np.ones((1, formatted_ref.shape[0]), dtype=np.float32)))
        
        feature_labels_data = [  
        PM.DataPoints.Label("x", 1),
        PM.DataPoints.Label("y", 1),
        PM.DataPoints.Label("z", 1),
        PM.DataPoints.Label("pad", 1)
        ]       
    
        feature_labels = PM.DataPoints.Labels(feature_labels_data)
    
        data = DP(features_data, feature_labels)
        
        test_base = "3D"
        
        
        # if is_3D:
        #     # Load single 3D point clouds
        #     ref = DP(DP.load('../data/car_cloud400.csv'))
        #     data = DP(DP.load('../data/car_cloud401.csv'))
        #     test_base = "3D"
        # else:
        #     # Load single 2D point clouds
        #     ref = DP(DP.load('../data/2D_twoBoxes.csv'))
        #     data = DP(DP.load('../data/2D_oneBox.csv'))
        #   test_base = "2D"

        # Create the default ICP algorithm
        icp = PM.ICP()

        if len(config_file) == 0:
            # See the implementation of setDefault() to create a custom ICP algorithm
            icp.setDefault()
        else:
            # load YAML config
            icp.loadFromYaml(config_file)

        cloud_dimension = ref.getEuclideanDim()

        assert cloud_dimension == 2 or cloud_dimension == 3, "Invalid input point clouds dimension"

        # Parse the translation and rotation to be used to compute the initial transformation
        translation = parse_translation(init_translation, cloud_dimension)
        rotation = parse_rotation(init_rotation, cloud_dimension)

        init_transfo = np.matmul(translation, rotation)

        rigid_trans = PM.get().TransformationRegistrar.create("RigidTransformation")

        if not rigid_trans.checkParameters(init_transfo):
            print("Initial transformations is not rigid, identiy will be used")
            init_transfo = np.identity(cloud_dimension + 1)

        initialized_data = rigid_trans.compute(data, init_transfo)

        # Compute the transformation to express data in ref
        T = icp(initialized_data, ref)

        if is_verbose:
            print(f"match ratio: {icp.errorMinimizer.getWeightedPointUsedRatio():.6}")
            print(f"Translation: {T[:3, 3]}, Rotation: {T[:3, :3]}")

        # Transform data to express it in ref
        data_out = DP(initialized_data)
        icp.transformations.apply(data_out, T)
        

        # Save files to see the results
        ref.save(f"{output_base_directory + test_base}_{output_base_file}_ref.vtk")
        data.save(f"{output_base_directory + test_base}_{output_base_file}_data_in.vtk")
        data_out.save(f"{output_base_directory + test_base}_{output_base_file}_data_out.vtk")

        ref = data_out
        if is_transfo_saved:
            init_file_name = f"{output_base_directory + test_base}_{output_base_file}_init_transfo.txt"
            icp_file_name = f"{output_base_directory + test_base}_{output_base_file}_icp.transfo.txt"
            complete_file_name = f"{output_base_directory + test_base}_{output_base_file}_complete_transfo.txt"

            with open(init_file_name, "w") as f:
                f.write(f"{init_transfo}".replace("[", " ").replace("]", " "))

            with open(icp_file_name, "w") as f:
                f.write(f"{T}".replace("[", " ").replace("]", " "))

            with open(complete_file_name, "w") as f:
                f.write(f"{np.matmul(T, init_transfo)}".replace("[", " ").replace("]", " "))

        else:
            if is_verbose:
                print(f"{test_base} ICP transformation:\n{T}".replace("[", " ").replace("]", " "))

