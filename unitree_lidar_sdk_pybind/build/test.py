import unitree_lidar_sdk_pybind

# Użycie funkcji z opakowanej klasy
lidar = unitree_lidar_sdk_pybind.UnitreeLidarWrapper()

if lidar.initialize():
    print("Lidar initialized successfully!")
else:
    print("Lidar initialization failed!")
