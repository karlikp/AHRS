#include <cassert>
#include <Eigen/Dense>
#include <flann/flann.hpp>
#include "function.h"



std::pair< PointMatcher<float>::TransformationParameters, PointMatcher<float>::DataPoints> 
                        icp_simple(PointMatcher<float>::DataPoints data, PointMatcher<float>::DataPoints ref)
{
	
    typedef PointMatcher<float> PM;
	typedef PM::DataPoints DP;
	

	// Create the default ICP algorithm
	PM::ICP icp;
	
	// See the implementation of setDefault() to create a custom ICP algorithm
	icp.setDefault();

	// Compute the transformation to express data in ref
	PM::TransformationParameters T = icp(data, ref);

	// Transform data to express it in ref
	DP data_out(data);
	icp.transformations.apply(data_out, T);
	
	// // Safe files to see the results
	// ref.save("test_ref.vtk");
	// data.save("test_data_in.vtk");
	// data_out.save("test_data_out.vtk");
	std::cout << "Final transformation:" << std::endl << T << std::endl;

	return std::make_pair(T, data_out);
}

// Funkcja do obliczania normalnych dla chmury punktów i zwracania połączonych wyników
std::vector<std::array<float, 6>> calculateNormalsSet(const std::vector<Point3D>& points, int k) {

    std::vector<std::array<float, 6>> combinedResults;

    // Tworzenie macierzy dla punktów
    size_t numPoints = points.size();
    flann::Matrix<float> dataset(new float[numPoints * 3], numPoints, 3);

    for (size_t i = 0; i < numPoints; ++i) {
        dataset[i][0] = points[i].x;
        dataset[i][1] = points[i].y;
        dataset[i][2] = points[i].z;
    }

    // Tworzenie KD-Tree
    flann::Index<flann::L2<float>> index(dataset, flann::KDTreeIndexParams(1));
    index.buildIndex();

    // Bufory do przechowywania wyników zapytań
    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;

    for (size_t i = 0; i < numPoints; ++i) {
        // Wyszukiwanie k najbliższych sąsiadów
        flann::Matrix<float> query(new float[3]{points[i].x, points[i].y, points[i].z}, 1, 3);
        index.knnSearch(query, indices, dists, k, flann::SearchParams(128));
        delete[] query.ptr();

        // Pobranie sąsiadów
        std::vector<Eigen::Vector3f> neighbors;
        for (size_t j = 0; j < indices[i].size(); ++j) {
            int neighborIdx = indices[i][j];
         neighbors.emplace_back(points[neighborIdx].x, points[neighborIdx].y, points[neighborIdx].z);
        }

        // Oblicz centroid
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto& neighbor : neighbors) {
            centroid += neighbor;
        }
        centroid /= static_cast<float>(neighbors.size());

        // Oblicz macierz kowariancji
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (const auto& neighbor : neighbors) {
            Eigen::Vector3f centered = neighbor - centroid;
            covariance += centered * centered.transpose();
        }

        // Rozkład SVD
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        Eigen::Vector3f normal = solver.eigenvectors().col(0); // Najmniejsza wartość własna

        // Orientacja normalnej w stronę sensora
        Eigen::Vector3f sensorDirection = Eigen::Vector3f(points[i].x, points[i].y, points[i].z) - centroid;
        if (normal.dot(sensorDirection) < 0) {
            normal = -normal;
        }

        // Połącz punkt z odpowiadającą mu normalną
        combinedResults.push_back({points[i].x, points[i].y, points[i].z, normal[0], normal[1], normal[2]});
    }

    delete[] dataset.ptr();
    return combinedResults;
}

template <typename T>
typename PointMatcher<T>::DataPoints convertToDataPoints(const std::vector<std::array<float, 6>>& pointsWithNormals) {
    using PM = PointMatcher<T>;

    // Liczba punktów
    const size_t pointCount = pointsWithNormals.size();

    // Tworzenie macierzy cech (features) i deskryptorów (descriptors)
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> features(4, pointCount); // x, y, z, pad
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> descriptors(3, pointCount); // nx, ny, nz

    // Wypełnianie macierzy cech i deskryptorów
    for (size_t i = 0; i < pointCount; ++i) {
        const auto& p = pointsWithNormals[i];

        // Cecha: x, y, z, 1 (dla współrzędnych homogenicznych)
        features(0, i) = p[0]; // x
        features(1, i) = p[1]; // y
        features(2, i) = p[2]; // z
        features(3, i) = 1.0;  // pad

        // Deskryptor: nx, ny, nz
        descriptors(0, i) = p[3]; // nx
        descriptors(1, i) = p[4]; // ny
        descriptors(2, i) = p[5]; // nz
    }

    // Etykiety dla cech i deskryptorów
    typename PM::DataPoints::Labels featureLabels = {
        PM::DataPoints::Label("x", 1),
        PM::DataPoints::Label("y", 1),
        PM::DataPoints::Label("z", 1),
        PM::DataPoints::Label("pad", 1)
    };

    typename PM::DataPoints::Labels descriptorLabels{
        PM::DataPoints::Label("normals", 3) // nx, ny, nz
    };

    // Tworzenie obiektu DataPoints
    typename PM::DataPoints dataPoints(features, featureLabels, descriptors, descriptorLabels);

    return dataPoints;
}

// Funkcja konwertująca std::vector<Point3D> na PointMatcher<float>::DataPoints
typename PointMatcher<float>::DataPoints convertToDataPoints(const std::vector<Point3D>& points) {
    using PM = PointMatcher<float>;
    using DataPoints = PM::DataPoints;

    // Liczba punktów w wektorze
    const size_t pointCount = points.size();

    // Tworzenie macierzy cech (features): x, y, z, pad
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> features(4, pointCount);

    // Wypełnianie macierzy cech
    for (size_t i = 0; i < pointCount; ++i) {
        features(0, i) = points[i].x; // x
        features(1, i) = points[i].y; // y
        features(2, i) = points[i].z; // z
        features(3, i) = 1.0f;        // pad (dla współrzędnych homogenicznych)
    }

   typename PointMatcher<float>::DataPoints::Labels featureLabels;
featureLabels.push_back(PointMatcher<float>::DataPoints::Label("x", 1));
featureLabels.push_back(PointMatcher<float>::DataPoints::Label("y", 1));
featureLabels.push_back(PointMatcher<float>::DataPoints::Label("z", 1));
featureLabels.push_back(PointMatcher<float>::DataPoints::Label("pad", 1));

typename PointMatcher<float>::DataPoints::Labels descriptorLabels;
descriptorLabels.push_back(PointMatcher<float>::DataPoints::Label("normals", 3)); // nx, ny, nz

    // Tworzenie obiektu DataPoints
    DataPoints dataPoints(features, featureLabels);

    return dataPoints;
}