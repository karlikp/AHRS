#include <cassert>
#include <Eigen/Dense>
#include <flann/flann.hpp>
#include <fstream>

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
	//std::cout << "Final transformation:" << std::endl << T << std::endl;

	return std::make_pair(T, data_out);
}

// Funkcja do obliczania normalnych dla chmury punktów i zwracania połączonych wyników
std::vector<std::array<float, 6>> calculateNormalsSet(const std::vector<Point3D>& points, int k) {
    std::vector<std::array<float, 6>> combinedResults;

    if (points.empty()) {
        return combinedResults; // Zwróć pusty wynik, jeśli brak punktów
    }

    size_t numPoints = points.size();
    std::vector<float> datasetVec(numPoints * 3);
    flann::Matrix<float> dataset(datasetVec.data(), numPoints, 3);

    for (size_t i = 0; i < numPoints; ++i) {
        dataset[i][0] = points[i].x;
        dataset[i][1] = points[i].y;
        dataset[i][2] = points[i].z;
    }

    flann::Index<flann::L2<float>> index(dataset, flann::KDTreeIndexParams(1));
    index.buildIndex();

    std::vector<std::vector<int>> indices(1);
    std::vector<std::vector<float>> dists(1);

    for (size_t i = 0; i < numPoints; ++i) {
        std::array<float, 3> queryArr = {points[i].x, points[i].y, points[i].z};
        flann::Matrix<float> query(queryArr.data(), 1, 3);

        indices[0].resize(k);
        dists[0].resize(k);
        index.knnSearch(query, indices, dists, k, flann::SearchParams(128));

        if (indices[0].empty()) {
            continue; // Brak sąsiadów
        }

        std::vector<Eigen::Vector3f> neighbors;
        for (size_t j = 0; j < indices[0].size(); ++j) {
            int neighborIdx = indices[0][j];
            if (neighborIdx >= 0 && neighborIdx < static_cast<int>(numPoints)) {
                neighbors.emplace_back(points[neighborIdx].x, points[neighborIdx].y, points[neighborIdx].z);
            }
        }

        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto& neighbor : neighbors) {
            centroid += neighbor;
        }
        centroid /= static_cast<float>(neighbors.size());

        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (const auto& neighbor : neighbors) {
            Eigen::Vector3f centered = neighbor - centroid;
            covariance += centered * centered.transpose();
        }

        if (covariance.isZero(1e-6)) {
            continue; // Pomijaj degenerate cases
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        Eigen::Vector3f normal = solver.eigenvectors().col(0);

        Eigen::Vector3f sensorDirection = Eigen::Vector3f(points[i].x, points[i].y, points[i].z) - centroid;
        if (normal.dot(sensorDirection) < 0) {
            normal = -normal;
        }

        combinedResults.push_back({points[i].x, points[i].y, points[i].z, normal[0], normal[1], normal[2]});
    }

    return combinedResults;
}

// template <typename T>
// typename PointMatcher<T>::DataPoints convertToDataPoints(const std::vector<std::array<float, 6>>& pointsWithNormals) {
//    


// }

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