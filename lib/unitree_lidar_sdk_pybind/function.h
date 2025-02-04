#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "../libpointmatcher/pointmatcher/PointMatcher.h"

typedef PointMatcher<float> PM; // Używany typ PointMatcher
typedef PM::DataPoints DP;      // Alias dla PointMatcher<float>::DataPoints

struct Point3D {
    float x, y, z;
};

std::vector<std::array<float, 6>> calculateNormalsSet(const std::vector<Point3D>& points, int k = 10);

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

    // Tworzenie etykiet dla cech
    typename PM::DataPoints::Labels featureLabels;
    featureLabels.push_back(typename PM::DataPoints::Label("x", 1));
    featureLabels.push_back(typename PM::DataPoints::Label("y", 1));
    featureLabels.push_back(typename PM::DataPoints::Label("z", 1));
    featureLabels.push_back(typename PM::DataPoints::Label("pad", 1));

    // Tworzenie etykiet dla deskryptorów
    typename PM::DataPoints::Labels descriptorLabels;
    descriptorLabels.push_back(typename PM::DataPoints::Label("normals", 3)); // nx, ny, nz

    // Tworzenie obiektu DataPoints
    typename PM::DataPoints dataPoints(features, featureLabels, descriptors, descriptorLabels);

    return dataPoints;
}

typename PointMatcher<float>::DataPoints convertToDataPoints(const std::vector<Point3D>& points);

std::pair< PointMatcher<float>::TransformationParameters, PointMatcher<float>::DataPoints> 
                        icp_simple(PointMatcher<float>::DataPoints data, PointMatcher<float>::DataPoints ref);