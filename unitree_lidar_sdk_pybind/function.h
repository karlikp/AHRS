#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "../lib/libpointmatcher_beta/pointmatcher/PointMatcher.h"


struct Point3D {
    float x, y, z;
};

std::vector<std::array<float, 6>> calculateNormalsSet(const std::vector<Point3D>& points, int k = 10);

template <typename T>
typename PointMatcher<T>::DataPoints convertToDataPoints(const std::vector<std::array<float, 6>>& pointsWithNormals);

typename PointMatcher<float>::DataPoints convertToDataPoints(const std::vector<Point3D>& points);

std::pair< PointMatcher<float>::TransformationParameters, PointMatcher<float>::DataPoints> 
                        icp_simple(PointMatcher<float>::DataPoints data, PointMatcher<float>::DataPoints ref);