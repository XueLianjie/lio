#ifndef _FEATURE_GENERATOR_H_
#define _FEATURE_GENERATOR_H_

#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <string>
#include <fstream>

class FeatureGenerator
{
private:
    /* data */
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points;
    std::vector<std::pair<Eigen::Vector4d, Eigen::Vector4d> > lines;
    std::ifstream infile;

public:
    FeatureGenerator(const std::string& model_path);
    ~FeatureGenerator();

    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > featureObservation(const Eigen::Matrix4d& Twc);

};







#endif // end feature generator
