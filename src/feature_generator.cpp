#include "feature_generator.h"

FeatureGenerator::FeatureGenerator(const std::string& model_path) 
{
  infile.open(model_path.c_str());
  while(!infile.eof())
  {
      std::string s;
      std::getline(infile, s);
      std::stringstream ss;
      ss << s;
      double x, y, z;
      ss >> x;
      ss >> y;
      ss >> z;
      Eigen::Vector4d pt0(x, y, z, 1);
      ss >> x;
      ss >> y;
      ss >> z;
      Eigen::Vector4d pt1(x, y, z, 1);

      bool isHistoryPoint = false;
      for(int i = 0; i < points.size(); ++i)
      {
          Eigen::Vector4d& pt = points[i];
          if(pt == pt0)
          {
              isHistoryPoint = true;
          }
      }
      if(!isHistoryPoint)
      {
        points.push_back(pt0);
      }
      isHistoryPoint = false;
      for (int i = 0; i < points.size(); ++i) {
        Eigen::Vector4d &pt = points[i];
        if (pt == pt1) {
          isHistoryPoint = true;
        }
      }
      if (!isHistoryPoint) 
      {
          points.push_back(pt1);
      }

      lines.push_back(std::make_pair(pt0, pt1));

  }
  std::cout << "points size " << points.size() << std::endl;
  for(int i = 0; i < points.size(); ++i)
  {
      std::cout << i << " " << points[i].transpose() << std::endl;
  }
  // create more 3d noisy points if is needed
//   int n = points.size();
//   for (int j = 0; j < n; ++j) 
//   {
//     Eigen::Vector4d p = points[j] + Eigen::Vector4d(0.5, 0.5, -0.5, 0);
//     points.push_back(p);
//   }
//   std::cout << "points size " << points.size() << std::endl;


}

FeatureGenerator::~FeatureGenerator() 
{ 
    infile.close(); 
}

std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
FeatureGenerator::featureObservation(const Eigen::Matrix4d& Twc) 
{
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > observations;
    for(int i = 0; i < points.size(); ++i)
    {
        Eigen::Vector4d pc;
        pc = Twc.inverse() * points[i];
        assert(pc(3) == 1.0);
        if(pc[2] <= 0)
        {
            continue;
        }
        Eigen::Vector2d  p2d = Eigen::Vector2d(pc(0)/pc(2), pc(1)/pc(2));
        observations.push_back(p2d);
    }

    return observations;

}
