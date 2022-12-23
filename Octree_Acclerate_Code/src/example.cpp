#include <iostream>
#include <cstdlib>
#include <time.h>

#include "Octree.hpp"



//#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <algorithm>

#include "OctreeOCL.hpp"

#include <typeinfo>

/** Example 4: Searching radius neighbors with default access by public x,y,z variables.
 *
 * \author behley
 */

#include <fstream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <chrono>
using namespace std;

template <typename PointT, typename ContainerT>
void readPoints(const std::string& filename, ContainerT& points)
{
  std::ifstream in(filename.c_str());
  if(!in.is_open()){
      throw std::runtime_error("No such file");
  }
  std::string line;
  boost::char_separator<char> sep(" ");
  // read point cloud from "freiburg format"
  while (!in.eof())
  {
    std::getline(in, line);
    cout << line << endl;
    in.peek();

    boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
    std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());

    cout << tokens.size() << endl;

    if (tokens.size() != 3) continue;
    float x = boost::lexical_cast<float>(tokens[3]);
    float y = boost::lexical_cast<float>(tokens[4]);
    float z = boost::lexical_cast<float>(tokens[5]);

    points.push_back(PointT(x, y, z));
  }

  in.close();
}

bool twoVectorsEqual(std::vector<int>& a, std::vector<int>& b){
    if(a.size() != b.size()) return false;
    std::sort(a.begin(), a.end(), std::greater<int>());
    std::sort(b.begin(), b.end(), std::greater<int>());
    for(int i=0; i<a.size(); i++){
        if(a[i] != b[i]) return false;
    }
    return true;
}

bool twoVectorVectorsEqual(std::vector<std::vector<int>>& a, std::vector<std::vector<int>>& b){
    if(a.size() != b.size()) return false;
    for(int i=0; i<a.size(); i++){
        if(!twoVectorsEqual(a[i], b[i])) return false;
    }
    return true;
}


using namespace std;

typedef pcl::PointXYZ PointT;



int main(int argc, char** argv)
{    
    float mult;
    bool test_type = false;
    if(test_type)
        mult = 0.5;
    else
        mult = 2.5;

    // Read filename
    if (argc < 2)
    {
        std::cerr << "filename of point cloud missing." << std::endl;
        return -1;
    }
    std::string filename = argv[1];
    cout << filename << endl;

    // Load points
    std::vector<PointT> points;
    pcl::PointCloud<PointT> pointCloud;
    pcl::io::loadPCDFile(filename, pointCloud);
    for(int i=0; i<pointCloud.size(); i++) points.push_back(pointCloud[i]);

    // Increase Points
    std::vector<PointT> pointsTemp;
    for(int i=0; i<points.size(); i++){
        for(float j=0; j<mult; j+=0.01){
            pointsTemp.push_back(pcl::PointXYZ(points[i].x+j, points[i].y-j, points[i].z+j));
        }
    }
    points = pointsTemp;
    std::random_shuffle(points.begin(), points.end());
    cout << "Total Size: " << points.size() << endl;

    // Load PCL version of points
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPoints (new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<points.size(); i++){
        pclPoints->push_back(pcl::PointXYZ(points[i].x, points[i].y, points[i].z));
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();

    // Initializing the Octree with points from point cloud.
    OctreeGPU<PointT> octree;
    unibn::OctreeParams params;

    // Initialize
    begin = std::chrono::steady_clock::now();
    octree.initialize(points);
    end = std::chrono::steady_clock::now();
    std::cout << "Init Time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

    begin = std::chrono::steady_clock::now();
    octree.linearizeTree();
    end = std::chrono::steady_clock::now();
    std::cout << "Lin Time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

    begin = std::chrono::steady_clock::now();
    octree.uploadTreeToGPU();
    end = std::chrono::steady_clock::now();
    std::cout << "GPU Upload Time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (pclPoints);

    // Setup queries
    std::vector<pcl::PointXYZ> queries;
    for(int i=0; i<points.size(); i++){
        PointT testPoint = pclPoints->at(i);
        testPoint.x+=0.02;
        testPoint.z+=0.05;
        testPoint.y-=0.01;
        queries.push_back(testPoint);
    }
    std::random_shuffle(queries.begin(), queries.end());
    cout << queries.size() << endl;

    if(test_type){

        // PCL Test
        begin = std::chrono::steady_clock::now();
        std::vector<std::vector<int>> pclResults(queries.size());
        for(int i=0; i<queries.size(); i++){
            std::vector<int> treeIndices;
            std::vector<float> treeDists;
            tree->radiusSearch(queries[i], 0.5, treeIndices, treeDists);
            pclResults[i] = treeIndices;
        }
        end = std::chrono::steady_clock::now();
        std::cout << "PCL Speed = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

        // Default Test
        begin = std::chrono::steady_clock::now();
        std::vector<std::vector<int>> defaultResults(queries.size());
        for(int i=0; i<queries.size(); i++){
            std::vector<uint32_t> resultsDefault;
            octree.radiusNeighbors<unibn::L2Distance<PointT>>(queries[i], 0.5, resultsDefault);
            std::vector<int> resultsDefaultConv;
            for(uint32_t m : resultsDefault) resultsDefaultConv.push_back((int)m);
            defaultResults[i] = resultsDefaultConv;
        }
        end = std::chrono::steady_clock::now();
        std::cout << "Default Speed = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

        // Non Recursive Test
        begin = std::chrono::steady_clock::now();
        std::vector<std::vector<int>> nonRecursiveResults(queries.size());
        for(int i=0; i<queries.size(); i++){
            std::vector<int> resultsNonRecursive = octree.radiusNeighboursExt(queries[i], 0.5);
            nonRecursiveResults[i] = resultsNonRecursive;
        }
        end = std::chrono::steady_clock::now();
        std::cout << "NR Speed = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

        if(!twoVectorVectorsEqual(pclResults, defaultResults)) cout << "FUCK" << endl;

        if(!twoVectorVectorsEqual(nonRecursiveResults, defaultResults)) cout << "FUCK" << endl;

    }
    else
    {
        // PCL Test
        begin = std::chrono::steady_clock::now();
        std::vector<int> indicesPCL(queries.size());
        for(int i=0; i<queries.size(); i++){
            std::vector<int> treeIndices (1);
            std::vector<float> treeDists (1);
            tree->nearestKSearch(queries[i], 1, treeIndices, treeDists);
            indicesPCL[i] = treeIndices[0];
        }
        end = std::chrono::steady_clock::now();
        std::cout << "PCL Speed = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

        // GPU Test
        begin = std::chrono::steady_clock::now();
        std::vector<int> indicesGPU = octree.findNeighboursGPU(queries);
        end = std::chrono::steady_clock::now();
        std::cout << "GPU Speed = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

        // CPU Test
        begin = std::chrono::steady_clock::now();
        std::vector<int> indicesCPU(queries.size());
        #pragma omp parallel for shared(octree)
        for(int i=0; i<queries.size(); i++){
            indicesCPU[i] = octree.findNeighbor(queries[i]);
        }
        end = std::chrono::steady_clock::now();
        std::cout << "CPU Speed = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

        // Confirm
        for(int i=0; i<queries.size(); i++){
            if(indicesCPU[i] != indicesGPU[i]) cout << "SHIT CPU/GPU MISMATCH " << i << endl;
            if(indicesPCL[i] != indicesGPU[i]) cout << "SHIT PCL/GPU MISMATCH " << i << endl;
        }
    }

    return 0;
}
