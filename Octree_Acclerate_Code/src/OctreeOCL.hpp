#include <iostream>
#include <cstdlib>
#include <time.h>
#include <algorithm>
#include <typeinfo>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>

#include "OCLManager.hpp"
#include "Octree.hpp"

const bool DEBUG = false;

using namespace std;

template<typename PointT>
class OctreeGPU : public unibn::Octree<PointT>{
public:

    typedef typename unibn::Octree<PointT>::Octant Octant;

    class OctantLinear
    {
    public:
        OctantLinear(){}
        ~OctantLinear(){}

        bool isLeaf;

        float x, y, z;
        float extent;

        uint32_t start, end;
        uint32_t size;

        int32_t childIndex[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };

        OctantLinear(Octant* octant){
            this->isLeaf = octant->isLeaf;
            this->x = octant->x;
            this->y = octant->y;
            this->z = octant->z;
            this->extent = octant->extent;
            this->start = octant->start;
            this->end = octant->start;
            this->size = octant->size;
        }

        void print(){
            printf("isLeaf: %d Centroid: (%f, %f, %f %f) SES: (%d, %d, %d) Tree: %d %d %d %d %d %d %d %d \n", isLeaf, x, y, z, extent, start, end, size, childIndex[0], childIndex[1], childIndex[2], childIndex[3], childIndex[4], childIndex[5], childIndex[6], childIndex[7]);
        }
    };

    struct Stack{
        int index;
        uint32_t mortonCode = 0;
        int loopIndex = 0;
        bool seen1 = false;
        bool seen2 = false;
        float sqrMaxDistance;

        Stack(){}

        Stack(int index){
            this->index = index;
        }
    };

    struct StackVector{

        static const int MAX_SIZE = 100;
        Stack indexStack[MAX_SIZE];

        int pointer = -1;

        int size(){
            return pointer+1;
        }

        Stack& back(){
            return indexStack[pointer];
        }

        void push_back(const Stack& stack){
            pointer++;
            if(pointer >= MAX_SIZE){
                cerr << "Stack Overflow" << endl;
                throw;
            }
            indexStack[pointer] = stack;
        }

        void pop_back(){
            pointer--;
        }

        void reinit(){
            pointer = -1;
        }

    };

    int leafCount_ = 0;
    std::vector<OctantLinear> octantLinearVector_;

    OCLManager* ocl_ = OCLManager::getInstance();
    cl::Buffer octantLinearVectorBuffer_;
    cl::Buffer successorsBuffer_;
    cl::Buffer cloudBuffer_;

    OctreeGPU() : unibn::Octree<PointT>(){
        ocl_->setCLPath(getCurrentPath());
        ocl_->buildKernelIntoManager<PointT>("OctreeOCL.cl", "nearestNeighbourKernel");
    }

    std::string getCurrentPath(){
        std::string path(__FILE__);
        std::vector<std::string> strs;
        boost::split(strs,path,boost::is_any_of("/"));
        std::string finalPath;
        cout << strs.size() << endl;
        for(int i=0; i<strs.size()-1; i++)  finalPath+=(strs[i] + "/");
        return finalPath;
    }

    void clear(){
        delete this->root_;
        if (this->params_.copyPoints) delete this->data_;
        this->root_ = 0;
        this->data_ = 0;
        this->successors_.clear();
        this->leafCount_ = 0;
        this->octantLinearVector_.clear();
    }

    void linearizeTree(){
        recursiveSearch(this->root_);
    }

    int recursiveSearch(Octant* octant){
        if(octant == NULL) return -1;
        OctantLinear octantLinear(octant);
        if(octantLinear.isLeaf) leafCount_++;
        for(int i=0; i<8; i++){
            octantLinear.childIndex[i] = recursiveSearch(octant->child[i]);
        }
        octantLinearVector_.push_back(octantLinear);
        return octantLinearVector_.size() - 1;
    }

    void uploadTreeToGPU(){
        const std::vector<PointT>& points = *this->data_;
        //cout << "Tree Count: " << octantLinearVector_.size() << " Cloud Count: " << points.size() << endl;
        octantLinearVectorBuffer_ = cl::Buffer(ocl_->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(OctantLinear) * octantLinearVector_.size(), &octantLinearVector_[0]);
        successorsBuffer_ = cl::Buffer(ocl_->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(uint32_t) * this->successors_.size(), &this->successors_[0]);
        cloudBuffer_ = cl::Buffer(ocl_->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(PointT) * points.size(), (void*)&points[0]);
    }

    int findNeighbor(PointT& query){

        // Initial Variables
        const std::vector<PointT>& points = *this->data_;
        float minDistance = -1;
        float maxDistance = std::numeric_limits<float>::infinity();
        int32_t resultIndex;

        // Initial Stack
        int currIndex = octantLinearVector_.size() - 1;
        StackVector indexStack;
        indexStack.push_back(Stack(currIndex));
        bool lastReturn = 0;

        // Run Iter
        int iter = 0;
        while(indexStack.size()){
            iter++;
            //if(iter++ > 500) {cerr << "Too many iterations" << endl; throw;}

            // Get Stack Items
            int currIndex = indexStack.back().index;
            uint32_t mortonCode = indexStack.back().mortonCode;
            int lastLoopIndex = indexStack.back().loopIndex;
            OctantLinear& octantLinear = octantLinearVector_[currIndex];
            if(DEBUG) cout << currIndex << " " << lastLoopIndex << endl;

            // First descend to leaf and check in leafs points.
            if(octantLinear.isLeaf){

                uint32_t idx = octantLinear.start;
                float sqrMaxDistance = unibn::L2Distance<PointT>::sqr(maxDistance);
                float sqrMinDistance = (minDistance < 0) ? minDistance : unibn::L2Distance<PointT>::sqr(minDistance);

                for (uint32_t i = 0; i < octantLinear.size; ++i)
                {
                    const PointT& p = points[idx];
                    float dist = unibn::L2Distance<PointT>::compute(query, p);
                    if (dist > sqrMinDistance && dist < sqrMaxDistance)
                    {
                        resultIndex = idx;
                        sqrMaxDistance = dist;
                    }
                    idx = this->successors_[idx];
                }

                maxDistance = unibn::L2Distance<PointT>::sqrt(sqrMaxDistance);
                lastReturn = inside(query, maxDistance, octantLinear);
                indexStack.pop_back();
                //cout << "POPSIZE: " << indexStack.size() << endl;
                if(DEBUG) std::cout << "inside: " << lastReturn << std::endl;
                continue;
            }

            // Stack Status Flags
            bool seen1Test = indexStack.back().seen1;
            bool seen2Test = indexStack.back().seen2;

            // Determine Morton code for each point...
            if(seen1Test){
                if(lastReturn){
                    indexStack.pop_back();
                    continue;
                }
            }else{
                mortonCode = 0;
                if (unibn::get<0>(query) > octantLinear.x) mortonCode |= 1;
                if (unibn::get<1>(query) > octantLinear.y) mortonCode |= 2;
                if (unibn::get<2>(query) > octantLinear.z) mortonCode |= 4;
                indexStack.back().mortonCode = mortonCode;
                if(octantLinear.childIndex[mortonCode] >= 0)
                {
                    indexStack.back().seen1 = true;
                    indexStack.push_back(Stack(octantLinear.childIndex[mortonCode]));
                    continue;
                }
            }

            // If current best point completely inside, just return.
            float sqrMaxDistance;
            if(seen2Test){
                sqrMaxDistance = indexStack.back().sqrMaxDistance;
            }else{
                sqrMaxDistance = unibn::L2Distance<PointT>::sqr(maxDistance);
                indexStack.back().sqrMaxDistance = sqrMaxDistance;
            }

            // Check adjacent octants for overlap and check these if necessary.
            bool found = false;
            for (uint32_t c = lastLoopIndex; c < 8; ++c)
            {
                indexStack.back().loopIndex = c+1;
                if(DEBUG) std::cout << currIndex << " loop idx: " << c << " " << octantLinear.childIndex[c] << std::endl;

                if(lastReturn){
                    if(DEBUG) cout << "lastreturn" << endl;
                    found = true;
                    indexStack.pop_back();
                    break;
                }else{
                    if(DEBUG) cout << "    test " << mortonCode << " " << sqrMaxDistance << " " << maxDistance << endl;
                    if (c == mortonCode) continue;
                    if (octantLinear.childIndex[c] < 0) continue;
                    if (!overlaps(query, maxDistance, sqrMaxDistance, octantLinearVector_[octantLinear.childIndex[c]])) continue;
                    indexStack.back().seen2 = true;
                    indexStack.push_back(Stack(octantLinear.childIndex[c]));
                    found = true;
                    break;
                }
            }
            if(found) continue;

            // Return
            lastReturn = inside(query, maxDistance, octantLinear);
            indexStack.pop_back();
        }

        return resultIndex;
    }

    std::vector<int> findNeighboursGPU(std::vector<PointT>& queries){
        std::vector<int> indices(queries.size());
        cl::Buffer queriesBuffer = cl::Buffer(ocl_->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(PointT) * queries.size(), (void*)&queries[0]);
        cl::Buffer indicesBuffer = cl::Buffer(ocl_->getContext(), CL_MEM_WRITE_ONLY, sizeof(int) * queries.size());
        try{
            cl::Kernel nearestNeighbourKernel = ocl_->getKernelFromManager("nearestNeighbourKernel");
            nearestNeighbourKernel.setArg(0, cloudBuffer_);
            nearestNeighbourKernel.setArg(1, octantLinearVectorBuffer_);
            nearestNeighbourKernel.setArg(2, (int)octantLinearVector_.size());
            nearestNeighbourKernel.setArg(3, successorsBuffer_);
            nearestNeighbourKernel.setArg(4, queriesBuffer);
            nearestNeighbourKernel.setArg(5, indicesBuffer);
            ocl_->getQueue().enqueueNDRangeKernel(nearestNeighbourKernel, 0, cl::NDRange(queries.size()), cl::NullRange, NULL, NULL);
            ocl_->getQueue().finish();
            ocl_->getQueue().enqueueReadBuffer(indicesBuffer, CL_TRUE, 0, sizeof(int) * queries.size(), (void*)&indices[0]);
        }catch(cl::Error& e){
            cout << e.err() << endl;
            cout << e.what() << endl;
        }
        return indices;
    }

    std::vector<int> radiusNeighboursExt(PointT& query, float radius){
        float sqrRadius = radius * radius;
        const std::vector<PointT>& points = *this->data_;

        // Initial Variables
        std::vector<int> resultIndices;

        // Initial Stack
        int currIndex = octantLinearVector_.size() - 1;
        StackVector indexStack;
        indexStack.push_back(Stack(currIndex));
        bool lastReturn = 0;

        // Run Iter
        int iter = 0;
        while(indexStack.size()){
            iter++;
            if(iter++ > 1000) break;

            // Get Stack Items
            int currIndex = indexStack.back().index;
            uint32_t mortonCode = indexStack.back().mortonCode;
            int lastLoopIndex = indexStack.back().loopIndex;
            OctantLinear& octantLinear = octantLinearVector_[currIndex];
            if(DEBUG) cout << currIndex << " " << lastLoopIndex << endl;

            // Stack Status Flags
            bool seen1Test = indexStack.back().seen1;
            bool seen2Test = indexStack.back().seen2;

            // if search ball S(q,r) contains octant, simply add point indexes.
            if (contains(query, sqrRadius, octantLinear) && !seen1Test)
            {
                uint32_t idx = octantLinear.start;
                for (uint32_t i = 0; i < octantLinear.size; ++i)
                {
                    resultIndices.push_back(idx);
                    idx = this->successors_[idx];
                }

                indexStack.pop_back();
                continue;
            }

            // leaf codes
            if (octantLinear.isLeaf && !seen1Test)
            {
                uint32_t idx = octantLinear.start;
                for (uint32_t i = 0; i < octantLinear.size; ++i)
                {
                    const PointT& p = points[idx];
                    float dist = compute(query, p);
                    if (dist < sqrRadius) resultIndices.push_back(idx);
                    idx = this->successors_[idx];
                }

                indexStack.pop_back();
                continue;
            }

            // check whether child nodes are in range.
            bool found = false;
            for (uint32_t c = lastLoopIndex; c < 8; ++c)
            {
                indexStack.back().loopIndex = c+1;
                if (octantLinear.childIndex[c] < 0) continue;
                if (!overlaps(query, radius, sqrRadius, octantLinearVector_[octantLinear.childIndex[c]])) continue;

                found = true;
                indexStack.back().seen1 = true;
                indexStack.push_back(Stack(octantLinear.childIndex[c]));
                break;
            }
            if(!found) indexStack.pop_back();
        }

        return resultIndices;
    }

    float compute(const PointT& p, const PointT& q)
    {
        float diff1 = unibn::get<0>(p) - unibn::get<0>(q);
        float diff2 = unibn::get<1>(p) - unibn::get<1>(q);
        float diff3 = unibn::get<2>(p) - unibn::get<2>(q);

        return std::pow(diff1, 2) + std::pow(diff2, 2) + std::pow(diff3, 2);
    }

    bool inside(const PointT& query, float radius, const OctantLinear& octantLinear)
    {
        // we exploit the symmetry to reduce the test to test
        // whether the farthest corner is inside the search ball.
        float x = unibn::get<0>(query) - octantLinear.x;
        float y = unibn::get<1>(query) - octantLinear.y;
        float z = unibn::get<2>(query) - octantLinear.z;

        x = std::abs(x) + radius;
        y = std::abs(y) + radius;
        z = std::abs(z) + radius;

        if (x > octantLinear.extent) return false;
        if (y > octantLinear.extent) return false;
        if (z > octantLinear.extent) return false;

        return true;
    }

    bool overlaps(const PointT& query, float radius, float sqRadius, const OctantLinear& octantLinear)
    {
        // we exploit the symmetry to reduce the test to testing if its inside the Minkowski sum around the positive quadrant.
        float x = unibn::get<0>(query) - octantLinear.x;
        float y = unibn::get<1>(query) - octantLinear.y;
        float z = unibn::get<2>(query) - octantLinear.z;

        x = std::abs(x);
        y = std::abs(y);
        z = std::abs(z);

        float maxdist = radius + octantLinear.extent;

        // Completely outside, since q' is outside the relevant area.
        if (x > maxdist || y > maxdist || z > maxdist) return false;

        int32_t num_less_extent = (x < octantLinear.extent) + (y < octantLinear.extent) + (z < octantLinear.extent);

        // Checking different cases:

        // a. inside the surface region of the octant.
        if (num_less_extent > 1) return true;

        // b. checking the corner region as well as the edge region.
        x = std::max(x - octantLinear.extent, 0.0f);
        y = std::max(y - octantLinear.extent, 0.0f);
        z = std::max(z - octantLinear.extent, 0.0f);

        return (unibn::L2Distance<PointT>::norm(x, y, z) < sqRadius);
    }

    bool contains(const PointT& query, float sqRadius, const OctantLinear& octantLinear)
    {
        // we exploit the symmetry to reduce the test to test
        // check whether the farthest corner is inside the search ball.
        float x = unibn::get<0>(query) - octantLinear.x;
        float y = unibn::get<1>(query) - octantLinear.y;
        float z = unibn::get<2>(query) - octantLinear.z;

        x = std::abs(x);
        y = std::abs(y);
        z = std::abs(z);
        // reminder: (x, y, z) - (-e, -e, -e) = (x, y, z) + (e, e, e)
        x += octantLinear.extent;
        y += octantLinear.extent;
        z += octantLinear.extent;

        return (unibn::L2Distance<PointT>::norm(x, y, z) < sqRadius);
    }


};
