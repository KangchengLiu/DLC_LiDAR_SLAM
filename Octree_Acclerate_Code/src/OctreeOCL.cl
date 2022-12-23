
///////////////// STACK //////////////////////////

typedef struct{
    int index;
    int mortonCode;
    int loopIndex;
    bool seen1;
    bool seen2;
    float sqrMaxDistance;
}Stack;

Stack getEmptyStack(int index){
    Stack stack;
    stack.index = index;
    stack.mortonCode = 0;
    stack.loopIndex = 0;
    stack.seen1 = false;
    stack.seen2 = false;
    return stack;
}

#define MAX_SIZE 100
typedef struct{
    Stack indexStack[MAX_SIZE];
    int pointer;
}StackVector;

StackVector getEmptyStackVector(){
    StackVector stackVector;
    for(int i=0; i<MAX_SIZE; i++){
        stackVector.indexStack[i] = getEmptyStack(0);
    }
    stackVector.pointer = -1;
    return stackVector;
}

int stack_size(StackVector* stackVector){
    return stackVector->pointer+1;
}

Stack* back(StackVector* stackVector){
    return &stackVector->indexStack[stackVector->pointer];
}

void push_back(StackVector* stackVector, Stack* stack){
    stackVector->pointer++;
    if(stackVector->pointer >= MAX_SIZE){
        printf("STACK OVERFLOW \n");
        return;
    }
    stackVector->indexStack[stackVector->pointer] = *stack;
}

void pop_back(StackVector* stackVector){
    stackVector->pointer--;
}

///////////////// STRUCT //////////////////////////

typedef struct{
    bool isLeaf;
    float x, y, z;
    float extent;
    int start, end;
    int size;
    int childIndex[8];
}OctantLinear;

void printOctantLinear(OctantLinear* octantLinear){
    printf("isLeaf: %d Centroid: (%f, %f, %f %f) SES: (%d, %d, %d) Tree: %d %d %d %d %d %d %d %d \n", octantLinear->isLeaf, octantLinear->x, octantLinear->y, octantLinear->z, octantLinear->extent, octantLinear->start, octantLinear->end, octantLinear->size, octantLinear->childIndex[0], octantLinear->childIndex[1], octantLinear->childIndex[2], octantLinear->childIndex[3], octantLinear->childIndex[4], octantLinear->childIndex[5], octantLinear->childIndex[6], octantLinear->childIndex[7]);
}

///////////////// CODE //////////////////////////

void printPoint(PointT* p)
{
    printf("%f %f %f\n",p->x,p->y,p->z);
}

void printPointNormal(PointT* p)
{
    //printf("%f %f %f %f %f %f\n",p->x,p->y,p->z,p->normal_x,p->normal_y,p->normal_z);
}

float compute(const PointT* a, const PointT* b){
    float diff1 = a->x - b->x;
    float diff2 = a->y - b->y;
    float diff3 = a->z - b->z;
    return pow(diff1, 2) + pow(diff2, 2) + pow(diff3, 2);
}

bool inside(const PointT* query, float radius, const OctantLinear* octantLinear)
{
    // we exploit the symmetry to reduce the test to test
    // whether the farthest corner is inside the search ball.
    float x = query->x - octantLinear->x;
    float y = query->y - octantLinear->y;
    float z = query->z - octantLinear->z;

    x = fabs(x) + radius;
    y = fabs(y) + radius;
    z = fabs(z) + radius;

    if (x > octantLinear->extent) return false;
    if (y > octantLinear->extent) return false;
    if (z > octantLinear->extent) return false;

    return true;
}

bool overlaps(const PointT* query, float radius, float sqRadius, const OctantLinear* octantLinear)
{
    // we exploit the symmetry to reduce the test to testing if its inside the Minkowski sum around the positive quadrant.
    float x = query->x - octantLinear->x;
    float y = query->y - octantLinear->y;
    float z = query->z - octantLinear->z;

    x = fabs(x);
    y = fabs(y);
    z = fabs(z);

    float maxdist = radius + octantLinear->extent;

    // Completely outside, since q' is outside the relevant area.
    if (x > maxdist || y > maxdist || z > maxdist) return false;

    int num_less_extent = (x < octantLinear->extent) + (y < octantLinear->extent) + (z < octantLinear->extent);

    // Checking different cases:

    // a. inside the surface region of the octant.
    if (num_less_extent > 1) return true;

    // b. checking the corner region && edge region.
    x = max(x - octantLinear->extent, 0.0f);
    y = max(y - octantLinear->extent, 0.0f);
    z = max(z - octantLinear->extent, 0.0f);

    float norm = pow(x,2) + pow(y,2) + pow(z,2);
    return (norm < sqRadius);
}

int nearestNeighbourSearch(__global PointT* data, __global OctantLinear* octantLinearVector, int octantLinearVectorSize, __global int* successors, __private PointT* queryP){
    PointT query = *queryP;

    // Initial Variables
    float minDistance = -1;
    float maxDistance = MAXFLOAT;
    int resultIndex;

    // Initial Stack
    Stack stack = getEmptyStack(octantLinearVectorSize-1);
    StackVector stackVector = getEmptyStackVector();
    push_back(&stackVector, &stack);
    bool lastReturn = 0;
    bool DEBUG = 0;

    // Run Iter
    int iter = 0;
    while(stack_size(&stackVector)){
        iter++;
        if(iter > 500) break; //safety remove later

        // Get Stack Items
        int currIndex = back(&stackVector)->index;
        int mortonCode = back(&stackVector)->mortonCode;
        int lastLoopIndex = back(&stackVector)->loopIndex;
        OctantLinear octantLinear = octantLinearVector[currIndex];
        if(DEBUG) printf("%d %d\n", currIndex, lastLoopIndex);

        // First descend to leaf and check in leafs points.
        if(octantLinear.isLeaf){
            int idx = octantLinear.start;
            float sqrMaxDistance = maxDistance * maxDistance;
            float sqrMinDistance = (minDistance < 0) ? minDistance : (minDistance * minDistance);

            for (int i = 0; i < octantLinear.size; ++i)
            {
                const PointT p = data[idx];
                float dist = compute(&query, &p);
                if (dist > sqrMinDistance && dist < sqrMaxDistance)
                {
                    resultIndex = idx;
                    sqrMaxDistance = dist;
                }
                idx = successors[idx];
            }

            maxDistance = sqrt(sqrMaxDistance);
            lastReturn = inside(&query, maxDistance, &octantLinear);
            pop_back(&stackVector);
            if(DEBUG) printf("inside: %d\n", lastReturn);
            continue;
        }

        // Stack Status Flags
        bool seen1Test = back(&stackVector)->seen1;
        bool seen2Test = back(&stackVector)->seen2;

        // Determine Morton code for each point...
        if(seen1Test){
            if(lastReturn){
                pop_back(&stackVector);
                continue;
            }
        }else{
            mortonCode = 0;
            if(query.x > octantLinear.x) mortonCode |= 1;
            if(query.y > octantLinear.y) mortonCode |= 2;
            if(query.z > octantLinear.z) mortonCode |= 4;
            back(&stackVector)->mortonCode = mortonCode;
            if(octantLinear.childIndex[mortonCode] >= 0){
                back(&stackVector)->seen1 = true;
                Stack tempStack = getEmptyStack(octantLinear.childIndex[mortonCode]);
                push_back(&stackVector, &tempStack);
                continue;
            }
        }

        // If current best point completely inside, just return.
        float sqrMaxDistance;
        if(seen2Test){
            sqrMaxDistance = back(&stackVector)->sqrMaxDistance;
        }else{
            sqrMaxDistance = maxDistance * maxDistance;
            back(&stackVector)->sqrMaxDistance = sqrMaxDistance;
        }

        // Check adjacent octants for overlap and check these if necessary.
        bool found = false;
        for (int c = lastLoopIndex; c < 8; ++c)
        {
            back(&stackVector)->loopIndex = c+1;
            if(DEBUG) printf("%d loop idx: %d %d\n", currIndex, c, octantLinear.childIndex[c]);

            if(lastReturn){
                if(DEBUG) printf("lastreturn\n");
                found = true;
                pop_back(&stackVector);
                break;
            }else{
                if(DEBUG) printf("    test %d %f %f\n", mortonCode, sqrMaxDistance, maxDistance);
                if (c == mortonCode) continue;
                if (octantLinear.childIndex[c] < 0) continue;
                OctantLinear tempOctantLinear = octantLinearVector[octantLinear.childIndex[c]];
                if (!overlaps(&query, maxDistance, sqrMaxDistance, &tempOctantLinear)) continue;
                back(&stackVector)->seen2 = true;
                Stack tempStack = getEmptyStack(octantLinear.childIndex[c]);
                push_back(&stackVector, &tempStack);
                found = true;
                break;
            }
        }
        if(found) continue;

        // Return
        lastReturn = inside(&query, maxDistance, &octantLinear);
        pop_back(&stackVector);
    }

    if(DEBUG) printf("ANS %d\n", resultIndex);
    return resultIndex;

    return 0;
}

__kernel void nearestNeighbourKernel(__global PointT* data, __global OctantLinear* octantLinearVector, int octantLinearVectorSize, __global int* successors, __global PointT* queries, __global int* indices)
{
    // Get point
    PointT query = queries[get_global_id(0)];

    // Search
    int resultIndex = nearestNeighbourSearch(data, octantLinearVector, octantLinearVectorSize, successors, &query);

    // Set back
    indices[get_global_id(0)] = resultIndex;
}
