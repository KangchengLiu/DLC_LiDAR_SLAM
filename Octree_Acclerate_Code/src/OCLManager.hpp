#define __CL_ENABLE_EXCEPTIONS

#include <fstream>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <numeric>
#include <string>
#include <map>
#include <boost/algorithm/string.hpp>

#ifdef __APPLE__
#include <OpenCl/cl.hpp>
#else
#include <CL/cl.hpp>
#endif

#ifndef __OCLManager__
#define __OCLManager__

#include <pcl/io/pcd_io.h>

using namespace std;

class Parser
{
public:
    Parser(std::string& type) : _type(type)
    {
        this->point = "PointT";
        std::cout << "Parsing OpenCL kernel for " << type << std::endl;
    }
    inline void setFile(std::string& filename, std::string& path, std::string& _struct)
    {
        // Support multiple files?
        std::vector<std::string> files;
        boost::split(files,filename,boost::is_any_of(" "));
        if(files.size() == 1){
            std::ifstream programFile((char*) (path+filename).c_str());
            std::string programString(std::istreambuf_iterator<char>(programFile),
                                      (std::istreambuf_iterator<char>()));
            if(!programString.size()) cerr << filename << " is empty" << endl;
            program = _struct + ReplaceString(programString);
        }
        else{
            program = "";
            program += _struct;
            for(std::string& file : files){
                std::ifstream programFile((char*) (path+file).c_str());
                std::string programString(std::istreambuf_iterator<char>(programFile),
                                          (std::istreambuf_iterator<char>()));
                if(!programString.size()) cerr << file << " is empty" << endl;
                program += ReplaceString(programString);
            }
        }
    }

    inline std::string ReplaceString(std::string subject) {
        size_t pos = 0;
        while ((pos = subject.find(point, pos)) != std::string::npos) {
            subject.replace(pos, point.length(), _type);
            pos += _type.length();
        }
        return subject;
    }

    inline std::string getProgram()
    {
        return program;
    }
private:
    std::string _type, program;
    std::string point;// = "PointT";

};

class OCLBuilder
{
public:
    virtual void configureFile(std::string&, std::string&) = 0;
    Parser *getResult()
    {
        return _result;
    }
protected:
    Parser *_result;
};

class XYZOCLBuilder: public OCLBuilder
{
public:
    XYZOCLBuilder()
    {
        type = "PointXYZ";
        _struct = "typedef struct {\n union {\n float data[4];\n struct {\n float x; \n float y; \n float z; \n float w; \n }; \n }; \n } PointXYZ; \n\n";
        _result = new Parser(type);
    }
    inline void configureFile(std::string& filename, std::string& path)
    {
        _result->setFile(filename, path, _struct);
    }
private:
    std::string type;
    std::string _struct;
};

class XYZRGBAOCLBuilder: public OCLBuilder
{
public:
    XYZRGBAOCLBuilder()
    {
        type = "PointXYZRGBA";
        _struct = "typedef struct {\n union {\n float data[4];\n float rgba[4];\n struct {\n float x;\n float y;\n float z;\n float w;\n};\n struct {\n float r;\n float g;\n float b;\n float a;\n};\n};\n} PointXYZRGBA;\n\n";
        _result = new Parser(type);
    }
    inline void configureFile(std::string& filename, std::string& path)
    {
        _result->setFile(filename, path, _struct);
    }
private:
    std::string type;
    std::string _struct;
};

class XYZRGBNormalOCLBuilder: public OCLBuilder
{
public:
    XYZRGBNormalOCLBuilder()
    {
        type = "PointXYZRGBNormal";
        _struct = "typedef struct {\nunion {\nfloat data[4];\nstruct {\n    float x;\n    float y;\n    float z;\n    float w;\n};\n};\nunion {\nfloat normals[4];\nstruct {\n    float normal_x;\n    float normal_y;\n    float normal_z;\n    float normal_w;\n};\n};\nunion {\nfloat rgba;\nstruct {\n    uchar b;\n    uchar g;\n    uchar r;\n    uchar a;\n};\n};\nfloat curvature;\nfloat free[2];\n} PointXYZRGBNormal;\n\n";
        _result = new Parser(type);
    }
    inline void configureFile(std::string& filename, std::string& path)
    {
        _result->setFile(filename, path, _struct);
    }
private:
    std::string type;
    std::string _struct;
};

class Reader
{
public:
    inline void setOCLBuilder(OCLBuilder *b)
    {
        _OCLBuilder = b;
    }
    inline  void construct(std::string& filename, std::string& path);
private:
    OCLBuilder *_OCLBuilder;
};

inline void Reader::construct(std::string& filename, std::string& path)
{
    _OCLBuilder->configureFile(filename, path);
}

class OCLManager {
    private:
        std::map<std::string, cl::Program> clPrograms;
        std::map<std::string, cl::Kernel> clKernels;
        std::string clPath;
        vector<cl::Platform> platforms;
        vector<cl::Device> devices;
        cl::Context context;
        cl::CommandQueue queue;
        enum Cloud { XYZ, XYZRGBA, XYZRGBNormal};
        void initCL();
        void createDevicesAndContext();
        static OCLManager* m_pInstance;

    public:
        OCLManager() {}
        ~OCLManager(){}
        static OCLManager* getInstance();
        void setCLPath(std::string path);
        void destroyInstance();
        cl::Context getContext();
        cl::CommandQueue getQueue();
        cl::Program buildProgramFromSource(string filename);
        cl::Program buildProgramFromSource(string filename, OCLManager::Cloud cloudType);
        cl::Program buildProgramFromBinary(string filename);
        void saveBinary(cl::Program* program, string filename);
        cl::Device getDevice();

    public:

        template<typename PointT>
        bool buildKernelIntoManager(string filename, std::string kernelname){
            // Type
            OCLManager::Cloud cloudType;
            if((std::is_same<PointT, pcl::PointXYZ>::value)) cloudType = XYZ;
            else if((std::is_same<PointT, pcl::PointXYZRGBA>::value)) cloudType = XYZRGBA;
            else if((std::is_same<PointT, pcl::PointXYZRGBNormal>::value)) cloudType = XYZRGBNormal;

            // Program not built
            if (!(clPrograms.find(filename) != clPrograms.end()))
            {
                clPrograms[filename] = buildProgramFromSource(filename,cloudType);
            }

            cl::Program& program = clPrograms[filename];

            // Kernel not built
            if (!(clKernels.find(kernelname) != clKernels.end()))
            {
                clKernels[kernelname] = cl::Kernel(program, kernelname.c_str());
            }
            else cout << "Kernel already built: " << kernelname << endl;
        }

        cl::Kernel& getKernelFromManager(std::string kernelname){
            if (!(clKernels.find(kernelname) != clKernels.end()))
            {
                cerr << "Kernel not found: " << kernelname << endl;
                throw;
            }
            return clKernels[kernelname];
        }

        template<typename PointT>
        cl_int writePointCloudToGPU(const pcl::PointCloud<PointT>& pointCloud, cl::Buffer& bufferCloud){
            cl_int err;
            bufferCloud = cl::Buffer(getContext(), CL_MEM_READ_WRITE, sizeof(PointT) * pointCloud.size(), NULL, &err);
            err = getQueue().enqueueWriteBuffer(bufferCloud, CL_TRUE, 0, sizeof(PointT) * pointCloud.size(), const_cast<float*> (&pointCloud.points[0].x), NULL);
            return err;
        }

        template<typename PointT>
        cl_int writePointCloudToGPU(int size, cl::Buffer& bufferCloud){
            cl_int err;
            bufferCloud = cl::Buffer(getContext(), CL_MEM_READ_WRITE, sizeof(PointT) * size, NULL, &err);
            return err;
        }

        template<typename PointT>
        cl_int readPointCloudFromGPU(const cl::Buffer& bufferCloud, pcl::PointCloud<PointT>& pointCloud){
            cl_int err;
            err = getQueue().enqueueReadBuffer(bufferCloud, CL_TRUE, 0, sizeof(PointT) * pointCloud.size(), static_cast<float*> (&pointCloud.points[0].x));
            return err;
        }

};

OCLManager* OCLManager::m_pInstance = NULL;

OCLManager* OCLManager::getInstance() {
    if(NULL == m_pInstance ) {
        m_pInstance = new OCLManager();
        m_pInstance->initCL();
    }
    return m_pInstance;
}
void OCLManager::setCLPath(std::string path){
    clPath = path;
}
void OCLManager::destroyInstance() {
    delete m_pInstance;
    m_pInstance = NULL;
}
void OCLManager::initCL() {
    this->createDevicesAndContext();
    cl::CommandQueue q(context, devices[0], CL_QUEUE_PROFILING_ENABLE);
    queue = q;
}

cl::Device OCLManager::getDevice(){
    return devices[0];
}

void OCLManager::createDevicesAndContext()
{
    vector<cl::Device> gpu_devices, cpu_devices, acc_devices;
    std::string device_name;
    cl_uint i, type;
    try {
        //Access all devices in first platform
        cl::Platform::get(&platforms);
        type = platforms[0].getDevices(CL_DEVICE_TYPE_GPU, &devices);
        if( type == CL_SUCCESS)
        {
            //Create context and access device names
            cl::Context ctx_(devices);
            context = ctx_;
            gpu_devices = context.getInfo<CL_CONTEXT_DEVICES>();
            for(i=0; i<gpu_devices.size(); i++) {
                device_name = gpu_devices[i].getInfo<CL_DEVICE_NAME>();
                cout << "Device: " << device_name.c_str() << endl;
            }
        }
        if(type == CL_INVALID_DEVICE_TYPE || type == CL_DEVICE_NOT_FOUND){
            // Access all devices in first platform
            cl::Platform::get(&platforms);
            type = platforms[0].getDevices(CL_DEVICE_TYPE_CPU, &devices);
            if( type == CL_SUCCESS)
            {
                // Create context and access device names
                cl::Context ctx_(devices);
                context = ctx_;
                cpu_devices = context.getInfo<CL_CONTEXT_DEVICES>();
                for(i=0; i<cpu_devices.size(); i++) {
                    device_name = cpu_devices[i].getInfo<CL_DEVICE_NAME>();
                    cout << "Device: " << device_name.c_str() << endl;
                }
            }
        }
        if(type == CL_INVALID_DEVICE_TYPE || type == CL_DEVICE_NOT_FOUND){
            // Access all devices in first platform
            cl::Platform::get(&platforms);
            type = platforms[0].getDevices(CL_DEVICE_TYPE_ACCELERATOR, &devices);
            cout << "ACC: " << type << endl;
            if( type == CL_SUCCESS)
            {
                // Create context and access device names
                cl::Context ctx__(devices);
                context = ctx__;
                acc_devices = context.getInfo<CL_CONTEXT_DEVICES>();
                for(i=0; i<acc_devices.size(); i++) {
                    device_name = acc_devices[i].getInfo<CL_DEVICE_NAME>();
                    cout<<" "<<endl<<endl<<endl;
                    cout << "Device: " << device_name.c_str() << endl;
                }
            }
        }
        if(type == CL_INVALID_DEVICE_TYPE || type == CL_DEVICE_NOT_FOUND)
            cerr<<"No OpenCL device found!!"<<endl;
    }
    catch(cl::Error e) {
        cout << e.what() << ": Error code " << e.err() << endl;
    }
}

cl::Context OCLManager::getContext(){
    return context;
}
cl::CommandQueue OCLManager::getQueue(){
    return queue;
}

cl::Program OCLManager::buildProgramFromSource(std::string filename) {
    cl::Program program;
    try{
        std::ifstream programFile((char*) filename.c_str());
        std::string programString(std::istreambuf_iterator<char>(programFile),
                                  (std::istreambuf_iterator<char>()));
        cl::Program::Sources source(1, std::make_pair(programString.c_str(),
                                                      programString.length()+1));
        cl::Program prog(context, source);
        prog.build(devices);
        program = prog;
    }
    catch(cl::Error e) {
        cout << e.what() << ": Error code " << e.err() << endl;
        exit(-1);
    }
    return program;
}

cl::Program OCLManager::buildProgramFromSource(std::string filename, OCLManager::Cloud cloudType) {
    printf("%s\n",filename.c_str());

    std::string src;
    Reader reader;
    switch(cloudType) {
        case XYZRGBNormal:
        {
            XYZRGBNormalOCLBuilder XYZRGBNormalOCLBuilder;
            reader.setOCLBuilder(&XYZRGBNormalOCLBuilder);
            reader.construct(filename, clPath);
            src.append(XYZRGBNormalOCLBuilder.getResult()->getProgram());
        }
        break;

        case XYZRGBA:
        {
            XYZRGBAOCLBuilder XYZRGBAOCLBuilder;
            reader.setOCLBuilder(&XYZRGBAOCLBuilder);
            reader.construct(filename, clPath);
            src.append(XYZRGBAOCLBuilder.getResult()->getProgram());
        }
        break;

        case XYZ:
        {
            XYZOCLBuilder XYZOCLBuilder;
            reader.setOCLBuilder(&XYZOCLBuilder);
            reader.construct(filename, clPath);
            src.append(XYZOCLBuilder.getResult()->getProgram());
        }
        break;

        default:
            cerr << "Type is not supported by OpenCL module"  << endl;
            exit(-1);
    }
    cl::Program program;
    try{
        cl::Program::Sources source(1, std::make_pair(src.c_str(),
                                                      src.length()+1));
        program = cl::Program(context, source);
        program.build(devices);
        //program = prog;
    }
    catch(cl::Error e) {
        cout << e.what() << ": Error code " << e.err() << endl;
        cerr << program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(getDevice()) << endl;
        cerr << program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(getDevice()) << endl;
        cerr << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(getDevice()) << endl;
        exit(-1);
    }
    return program;
}

cl::Program OCLManager::buildProgramFromBinary(std::string filename)
{
    cl::Program::Binaries binary;
    cl::Program program;
    FILE *fp = fopen((char*) filename.c_str(), "rb");
    size_t binarySize, read;
    fseek(fp, 0, SEEK_END);
    binarySize = ftell(fp);
    rewind(fp);
    unsigned char *programBinary = new unsigned char[binarySize];

    read = fread(programBinary, 1, binarySize, fp);
    if(read==0)
        throw cl::Error(-999, "Empty binary!");
    binary.insert(binary.begin(), std::make_pair(programBinary, binarySize));
    program = cl::Program(context, devices, binary);
    program.build(devices);
    fclose(fp);
    return program;
}


void OCLManager::saveBinary(cl::Program* program, std::string filename){
    // Allocate some memory for all the kernel binary data
    const std::vector<unsigned long> binSizes = program->getInfo<CL_PROGRAM_BINARY_SIZES>();
    std::vector<char> binData (std::accumulate(binSizes.begin(),binSizes.end(),0));

    char* binChunk = &binData[0] ;
    //A list of pointers to the binary data
    std::vector<char*> binaries;

    for(unsigned int i = 0; i<binSizes.size(); ++i)
    {
        binaries.push_back(binChunk) ;
        binChunk += binSizes[i] ;
    }
    std::cout<<"Program name: " << (char*) filename.c_str() << std::endl;
    program->getInfo(CL_PROGRAM_BINARIES , &binaries[0] ) ;
    std::ofstream binaryfile((char*) filename.c_str(), std::ios::binary);
    for (unsigned int i = 0; i < binaries.size(); ++i)
        binaryfile.write(binaries[i], binSizes[i]);
}

#endif
