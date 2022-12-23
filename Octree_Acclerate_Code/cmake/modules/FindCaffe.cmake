# Caffe package for CNN Triplet training
unset(Caffe_FOUND)

find_path(CAFFE_INCLUDE_DIRS NAMES caffe/caffe.hpp caffe/common.hpp caffe/net.hpp caffe/proto/caffe.pb.h caffe/util/io.hpp
  HINTS
  /usr/local/include)

find_library(CAFFE_LIBRARIES NAMES caffe
  HINTS
  /usr/local/lib)

if(CAFFE_LIBRARIES AND CAFFE_INCLUDE_DIRS)
    set(Caffe_FOUND 1)
endif()
