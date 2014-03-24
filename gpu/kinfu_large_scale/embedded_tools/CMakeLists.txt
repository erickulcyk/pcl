## KINECT FUSION
set(the_target pcl_kinfu_largeScale)
set(srcs kinfuLS_app.cpp capture.cpp evaluation.cpp)

source_group("Source Files" FILES ${srcs} )

PCL_ADD_EXECUTABLE_OPT_BUNDLE("${the_target}" "${SUBSYS_NAME}" ${srcs} ${hdrs})
target_link_libraries("${the_target}" pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_gpu_kinfu_large_scale pcl_octree)
