% foveated stereo mex compilation
mex foveated_stereo_interface_mex.cpp mc_convert/mc_convert.cpp cpp/PeripheralFovealStereo.cpp cpp/stereo_sensors/PeripheralFoveal.cpp cpp/stereo_sensors/Uniform.cpp cpp/Stereo.cpp cpp/stereo_sensors/StereoData.cpp cpp/stereo_sensors/StereoSensor.cpp cpp/stereo_sensors/Periphery.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_contrib -lopencv_calib3d -lpcl_common -I/usr/include/pcl-1.7 -I/usr/include/eigen3

% conventional stereo mex compilation
mex conventional_stereo_interface_mex.cpp mc_convert/mc_convert.cpp cpp/ConventionalStereo.cpp cpp/stereo_sensors/Cartesian.cpp cpp/stereo_sensors/Uniform.cpp cpp/Stereo.cpp cpp/stereo_sensors/StereoData.cpp cpp/stereo_sensors/StereoSensor.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_contrib -lopencv_calib3d -lpcl_common -I/usr/include/pcl-1.7 -I/usr/include/eigen3
