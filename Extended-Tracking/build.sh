echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../../lib/SRC

echo "Building ARToolKit libs ..."
make -j4

cd ../../

echo "Building ORB-SLAM2 lib and then Extended-tracking example ..."
mkdir build
cd build
cmake ..
make -j4
