echo "Cleaning Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2/build
make clean

cd ../../g2o

echo "Cleaning Thirdparty/g2o ..."
cd build
make clean

cd ../../../lib/SRC

echo "Cleaning ARToolKit libs ..."
make clean

cd ../../build

echo "Cleaning ORB-SLAM2 libs and Extended-tracking example ..."
make clean