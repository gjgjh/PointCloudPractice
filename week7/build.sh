echo "Configuring and building Thirdparty/libnabo ..."

cd Thirdparty/libnabo
mkdir build
cd build
cmake .. -DSHARED_LIBS=true
make -j4

cd ../../../

echo "Configuring and building ISSEstimator ..."

mkdir build
cd build
cmake ..
make -j4