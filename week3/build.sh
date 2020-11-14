echo "Configuring and building Thirdparty/libnabo ..."

cd Thirdparty/libnabo
mkdir build
cd build
cmake .. -DSHARED_LIBS=true
make -j4

cd ../../../

echo "Configuring and building clustering ..."

mkdir build
cd build
cmake .. -DPYTHON_EXECUTABLE=$(which python)
make -j4