mkdir build
cd build
cmake ..
cmake --build . --config Debug --target OpenPS
cmake --build . --config Release --target OpenPS
cmake --build . --config Debug --target Example
cmake --build . --config Release --target Example