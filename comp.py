g++ test.cpp -std=c++11 -o output $(pkg-config --cflags --libs opencv)
./output


g++ cam.cpp -std=c++11 -o outcam $(pkg-config --cflags --libs opencv)
./outcam


v4l2-ctl --set-ctrl=exposure_auto=1


