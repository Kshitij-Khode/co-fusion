if [ "${1}" = "--build" ]; then

    export BOOST_ROOT="/home/kshitij/codeSandbox/repos/co-fusion/deps/boost/"
    export OpenCV_DIR="/home/kshitij/codeSandbox/repos/co-fusion/deps/opencv-3.1.0/build"
    export Pangolin_DIR="/home/kshitij/codeSandbox/repos/co-fusion/deps/Pangolin/build"

    cd ../build/

    cmake \
      -DBOOST_ROOT="${BOOST_ROOT}" \
      -DOpenCV_DIR="${OpenCV_DIR}" \
      -DPangolin_DIR="${Pangolin_DIR}" \
      -DCMAKE_BUILD_TYPE=Debug \
      ..
    make -j8

elif [ "${1}" = "--run" ]; then
    ../build/GUI/./CoFusion -realSense /home/kshitij/codeSandbox/repos/co-fusion/build/Datasets/testD435.bag -run
fi
