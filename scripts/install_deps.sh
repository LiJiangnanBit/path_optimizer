#!/bin/bash
set -e
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../" && pwd )"
TMP_DIR="/tmp"

install_ipopt() {
    echo "Prepare to install IPOPT ..."
    IPOPT_URL="https://e.coding.net/tmp-code/ipopt-3.12.4.git"
    sudo apt-get -y install \
        gfortran \
        cmake  \
        build-essential \
        gcc \
        g++
    sudo ldconfig
    if ( ldconfig -p | grep libipopt ); then
        echo "Ipopt is already installed......."
    else
        echo "Start installing Ipopt, version: 3.12.4  .........."
        pwd
        cd $TMP_DIR
        pwd
        rm -rf ipopt-3.12.4 && git clone "$IPOPT_URL" && cd ipopt-3.12.4
        # configure,build and install the IPOPT
        echo "Configuring and building IPOPT ..."
        ./configure --prefix /usr/local
        make -j$(nproc)
        make test
        sudo make install
        if (grep 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' $HOME/.bashrc); then
          echo "LD_LIBRARY_PATH has been set."
        else
          echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> $HOME/.bashrc
        fi
        sudo ldconfig
        echo "IPOPT installed successfully"
        source $HOME/.bashrc
    fi
    cd $REPO_DIR
}

install_cppad() {
    echo "Prepare to install CppAD ..."
    CppAD="cppad"
    VERSION="20180000.0"
    CppAD_URL="http://www.coin-or.org/download/source/CppAD/$CppAD-$VERSION.gpl.tgz"
    TEMP_DIR=$(mktemp -d)
    #sudo apt-get -qq install cmake
    if ( ls /usr/include | grep cppad );then
        echo "cppad is already installed......"
    else
        cd $TEMP_DIR
        wget $CppAD_URL
        tar -xf $CppAD-$VERSION.gpl.tgz
        rm -f $CppAD-$VERSION.gpl.tgz
        mkdir -p $CppAD-$VERSION/build
        cd $CppAD-$VERSION/build
        cmake \
            -D cppad_cxx_flags="-Wall -ansi -pedantic-errors -std=c++11 -Wshadow" \
            ..
        sudo make install
        echo "CppAD installed successfully"
    fi
    cd $TMP_DIR
    rm -rf $TEMP_DIR
    sudo ldconfig  
    cd $REPO_DIR
}

install_benchmark() {
    echo "Prepare to install google benchmark"
    TEMP_DIR=$(mktemp -d)
    if ( ls /usr/local/include | grep benchmark);then
        echo "benchmark is already installed......"
    else
        cd $TEMP_DIR
        git clone https://github.com/google/benchmark.git
        cd benchmark
        git clone https://github.com/google/googletest.git
        mkdir build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=RELEASE
        make -j$(nproc) 
        sudo make install
        echo "benchmark installed successfully"
    fi
    cd $TMP_DIR
    rm -rf $TEMP_DIR
    sudo ldconfig 
    cd $REPO_DIR   
}

install_osqp_eigen() {
    sudo apt-get install libeigen3-dev
    echo "eigen3 is installed successfully!"

    echo "Prepare to install osqp"
    TEMP_DIR=$(mktemp -d)
    if ( ls /usr/local/include | grep osqp);then
        echo "osqp is already installed......"
    else
        cd $TEMP_DIR
        git clone --recursive https://github.com/oxfordcontrol/osqp
        cd osqp
        mkdir build && cd build
        cmake -G "Unix Makefiles" ..
        cmake --build .
        sudo cmake --build . --target install
        echo "osqp installed successfully"
    fi
    sudo ldconfig 
    echo "Prepare to install osqp-eigen"
    if ( ls /usr/local/include | grep OsqpEigen);then
        echo "OsqpEigen is already installed......"
    else
        cd $TEMP_DIR
        git clone https://github.com/robotology/osqp-eigen.git
        cd osqp-eigen
        mkdir build && cd build
        cmake ../
        make -j$(nproc) 
        sudo make install
        echo "osqp-eigen installed successfully"
    fi
    cd $TMP_DIR
    rm -rf $TEMP_DIR
    sudo ldconfig 
    cd $REPO_DIR   
}

install_grid_map() {
    sudo apt-get -y install ros-kinetic-pcl-ros ros-kinetic-costmap-2d ros-kinetic-grid-map
    echo "grid_map is installed successfully!"
}

install_glog() {
    sudo apt-get install libgoogle-glog-dev
    echo "glog is installed successfully!"
}

install_gflags() {
    sudo apt-get install libgflags-dev
    echo "gflags is installed successfully!"
}

clone_other_ros_pkgs() {
    cd $REPO_DIR/..
    if (ls $REPO_DIR/.. | grep ros_viz_tools); then
        echo "ros_rviz_tools already exist!"
    else 
        git clone https://github.com/Magic-wei/ros_viz_tools
    fi
    if (ls $REPO_DIR/.. | grep tinyspline_ros);then
        echo "tinyspline_ros already exist!"
    else
        git clone https://github.com/qutas/tinyspline_ros
    fi
}

main() {
    #sudo apt-get update
    install_ipopt
    install_cppad
    install_benchmark
    install_glog
    install_gflags
    install_grid_map
    install_osqp_eigen
    clone_other_ros_pkgs
}

main

