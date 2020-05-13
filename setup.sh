PROJECT_SOURCE_DIR=$(cd $(dirname ${BASH_SOURCE[0]:-${(%):-%x}})/; pwd)
PROJECT_BINARY_DIR=${PROJECT_SOURCE_DIR}/build

# default build tool
build_tool="make"

docker_image="aidrive"

lunch() {
    echo "running lunch ..."

    local build_dir="${PROJECT_BINARY_DIR}"
    local cmake_cmd="cmake .."

    cd $build_dir && \
    ${cmake_cmd}
}

dkb() {
    echo "running dkb ..."

    CMD="docker run -it --rm\
        --runtime=nvidia
        -v ${HOME}/.Xauthority:/home/user/.Xauthority \
        -v $PROJECT_SOURCE_DIR:$PROJECT_SOURCE_DIR -v $PROJECT_BINARY_DIR:$PROJECT_BINARY_DIR \
        -w `realpath $PWD` -u $(id -u):$(id -g) --rm \
        -e DISPLAY \
        --net=host \
        $docker_image \
        bash -c \"$*\""

    echo ${CMD}

    eval ${CMD}
}