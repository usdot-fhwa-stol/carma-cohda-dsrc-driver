# Run initial dependency installation
ARG DOCKER_ORG="usdotfhwastoldev"
ARG DOCKER_TAG="develop"
FROM ${DOCKER_ORG}/carma-base:${DOCKER_TAG} AS base_image
FROM base_image AS setup
ARG GIT_BRANCH="develop"

ARG ROS1_PACKAGES=""
ENV ROS1_PACKAGES=${ROS1_PACKAGES}
ARG ROS2_PACKAGES=""
ENV ROS2_PACKAGES=${ROS2_PACKAGES}

RUN mkdir ~/src
COPY --chown=carma . /home/carma/src/
RUN ~/src/docker/checkout.bash -b ${GIT_BRANCH}
RUN ~/src/docker/install.sh
