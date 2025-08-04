FROM ubuntu:latest


ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Step 3: Install core dependencies, add ROS 2 and Bazel repositories
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Step 4: Install ROS 2 Iron
# Following the official ROS 2 documentation
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Step 5: Install Bazel 8.3.1 and other required tools (lcov, clang)
RUN apt-get update && apt-get install -y apt-transport-https
RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > /etc/apt/trusted.gpg.d/bazel.gpg
RUN echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list
RUN apt-get update && apt-get install -y \
    bazel \
    gcc-14 \
    g++-14 \
    libdatetime-perl \
    libtimedate-perl \
    libcapture-tiny-perl \
    && rm -rf /var/lib/apt/lists/*

# Set gcc-14 and g++-14 as default
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 100 \
    && update-alternatives --install /usr/bin/gcov gcov /usr/bin/gcov-14 100

# Install lcov
RUN curl -L https://github.com/linux-test-project/lcov/releases/download/v2.3.1/lcov-2.3.1.tar.gz | tar xz \
    && cd lcov-2.3.1 \
    && make install \
    && cd .. && rm -rf lcov-2.3.1

#RUN curl -L -o /usr/local/bin/bazel "https://github.com/bazelbuild/bazelisk/releases/download/v${BAZELISK_VERSION}/bazelisk-linux-amd64" \
#    && chmod +x /usr/local/bin/bazel

# Step 6: Configure the environment
# Automatically source ROS 2 for every new shell session.
# This removes the need for `source /opt/ros/.../setup.bash` in your CI scripts.
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "export LCOVRC=/path/to/.lcovrc" >> /etc/bash.bashrc

# Step 7: Define the entrypoint for the container
# This ensures commands are run within a bash shell that has sourced the setup files.
CMD ["/bin/bash"]