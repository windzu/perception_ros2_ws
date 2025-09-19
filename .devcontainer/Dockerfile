# 1. 基于你选择的 NVIDIA 基础镜像
FROM nvcr.io/nvidia/tensorrt:23.10-py3

# 设置环境变量，防止 apt-get 交互式弹窗
ENV DEBIAN_FRONTEND=noninteractive

# 2. 安装 ROS 2 (Humble) 的基础依赖
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    locales \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# 3. 设置 Locale
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 4. 添加 ROS 2 官方源
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 5. 安装 ROS 2 Humble (Desktop Full) 和开发工具
RUN apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    # Colcon 和其他编译工具
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    # 常用开发工具
    build-essential \
    git \
    vim

# 6. 安装一些常用的系统工具
RUN apt-get update && apt-get install -y \
    bash-completion \
    libcgal-dev \
    python3-vcstool

# 7. 安装其他常用 ROS 2 包（根据需要添加）
RUN apt-get update && apt-get install -y \
    librange-v3-dev \
    ros-humble-navigation2 \
    ros-humble-sophus \
    ros-humble-geographic-msgs \
    ros-humble-proxsuite \
    ros-humble-osqp-vendor \
    ros-humble-lanelet2 \
    ros-humble-point-cloud-msg-wrapper \
    ros-humble-ros-testing \
    ros-humble-tl-expected \
    ros-humble-generate-parameter-library \
    && rm -rf /var/lib/apt/lists/* \
    && rm /etc/apt/apt.conf.d/docker-clean

# 8. pip 安装一些 Python 包


# 初始化 rosdep (作为 root)
RUN rosdep init || true
RUN rosdep update

# --- 创建非 root 用户 (关键部分) ---
# 接收来自 devcontainer.json 的构建参数
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=ros

# 创建组和用户
RUN groupadd --gid $USER_GID $USERNAME
RUN useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME

# 授予 sudo 权限
RUN echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME
RUN chmod 0440 /etc/sudoers.d/$USERNAME

# 忽略一些路径
ENV CMAKE_IGNORE_PREFIX_PATH="/opt/hpcx"

# (可选) 授予对 video/audio 设备的访问权限
RUN usermod -aG video $USERNAME
RUN usermod -aG audio $USERNAME

# 在切换到普通用户前创建 /workspace 并授予权限
RUN mkdir -p /workspace && chown $USER_UID:$USER_GID /workspace

#  --- 切换到新用户 ---
USER $USERNAME
ENV HOME=/home/$USERNAME

# 为新用户创建工作空间目录
RUN mkdir -p /workspace/src
WORKDIR /workspace

# 9. 在新用户的 .bashrc 中配置 ROS 环境
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /home/$USERNAME/.bashrc