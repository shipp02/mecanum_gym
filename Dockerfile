FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
		       openssh-server \
		       fzf \
		       ripgrep \
		       neovim \
                       ros-jazzy-rviz2 \
		       lsb-release \
		       gnupg

RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN sudo apt-get install -y gz-harmonic

RUN apt-get -y dist-upgrade

RUN mkdir /sim_ws

COPY ./ws_entrypoint.sh /ws_entrypoint.sh
RUN chmod +x /ws_entrypoint.sh

RUN apt install -y gosu \
		   sudo \
		   udev

# USER ubuntu
RUN . /opt/ros/jazzy/setup.bash
RUN ls > $(mktemp)
COPY . /sim_ws/
RUN  <<EOF
. /sim_ws/install/setup.bash
cd /sim_ws 
rosdep update
ls
rosdep install -y --from-paths src --ignore-src
EOF

USER root
RUN rm -r /sim_ws
RUN mkdir /sim_ws


WORKDIR '/sim_ws'

