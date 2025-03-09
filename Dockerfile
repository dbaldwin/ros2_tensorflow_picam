FROM arm64v8/ros:jazzy

# Set environment variables  
ENV VIRTUAL_ENV=/opt/venv  
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
ENV PYTHONPATH="$PYTHONPATH:$VIRTUAL_ENV/lib/python3.12/site-packages"

# Update and install necessary packages
RUN sudo apt update -y && \
    sudo apt install -y python3-pip python3.12-venv libopencv-dev python3-opencv git

# Create and activate Python virtual environment
RUN python3 -m venv $VIRTUAL_ENV --system-site-packages

# Set working directory
WORKDIR /app

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Clone ROS2 repositories
RUN mkdir -p src && \
    cd src && \
    git clone https://github.com/dbaldwin/ros2_tensorflow_picam.git && \
    git clone https://github.com/ros-perception/vision_opencv.git

# Install dependencies for ROS2 packages
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the ROS2 packages
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select ros2_tensorflow_picam cv_bridge

COPY entrypoint.sh /app/
RUN chmod +x /app/entrypoint.sh

ENTRYPOINT ["/app/entrypoint.sh"]