FROM arm64v8/ros:jazzy

sudo apt update

sudo apt install python3-pip

sudo apt install python3.12-venv

python3 -m venv venv

source venv/bin/activate

pip install tensorflow