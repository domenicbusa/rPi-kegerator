# docker-rpi-gpio
A  Docker container with RPi.GPIO installed to use the Raspberry Pi's GPIO pins

docker build -t control:[revision_tag] .
docker run --device /dev/ttyACM0:/dev/ttyACM0 --privileged -d --restart always --name control_v3 control:03MAR2022
