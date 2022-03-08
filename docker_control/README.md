# docker-rpi-gpio
A  Docker container with RPi.GPIO installed to use the Raspberry Pi's GPIO pins

docker build -t control:[revision_tag] .
docker run --device /dev/ttyUSB0:/dev/ttyUSB0 --privileged -d --restart always --name control_v4 control:08MAR2022
