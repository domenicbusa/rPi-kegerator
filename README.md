# rPi-kegerator

docker build -t control:[revision_tag] dockerfile
docker run --privileged -d --restart always --name [container_name] control:[revisionTag]
