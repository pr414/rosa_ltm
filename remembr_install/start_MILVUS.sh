sed -i 's/sudo docker/docker/g' launch_milvus_container.sh
apt-get update && apt-get install -y docker.io
./launch_milvus_container.sh start