#!/usr/bin/env bash

####### RUN WITH "bash milvius_patch.sh start" ##################
## TO REMOVE DOCKER CONTAINER IF ISSUES:
# # Stop and remove the container
#sudo docker stop milvus-standalone 2>/dev/null
#sudo docker rm milvus-standalone 2>/dev/null
# Remove the old image
#sudo docker rmi milvusdb/milvus:v2.6.4 2>/dev/null
#sudo docker rmi $(sudo docker images | grep milvus | awk '{print $3}') 2>/dev/null
# Clean up data and config files
#sudo rm -rf volumes embedEtcd.yaml user.yaml
# Clean Docker cache
#sudo docker system prune -f

# Licensed to the LF AI & Data foundation under one
# or more contributor license agreements. See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership. The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

run_embed() {
    # Create volumes directory with proper permissions
    mkdir -p $(pwd)/volumes/milvus
    chmod 755 $(pwd)/volumes/milvus
    
    # Create embedEtcd.yaml
    cat << EOF > $(pwd)/embedEtcd.yaml
listen-client-urls: http://0.0.0.0:2379
advertise-client-urls: http://0.0.0.0:2379
quota-backend-bytes: 4294967296
auto-compaction-mode: revision
auto-compaction-retention: '1000'
EOF

    # Create user.yaml
    cat << EOF > $(pwd)/user.yaml
# Extra config to override default milvus.yaml
EOF

    # Set proper permissions
    chmod 644 $(pwd)/embedEtcd.yaml
    chmod 644 $(pwd)/user.yaml

    # Verify files exist
    if [ ! -f "$(pwd)/embedEtcd.yaml" ]; then
        echo "Error: embedEtcd.yaml file was not created successfully."
        exit 1
    fi

    if [ ! -f "$(pwd)/user.yaml" ]; then
        echo "Error: user.yaml file was not created successfully."
        exit 1
    fi
    
    echo "Configuration files created successfully."
    echo "Starting Milvus container..."
    
    # Pull the latest image first to avoid using cached old versions
    sudo docker pull milvusdb/milvus:v2.4.15
    
    sudo docker run -d \
        --name milvus-standalone \
        --security-opt seccomp:unconfined \
        -e ETCD_USE_EMBED=true \
        -e ETCD_DATA_DIR=/var/lib/milvus/etcd \
        -e ETCD_CONFIG_PATH=/milvus/configs/embedEtcd.yaml \
        -e COMMON_STORAGETYPE=local \
        -e DEPLOY_MODE=STANDALONE \
        -v $(pwd)/volumes/milvus:/var/lib/milvus:rw \
        -v $(pwd)/embedEtcd.yaml:/milvus/configs/embedEtcd.yaml:ro \
        -v $(pwd)/user.yaml:/milvus/configs/user.yaml:ro \
        -p 19530:19530 \
        -p 9091:9091 \
        -p 2379:2379 \
        --health-cmd="curl -f http://localhost:9091/healthz" \
        --health-interval=30s \
        --health-start-period=90s \
        --health-timeout=20s \
        --health-retries=3 \
        milvusdb/milvus:v2.4.15 \
        milvus run standalone
    
    if [ $? -ne 0 ]; then
        echo "Failed to start container. Checking logs..."
        sudo docker logs milvus-standalone 2>&1 | tail -20
        return 1
    fi
}

wait_for_milvus_running() {
    echo "Wait for Milvus Starting..."
    local max_wait=300  # 5 minutes timeout
    local waited=0
    
    while [ $waited -lt $max_wait ]; do
        res=$(sudo docker ps 2>/dev/null | grep milvus-standalone | grep healthy | wc -l)
        if [ $res -eq 1 ]; then
            echo "Start successfully."
            echo "To change the default Milvus configuration, add your settings to the user.yaml file and then restart the service."
            return 0
        fi
        
        # Check if container exited
        exited=$(sudo docker ps -a 2>/dev/null | grep milvus-standalone | grep -i "exited" | wc -l)
        if [ $exited -eq 1 ]; then
            echo "Container exited unexpectedly. Showing logs:"
            sudo docker logs milvus-standalone 2>&1 | tail -30
            return 1
        fi
        
        sleep 2
        waited=$((waited + 2))
        if [ $((waited % 10)) -eq 0 ]; then
            echo "Still waiting... (${waited}s elapsed)"
        fi
    done
    
    echo "Timeout waiting for Milvus to become healthy."
    echo "Container logs:"
    sudo docker logs milvus-standalone 2>&1 | tail -30
    return 1
}

start() {
    res=$(sudo docker ps 2>/dev/null | grep milvus-standalone | grep healthy | wc -l)
    if [ $res -eq 1 ]; then
        echo "Milvus is already running."
        exit 0
    fi

    res=$(sudo docker ps -a 2>/dev/null | grep milvus-standalone | wc -l)
    if [ $res -eq 1 ]; then
        echo "Starting existing container..."
        sudo docker start milvus-standalone
    else
        run_embed
    fi

    if [ $? -ne 0 ]; then
        echo "Start failed."
        exit 1
    fi

    wait_for_milvus_running
}

stop() {
    sudo docker stop milvus-standalone 2>/dev/null

    if [ $? -ne 0 ]; then
        echo "Stop failed."
        exit 1
    fi
    echo "Stop successfully."
}

delete_container() {
    res=$(sudo docker ps 2>/dev/null | grep milvus-standalone | wc -l)
    if [ $res -eq 1 ]; then
        echo "Please stop Milvus service before delete."
        exit 1
    fi
    sudo docker rm milvus-standalone 2>/dev/null
    if [ $? -ne 0 ]; then
        echo "Delete milvus container failed."
        exit 1
    fi
    echo "Delete milvus container successfully."
}

delete() {
    read -p "Please confirm if you'd like to proceed with the delete. This operation will delete the container and data. Confirm with 'y' for yes or 'n' for no. > " check
    if [ "$check" == "y" ] || [ "$check" == "Y" ]; then
        delete_container
        sudo rm -rf $(pwd)/volumes
        sudo rm -rf $(pwd)/embedEtcd.yaml
        sudo rm -rf $(pwd)/user.yaml
        echo "Delete successfully."
    else
        echo "Exit delete"
        exit 0
    fi
}

upgrade() {
    read -p "Please confirm if you'd like to proceed with the upgrade. The default will be to the latest version. Confirm with 'y' for yes or 'n' for no. > " check
    if [ "$check" == "y" ] || [ "$check" == "Y" ]; then
        res=$(sudo docker ps -a 2>/dev/null | grep milvus-standalone | wc -l)
        if [ $res -eq 1 ]; then
            stop
            delete_container
        fi

        curl -sfL https://raw.githubusercontent.com/milvus-io/milvus/master/scripts/standalone_embed.sh -o standalone_embed_latest.sh && \
        bash standalone_embed_latest.sh start && \
        echo "Upgrade successfully."
    else
        echo "Exit upgrade"
        exit 0
    fi
}

case $1 in
    restart)
        stop
        start
        ;;
    start)
        start
        ;;
    stop)
        stop
        ;;
    upgrade)
        upgrade
        ;;
    delete)
        delete
        ;;
    *)
        echo "Usage: bash $0 {start|stop|restart|upgrade|delete}"
        ;;
esac