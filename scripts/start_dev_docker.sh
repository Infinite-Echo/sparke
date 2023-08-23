#!/bin/bash

folder_path=$(find "$HOME" -type d -name "sparke" 2>/dev/null)

if [ -z "$folder_path" ]; then
  echo "Folder 'sparke' not found."
else
  cd $folder_path
  docker container rm sparke_devcontainer
  export UID=$(id -u) export GID=$(id -g); 
  docker-compose -f docker-compose.yml build sparke_dev
  docker compose -f docker-compose.yml run --name sparke_devcontainer sparke_dev
fi