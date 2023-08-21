#!/bin/bash

folder_path=$(find "$HOME" -type d -name "sparke" 2>/dev/null)
current_working_dir=$(pwd)

if [ -z "$folder_path" ]; then
  echo "Folder 'sparke' not found."
else
  cd $folder_path/dockerfiles/
  docker build -t infiniteecho/sparke_base -f Dockerfile.sparke_base .
fi

cd $current_working_dir