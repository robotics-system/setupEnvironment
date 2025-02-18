#!/bin/bash

# Exit on any error
set -e

# Function to check if a command was successful
check_status() {
  if [ $? -eq 0 ]; then
    echo "✓ $1 completed successfully"
  else
    echo "✗ Error during $1"
    exit 1
  fi
}

# Function to ensure directory exists
ensure_dir() {
  if [ ! -d "$1" ]; then
    mkdir -p "$1"
  fi
}
