#!/bin/bash
# demo/random_mock_run_demo.sh
# Automatically launch the random mock esp32 server and visualization program
# Works on Linux / macOS

# Get the project root directory
ROOT_DIR="$(dirname "$0")/.."

# Color settings
GREEN='\033[1;32m'
RED='\033[1;31m'
NC='\033[0m'  # No color

# Move to the project root directory
cd "$ROOT_DIR" || exit 1

# Start the mock esp32 server
echo -e "${GREEN}[Demo] Starting mock ESP32 server...${NC}"
python3 mock_esp32/server.py --mode random --interval 0.5 &
SERVER_PID=$!  # Record the server’s process ID

sleep 1  # Wait for the server to start

# Start the visualization program
echo -e "${GREEN}[Demo] Starting visualization program...${NC}"
python3 visualization/mock_visualization.py

# When visualization ends, stop the mock server
echo -e "${RED}[Demo] Shutting down mock server...${NC}"
kill $SERVER_PID 2>/dev/null
wait $SERVER_PID 2>/dev/null
