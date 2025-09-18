#!/usr/bin/env bash
set -euo pipefail

echo "Configuring LeKiwi Robot (ID: ${ROBOT_ID})"
echo "Environment: ${DEPLOY_ENV}"
echo "Log Level: ${LOG_LEVEL}"

# Validate required environment
if [[ -z "${ROBOT_ID}" ]]; then
    echo "ERROR: ROBOT_ID environment variable is required"
    exit 1
fi

# Start LeKiwi host agent
echo "Starting HostAgent..."
exec python3 -m lerobot.robots.lekiwi.lekiwi_host \
    --robot-id "${ROBOT_ID}" \
    --config "${CONFIG_PATH}" \
    --log-level "${LOG_LEVEL}"