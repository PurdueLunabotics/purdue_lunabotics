#!/usr/bin/env bash
echo "Waiting 5 seconds before proceeding..."
sleep 5
timeout 5 ./proceed.sh & timeout 5 ./mini_proceed.sh
