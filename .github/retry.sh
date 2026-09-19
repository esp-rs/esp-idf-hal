#!/usr/bin/env bash
# Runs the given command, retrying it up to three times with a pause in between.
# For the network-bound steps of CI (toolchain and tool downloads), which now and then
# fail on a reset connection.
attempts=3
for attempt in $(seq 1 $attempts); do
    "$@" && exit 0
    if [ "$attempt" -lt "$attempts" ]; then
        echo "::warning::Attempt $attempt of $attempts failed: $*; retrying in 20s"
        sleep 20
    fi
done
echo "::error::All $attempts attempts failed: $*"
exit 1
