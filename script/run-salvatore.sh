#!/bin/bash

./salvatore -c config.toml -f bathymetry.json  --harness -lV &2>1 > /dev/null &
sleep 20
RUN_NAME=$(ls runs)
echo $RUN_NAME
# ./script/extract_surfaces runs/$RUN_NAME . &