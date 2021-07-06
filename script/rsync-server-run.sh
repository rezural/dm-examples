#!/bin/bash
SERVER_ADDRESS=$(terraform show -json|jq -cr .values.outputs.instance_dns.value)
RUN_NAME=$(ssh -i ../keys/salvatore ubuntu@$SERVER_ADDRESS 'ls output/runs')

mkdir -p runs
while true
do 
    rsync -Pa --compress-level=9 -e "ssh -i ../keys/salvatore" ubuntu@${SERVER_ADDRESS}:~/output/runs/${RUN_NAME} runs/
    echo "Sleeping for 30"
    sleep 30
done
