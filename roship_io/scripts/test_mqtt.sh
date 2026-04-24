#!/bin/bash

topics=("topic1" "topic2" "topic3") # Add your topics here
counter=0

while [ $counter -lt 60 ]; do
    for topic in "${topics[@]}"; do
        mosquitto_pub -h localhost -t "$topic" -m "Hello, MQTT from $topic!"
    done
    sleep 1
    ((counter++))
done
