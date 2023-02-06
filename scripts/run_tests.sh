#!/bin/bash
input="./test_data.txt"
counter=0
TIMEFORMAT='Generate Path in %R seconds.'
while IFS= read -r line
do
  time {
    let counter++
    echo ""
    echo "=== $counter. $line ==="
    ./checker_linux "$(./solution $line)"
  }
done < "$input"
