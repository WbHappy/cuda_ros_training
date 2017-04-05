#/bin/bash

while read -r line
do
    kill -INT "$line"
done <pid.txt

rm pid.txt
