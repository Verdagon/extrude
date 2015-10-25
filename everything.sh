#!/bin/bash

# example: ./everything.sh arial.ttf ariallow 5 1.0

for i in `seq 1 65535`;
do
	./extrudecpp $1 $i $2 $3 $4 $5
done
