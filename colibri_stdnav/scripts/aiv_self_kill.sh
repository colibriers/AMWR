#!/bin/bash

rosnode list > ~/.aiv_node_list.txt

while read line
do
   echo "--- got a node : $line"
   rosnode kill $line
   sleep 0.1
   echo "--- killed the above node"
done < ~/.aiv_node_list.txt
