#!/bin/bash

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key=$1
case $key in
	-h|--help)
		echo "-i|--install - for gettting the project working on a new machine"
		shift	
	;;
	-i|--install) 
		shift
	;;
esac
done
