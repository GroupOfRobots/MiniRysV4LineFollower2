#!/bin/bash


build_motors () (
	cd "libs"
	cd $1 
	mkdir "build"
	cd "build" 	
	cmake ".." 
	make
)

build_motors "camera"
build_motors "udp_comm"
build_motors "motors"
build_motors "pid"
build_motors "data_saver"
