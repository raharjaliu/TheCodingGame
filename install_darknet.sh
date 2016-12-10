#!/bin/bash
#description    :	Darknet installation script.
#					This will install darknet on home directory
#author		 	:	Pandu Raharja (pandu.raharja@tum.de)
#==============================================================================

cd $HOME

echo "Cloning darknet to $HOME"
git clone https://github.com/pjreddie/darknet.git

cd darknet
make

echo "Calling darknet. Make sure the ouput equals 'usage: ./darknet <function>'"
./darknet
./darknet imtest data/eagle.jpg

echo "Installation complete. Make sure to open Original.png, Gray.png and C4.png to make sure the installation is correct"
