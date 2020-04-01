#!/bin/bash

#This program will use your public key to ssh into the robodelivery server and pull
#everything (including this script) to your local machine.  
#
#Where to place this script?
#Please place this script in the directory where you have the project you want to 
#upload to github.
#example:
# ls
# baxter-sim-demo/
#

############################     NOTE    ###############################
#		This is only used for file transfer
#		You would still need to
#		git commands to push to the server
#
#		Execution:
#		./version-control <location of private key>
#
#######################################################################




####### Parameters #########
privatekey=$1
location_in_server=//home/robodeliver/GithubFile

# shh into the robodeliver server
rm -rf ./GithubFile
scp -r -i $privatekey robodeliver@131.170.250.237:$location_in_server .

