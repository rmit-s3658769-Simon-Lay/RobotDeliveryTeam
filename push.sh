#!/bin/sh

#This program will use your public key to ssh into the robodelivery server and push
#the Github File folder from your local machine to the remote server.  
#
#Where to place this script?
#Please place this script in the directory where you have the project you want to 
#upload to the server.
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
location_in_server=/home/robodeliver/baxter-mobility-base-simdemo/rosie/RobotDeliveryTeam

# shh into the robodeliver server
scp -r -i $privatekey ./GithubFile robodeliver@131.170.250.237:$location_in_server
