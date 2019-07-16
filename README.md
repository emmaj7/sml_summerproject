# sml_summerproject
This project uses [blockly](https://developers.google.com/blockly/) which lets the user code a simple program to control a driver-less RC vehicle (KTH's SVEA cars). The user simulates the vehicle's path in the browser before trying it out for real. The project is mainly written in JavaScript, HTML and Python.
## Setting up the invironment
The project assumes you are running ubuntu 16.04. and that you have the following applications installed on your machine:
* [ROS](https://www.ros.org/). You can install ROS by following [this guide](http://wiki.ros.org/kinetic/Installation/Ubuntu).
* [Node.js](https://nodejs.org/en/) and [npm](https://www.npmjs.com/). You can install them by following [this guide](https://tecadmin.net/install-latest-nodejs-npm-on-ubuntu/).

The python application requires numpy and matplotlib. Install them if you don't already have them.

	pip install numpy matplotlib

Futhermore we recommend installing [nodemon](https://www.npmjs.com/package/nodemon) which simplifies debuggging when you're testing out your applications.

	npm install -g nodemon

Now clone the git repository

	git clone https://github.com/emmaj7/sml_summerproject.git

and go to it using the terminal. Next you will have to install a few ROS dependencies

	cd svea_starter

Now run

	apt-get install libbullet-dev

then run

	sudo apt-get install libsdl-image1.2-dev
	sudo apt-get install libsdl-dev

Compile and link the ROS libraries

	catkin_make
	source devel/setup.bash
	rospack profile

To make sure the libraries are linked in the future, also call

	echo "source <path-to-svea-starter>/devel/setup.bash" >> ~/.bashrc
	source ~/.bashrc

where you need to replace "<path-to-svea-starter>" with the file path to wherever you cloned "svea_starter". For example, the full path might look like "/home/user/sml_summerproject/svea_starter/devel/setup.bash".

Finally you have to install all node.js app dependencies. Go into the folder called "myapp" and run

	npm install

You should now be good to go. Start the app on a local server by running:

	nodemon app.js

You can view the application by going into a browser and accessing

	localhost:3000
