# action-client
This application serves as a client interface to the [ROS Simple Action Server](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29).

This README is a work in progress.

It is assumed that the user already has the action server (linked above) running, which means we skip over some steps: installing ROS, installing the action server, running the action server...here are some useful links if these steps have not been accomplished.

[ROS Installation Options](http://wiki.ros.org/ROS/Installation)


## Installing dependencies

### Node
Check if you have node installed with `node --version`. This was developed on v10.9.0 and has not been tested on other versions. If node is not installed, it is recommended to install with nvm (node version manager) using the instructions on [this page.](https://github.com/creationix/nvm)

### Action client
In the action-client directory, run the following to install all dependencies.
```
npm install
```

## Running the application

## ROS/Action server
```
roscore
```
```
rosrun actionlib_tutorials fibonacci_server.py
```

## Bridge
rosbridge is used to interface to the action server. The bridge needs to be running for a connection to the server to be established. Run the bridge with the following command while roscore and the action server are running:
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

## Client
```
npm run serve
```
After running the above command, access the application in your browser at localhost:8080.

All done! Read on for some info about developing on the app.

## Development

### Compiles and hot-reloads for development
```
npm run serve
```

### Compiles and minifies for production
```
npm run build
```

### Lints and fixes files
```
npm run lint
```
