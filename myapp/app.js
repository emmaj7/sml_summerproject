// Main file for the server application.
// Written by: Mikael Glamheden & Emma Johansson
// Last edited: 2019-06-27


const fs = require("fs");
const express = require("express");
const bodyParser = require("body-parser");
const path = require('path');
const {spawn} = require('child_process'); // For calling python scripts
const shell = require('shelljs'); // For running shell commands
const Blockly = require('node-blockly'); // Required in frontend.
const prependFile = require('prepend-file'); // To append to beginning of file
const lineReader = require('line-reader');

// ros-related packages
const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const geometry_msgs = rosnodejs.require('geometry_msgs').msg

var app = express();
var http = require("http").createServer(app);
var io = require('socket.io')(http); // handles two-way data streams

var urlencodedParser = bodyParser.urlencoded({extended:false});

app.set("view engine", "ejs");

app.use("/assets", express.static("assets"));
app.use("/node_modules/node-blockly/blockly", express.static("node_modules/node-blockly/blockly"));
app.use("/customBlocks", express.static("customBlocks"));
app.use(express.static("views"));
app.use("/node_modules/pixi.js", express.static("node_modules/pixi.js")); // for the simulation window.
app.use("/node_modules/socket.io-client", express.static("node_modules/socket.io-client"));


// renders the index page.
app.get("/", function(req, res){
  res.render("firstPage");
});

// Renders level 1
app.get("/level1", function(req, res){
  res.render("level1");
});

// Renders level 2
app.get("/level2", function(req, res){
  res.render("level2");
});

// Renders level 3
app.get("/level3", function(req, res){
  res.render("level3");
});

app.get('/helpPage', function(req, res){
  res.render('helpPage');
});

app.get('/adminPage', function(req, res){
  res.render('adminPage');
});

// writes post to file code.py.
app.post("/postcode", urlencodedParser, function(req,res){
  // console.log(req.body.code);
  var filename = 'code.py';
  writeCode(req.body.code,req.body.id, filename);
});

// writes post to file code_real.py.
app.post("/postcode2", urlencodedParser, function(req,res){
  // console.log(req.body.code);
  var filename = 'code_real.py';
  writeCode(req.body.code,req.body.id, filename);
});

// Launches roscore. REQUIRES ROS INSTALLED ON COMPUTER
// Required to run simulation.
shell.exec('roscore', {async:true});


io.on('connection', function(socket){
  console.log('opened connection');
  var forced_exit = false;
  // Responds to request to 'button pressed'.
  // Does the following:
  // 1. recieves window id.
  // 2. Recieves coordinate stream from python simulation as json
  // 3. Sends stream to 'position sent'
  socket.on('simulationOnly', function(msg){
    var msgParsed = JSON.parse(msg);
    const subprocess = runScript(msgParsed.id, msgParsed.start, msgParsed.goal);
    subprocess.stdout.on('data', (data) => {
    try { // try-catch to only send data that is in JSON format
      var obj = JSON.parse(data);
      socket.emit('position-sent-sim', `${JSON.stringify(obj)}`);
      // console.log('position sent');
    } catch(e) {
      console.log(`${data}`); // Writes data which isn't json to log.
    }
    });
    // The code below is only for error catching.
    subprocess.stderr.on('data', (data) => {
      console.log(`[USER ${msgParsed.id}] Error type 2:${data}`);
    });
    subprocess.stderr.on('close', () => {
      console.log(`[USER ${msgParsed.id}] Closed`);
      socket.emit('close', forced_exit);
    });
    subprocess.on("exit", exitCode => {
      console.log(`[USER ${msgParsed.id}] Exiting with code ${exitCode}`);
      socket.disconnect(true);
    });
    socket.on('collision', function(){
      socket.emit('close');
      console.log(`[USER ${msgParsed.id}] Collision detected`);
      subprocess.kill();
    });
    socket.on('cancel-simulation', function(){
        console.log(`[USER ${msgParsed.id}] Cancelled simulation`);
        forced_exit = true;
        subprocess.kill();
    });
  });

  // Launches the car.
  socket.on('runCodeOnCar', function(msg){
    // var command = 'rosnode kill SVEA5 /SVEA_high_level_commands /listener_node/SVEA5 /qualisys /serial_node /world_to_qualisys_tf';
    // // var command = 'rosnode kill /SVEA5 /listener_node/SVEA5 /qualisys /serial_node world_to_qualisys_tf'
    // shell.exec(command, {async:false}, function(){
    //   shell.exec('rosnode list', function(code, stdout, stderr){
    //     console.log(stdout);
    //   });
    // console.log('shutting down old nodes');
      console.log('starting car');
      var obj = JSON.parse(msg);
      var id = 'USER' + obj.id;
      var goal = obj.goal;
      var command = 'roslaunch svea SVEA_high_level_commands.launch ';
      var args = 'my_args:=' + '"' + id + ' ' + JSON.stringify(goal) + '"';
      shell.exec(command + args, {async:true}, function(code, stdout, stderr){
        console.log('Exit Code: ', code);
        console.log('Program stderr:', stderr);
        console.log('Program output:', stdout);
      });
      transmitPose(socket, id);
      console.log('Launched SVEA_high_level_commands');
    // });
  });

  socket.on('inspectCode', function(){
    var outputString = '';
    var startSaving = false;
    lineReader.eachLine('code_real.py', function(line) {
        if (line.includes('# ID:')) {
            startSaving = true;
        } else if (line.includes('#####')) {
            socket.emit('inspectCodeRes', outputString);
            console.log(outputString);
        } else if (startSaving) {
            outputString = outputString + line;
        }
    });
  });
  socket.on('clearCode', function(){
    const data = '';
    const message = 'code.py has been cleared';
    fs.writeFile('code.py', data, (err) => {
      if (err) throw err;
      console.log(message);
    });
    socket.emit('clearCodeRes', message);
  });
  socket.on('clearCodeReal', function(){
    const data = '';
    const message = 'code_real.py has been cleared';
    fs.writeFile('code_real.py', data, (err) => {
      if (err) throw err;
      console.log(message);
    });
    socket.emit('clearCodeRealRes', message);
  });

});


// This function launches the python simulation
function runScript(id, start, goal){
  id = 'USER' + id;
  var pathId = path.join(__dirname, '/../svea_starter/src/svea/src/scripts/sim/sim_SVEA_high_level_commands.py');
  return spawn('python', ["-u",
                          pathId,
                          id,
                          JSON.stringify(start),
                          JSON.stringify(goal)], {detached: true});
}

// Writes code to beginning of code file
function writeCode(code, id, filename){
  data = '# ' + 'ID:' + id + '\n' + code + '#####\n';
  prependFile(filename, data, function(err){
    if (err){
      console.log(err);
    }
  });
}
// get yaw from quaternions
function yawFromQuaternions(qObj){
  var yaw   =  Math.asin(2*qObj.x*qObj.y + 2*qObj.z*qObj.w);
  return yaw;
}

// send car position to simulation window.
function transmitPose(socket, name) {
  // Register node with ROS master
  console.log('starting listener')
  rosnodejs.initNode('/listener_node/SVEA5')
    .then((rosNode) => {
      // Create ROS subscriber on the 'chatter' topic expecting String messages
      let sub = rosNode.subscribe('/SVEA5/pose', geometry_msgs.PoseStamped,
        (data) => { // define callback execution
          var dataString = JSON.stringify(data);
          var pos = data.pose.position;
          var obj = {'x':pos.x, 'y': pos.y,};
          var yaw = yawFromQuaternions(data.pose.orientation);
          obj.yaw = yaw;
          socket.emit('position-sent-car', JSON.stringify(obj));
          // console.log(dataString);
        }
      );
    });
}

// Error 404 handling.
app.use(function(req, res, next) {
  res.status(404);
  var fullUrl = req.protocol +'://' + req.get('host') + req.url;
  res.render('404page', {'message': fullUrl});
});


http.listen(3000, function(){
  console.log('listening on *:3000')
});
