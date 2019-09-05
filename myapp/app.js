// Main file for the server application.
// Written by: Mikael Glamheden & Emma Johansson
// Last edited: 2019-09-03

// var PORT = process.env.PORT || 3000;
const fs = require("fs");
const express = require("express");
const bodyParser = require("body-parser");
const path = require('path');
const {spawn} = require('child_process'); // For calling python scripts
const shell = require('shelljs'); // For running shell commands
const Blockly = require('node-blockly'); // Required in frontend.
const prependFile = require('prepend-file'); // To append to beginning of file
const lineReader = require('line-reader');
const url = require("url");

// ros-related packages
const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const geometry_msgs = rosnodejs.require('geometry_msgs').msg

var app = express();
var http = require("http").createServer(app);
var io = require('socket.io')(http); // handles two-way data streams

var urlencodedParser = bodyParser.urlencoded({extended:false});

app.set("view engine", "ejs");

// allows access to the folders listed
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

var computerNumber = 0; // generates a number for the user on the last page.

// Renders level 3
app.get("/level3", function(req, res){
  computerNumber = computerNumber + 1;
  res.render("level3", {id: computerNumber});
});

// Renders help-page
app.get('/helpPage', function(req, res){
  res.render('helpPage');
});

// Renders admin page. Not accessable through UI.
app.get('/adminPage', function(req, res){
  res.render('adminPage');
});

// Sends teamname and url
app.get('/teamName', function(req, res){
  var current_url = req.url;
  var baseUrl = req.protocol + "://" + req.get('host');
  var fullUrl = baseUrl + current_url;
  console.log("Full URL: " + fullUrl);
  current_url_obj = new URL(fullUrl);
  const search_params = current_url_obj.searchParams;
  const id = search_params.get('id');

  // Opens file with list of available team names
  var rawData = fs.readFileSync('teamNames.json');
  var teamNames = JSON.parse(rawData);
  var array = teamNames.teamList;
  if (id <= 10){             // If the id is under/or 10 assign name with the same index
    res.send({teamName: array[id-1],
              url: baseUrl});
  }
  else{                     // If the id is over 10 assign name with same index as last digit, also add number at end
    var lastDigit =  id % 10;
    var firstDigit = ((id % 10) % 10);
    teamName = array[lastDigit-1] + " " + (firstDigit+1).toString();
    res.send({teamName: teamName,
              url: baseUrl});
  }
});

// Renders lastpage (shown when all levels are completed)
app.get('/lastPage', function(req, res){
  var current_url = req.url;
  var baseUrl = req.protocol + "://" + req.get('host');
  var fullUrl = baseUrl + current_url;
  current_url_obj = new URL(fullUrl);
  const search_params = current_url_obj.searchParams;
  const id = search_params.get('teamName');
  res.render('lastPage', {teamName: id});
});

// writes post to file code.py.
app.post("/postcode", urlencodedParser, function(req, res){
  var filename = 'code.py';
  writeCode(req.body.code,req.body.id, filename);
});

// writes post to file code_real.py.
app.post("/postcode2", urlencodedParser, function(req, res){
  var filename = 'code_real.py';
  writeCode(req.body.code,req.body.id, filename);
});

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
  // Button on admin page. Clears code in code.py
  socket.on('clearCode', function(){
    const data = '';
    const message = 'code.py has been cleared.';
    fs.writeFile('code.py', data, (err) => {
      if (err) throw err;
      console.log(message);
    });
    socket.emit('clearCodeRes', message);
  });
  // Button on admin page. Clears code in code_real.py
  socket.on('clearCodeReal', function(){
    const data = '';
    const message = 'code_real.py has been cleared.';
    fs.writeFile('code_real.py', data, (err) => {
      if (err) throw err;
      console.log(message);
    });
    socket.emit('clearCodeRealRes', message);
  });
  // Returns the id which have been posted to the server.
  socket.on('checkUsedId', function(){
    const filename = 'code_real.py';
    getIdList(filename, function(idList){
      if (idList.length > 0){
        nameList = idNumberToName(idList);
        console.log("Returning used id's", nameList);
        socket.emit('checkUsedIdRes', nameList);
      } else {
        console.log("No used id's");
        socket.emit('checkUsedIdRes', 'None');
      }
    });
  });
  // returns the name and length of the solution which is shortest.
  socket.on('getShortestCode', function(){
    var filename = 'code_real.py';
    var shortest = getShortestCode(filename);
    if (shortest != null) {
      console.log('Returning the team names with shortest code.')
      nameList = idNumberToName(shortest.list);
      socket.emit('getShortestCodeRes', {id: shortest.id,
                                         idName: nameList[0],
                                         length: shortest.length,
                                         shortList: nameList});
    } else {
      socket.emit('checkAvailableIdRes', "Couldn't find code.");
    }
  });
});

// Returns the id of the solution which is shortest.
function getShortestCode(filename){
  var textLines = fs.readFileSync(filename, 'utf-8').split('\n');
  var shortestL = 1000;
  var currentId, currentL, shortestId;
  var shortestList = [];
  for (var i = 0; i < textLines.length; i++) {
    if (textLines[i].includes('# ID:')) {
      currentId = parseInt(textLines[i].replace(/\D/g,''));
      currentL = 0;
    } else if (textLines[i].includes('####')) {
      if (currentL < shortestL) {
        shortestL = currentL;
        shortestId = currentId;
        shortestList = [];
        shortestList.push(shortestId);
      } else if (currentL == shortestL) {
        shortestList.push(currentId);
      }
    } else {
      currentL = currentL + 1;
    }
  }
  return {id: shortestId, length: shortestL, list: shortestList};
}

// converts an id number into a cool name, input: numberList
function idNumberToName(numberList){
  const rawData = fs.readFileSync('teamNames.json');
  const nameList = JSON.parse(rawData).teamList;
  var usedNames = [];
  for (var i = 0; i < numberList.length; i++) {
    var id = numberList[i]
    if (id <= 10){
      usedNames.push(nameList[numberList[i] - 1]);
    }
    else {
      var lastDigit =  id % 10;
      var firstDigit = ((id % 10) % 10);
      teamName = nameList[lastDigit-1] + " " + (firstDigit+1).toString();
      usedNames.push(teamName);
    }
  }
  return usedNames;
}

// returns the list of used id's from code_real
function getIdList(filename,callback){
  var lines = fs.readFileSync(filename, 'utf-8').split('\n')
  lines = lines.filter(function(line){
    if (line.includes('# ID:')) {
      return line;
    }
  });
  numbers = [];
  for (var i = 0; i < lines.length; i++) {
    numbers.push(parseInt(lines[i].replace(/\D/g,'')));
  }
  callback([... new Set(numbers)].sort());

}

// This function launches the python simulation
function runScript(id, start, goal){
  var pathId = path.join(__dirname, '/../svea_starter/src/svea/src/scripts/sim/sim_SVEA_high_level_commands.py');
  return spawn('python', ["-u",
                          pathId,
                          id,
                          JSON.stringify(start),
                          JSON.stringify(goal)], {detached: true});
}

// Writes code to beginning of code file
function writeCode(code, id, filename){
  data = '# ' + 'ID:' + '&' + id + '&' + '\n' + code + '#####\n';
  prependFile(filename, data, function(err){
    if (err){
      console.log(err);
    }
  });
}

// Launches roscore. REQUIRES ROS INSTALLED ON COMPUTER
// Required to run simulation.
function startRos(){
  shell.exec('roscore', {async:true});
}
// starts roscore
startRos()

// Error 404 handling.
app.use(function(req, res, next) {
  res.status(404);
  var fullUrl = req.protocol +'://' + req.get('host') + req.url;
  res.render('404page', {'message': fullUrl});
});



http.listen(process.env.PORT || 3000, function(){
  console.log('Server running')
});
