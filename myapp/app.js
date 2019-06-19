var http = require("http");
var fs = require("fs");
var express = require("express");
var bodyParser = require("body-parser");
const path = require('path')
const {spawn} = require('child_process') // For calling python scripts
var shell = require('shelljs'); // For running shell commands

var app = express();

var urlencodedParser = bodyParser.urlencoded({extended:false});

app.set("view engine", "ejs");

app.use("/assets", express.static("assets"));
app.use("/node_modules/node-blockly/blockly", express.static("node_modules/node-blockly/blockly"));
app.use("/customBlocks", express.static("customBlocks"));
app.use(express.static("views"));
app.use("/node_modules/pixi.js", express.static("node_modules/pixi.js"));

app.get("/", function(req, res){
  res.render("firstPage");
});

app.get("/level1", function(req, res){
  res.render("level1");
});

app.get("/level2", function(req, res){
  res.render("level2");
});

app.get("/level3", function(req, res){
  res.render("level3");
});

app.post("/", urlencodedParser, function(req,res){
  console.log(req.body.code);
  fs.writeFile("code.py", req.body.code + "##### \n",function(error){
    if (error) throw error;
    console.log("Updated!")
  });
})

// Renders the simulation window page. Not neccessary later.
app.get('/testpage', function(req,res){
  res.render('test2');
});

// Launches roscore. Required to run simulation.
shell.exec('roscore', {async:true});

// This function launches the python simulation
function runScript(){
  return spawn('python', [
    "-u",
    path.join(__dirname, '/../svea_starter/src/svea/src/scripts/sim/sim_SVEA_high_level_commands.py')]);
}

// Responds to get request to /testpage/button
// When called establishes a connection that wont close.
// Does the following:
// 1. Starts the python simulation
// 2. Recieves coordinate steam from pthon simulation as json
// 3. Streams coordinates to /testpage/button event stream.
app.get('/testpage/button', function(req,res){
  res.status(200).set({
    'connection': 'keep-alive',
    'cache-control': 'no-cache',
    'content-Type': 'text/event-stream'
  });
  const subprocess = runScript()
    subprocess.stdout.on('data', (data) => {
      try { // try-catch to only send data that is in JSON format
        var data = JSON.parse(data);
        res.write(`data: ${JSON.stringify(data)} \n\n`);
      } catch(e) {
      }
    });
  // The code below is only for error catching.
  subprocess.stderr.on('data', (data) => {
    console.log(`error type 2:${data}`);
  });
  subprocess.stderr.on('close', () => {
    console.log("Closed");
  });
  subprocess.on("exit", exitCode => {
  console.log("Exiting with code " + exitCode);
  });
});


app.listen(3000);
