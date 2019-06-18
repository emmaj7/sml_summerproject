var http = require("http");
var fs = require("fs");
var express = require("express");
var bodyParser = require("body-parser");
var Blockly = require("node-blockly");
const path = require('path')
const {spawn} = require('child_process') // For calling python scripts

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

/*
###################
PYTHON TEST PART
###################
*/

function runScript(){
  return spawn('python', [
    "-u",
    path.join(__dirname, '/../svea_starter/src/svea/src/scripts/sim/sim_SVEA_high_level_commands.py')]);
}
app.get('/test2', function(req,res){
  var data = { name: 'Tobi', x_var: 20, y_var: 20};
  res.render('test2',data);
});

app.get('/testpage', function(req,res){
  // res.render('test2');
  const subprocess = runScript()
  // print output of script
    subprocess.stdout.on('data', (data) => {
      try {
        var data = JSON.parse(data);
        // res.render('test2',data);
      } catch(e) {
          console.log(`error:${e}`);
      }

    });
  subprocess.stderr.on('data', (data) => {
    console.log(`error:${data}`);
  });
  subprocess.stderr.on('close', () => {
    console.log("Closed");
  });
  subprocess.on("exit", exitCode => {
  console.log("Exiting with code " + exitCode);
  });
});




app.listen(3000);

// var xmlText = `<xml xmlns="http://www.w3.org/1999/xhtml">
//                   <block type="variables_set">
//                     <field name="VAR">blockly</field>
//                     <value name="VALUE">
//                         <block type="text">
//                           <field name="TEXT">Hello Node.js!</field>
//                         </block>
//                     </value>
//                   </block>
//               </xml>`;
//
// try {
//     var xmlText = Blockly.Xml.textToDom(xmlText);
// }
// catch (e) {
//     console.log(e);
//     return
// }
//
// var workspace = new Blockly.Workspace();
// Blockly.Xml.domToWorkspace(xmlText, workspace);
// var code = Blockly.JavaScript.workspaceToCode(workspace);
//
// console.log(code)

// var server = http.createServer(function(req, res){
//   console.log("request was made: " + req.url);
//   res.writeHead(200, {"Content-Type": "text/html"});
//   var myReadStream = fs.createReadStream(__dirname + "/readMe.txt", "utf8");
//   myReadStream.pipe(res);
// });
//
// server.listen(3000, "127.0.0.1");
// console.log("Nu lyssnar vi på port 3000");
