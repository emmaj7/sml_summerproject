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

const logOutput = (name) => (data) => console.log(`[${name}] ${data.toString()}`)

function run() {
  return new Promise((resolve, reject) => {
    const process = spawn('python', ['./script.py', 'my', 'args']);

    const out = []
    process.stdout.on(
      'data',
      (data) => {
        out.push(data.toString());
        logOutput('stdout')(data);
      }
    );


    const err = []
    process.stderr.on(
      'data',
      (data) => {
        err.push(data.toString());
        logOutput('stderr')(data);
      }
    );

    process.on('exit', (code, signal) => {
      logOutput('exit')(`${code} (${signal})`)
      if (code !== 0) {
        reject(new Error(err.join('\n')))
        return
      }
      try {
        resolve(JSON.parse(out[0]));
      } catch(e) {
        reject(e);
      }
    });
  });
}

(async () => {
  try {
    const output = await run()
    logOutput('main')(output.message)
    process.exit(0)
  } catch (e) {
    console.error('Error during script execution ', e.stack);
    process.exit(1);
  }
})();



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
