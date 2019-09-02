
//
// Functions related to running the code on the car itself. + functions for the admin page.
// Written by: Mikael Glamheden
// Last updated: 2019-07-17
//

function getRequest(){
  // Ska byta sida
  $.get(`/teamName/?id=${unique_id}`, function(data, status){
    console.log(`${data} and status is ${status}`);
    window.open(`${data.url}/lastPage/?teamName=${data.teamName}`, "_self");
    console.log("Went to last page.");
  });
}

// Post the python code to the server.
function postCodeToCar(callback){
  // Get python code from blockly
  window.LoopTrap = 1000;
  Blockly.JavaScript.INFINITE_LOOP_TRAP =
  'if (--window.LoopTrap == 0) throw "Infinite loop.";\n';
  var code = Blockly.Python.workspaceToCode(workspace);

  var codeObj = {code:code, id: unique_id};
  console.log('posting code');
  //  post code to server.
  var codeObj = {code: code, id: unique_id};
  $.post("/postcode2", codeObj, function(data, status){
    console.log(`${data} and status is ${status}`)
  });
  callback();
}

// clear the content of code.py
function clearCode(){
  var socket = io();
  socket.on('connect', function(){
    socket.emit('clearCode', function(){
    });
    socket.on('clearCodeRes', function(msg){
      alert(msg);
      socket.close();
    });
  });
}

// clear content of code_real.py
function clearCodeReal(){
  var socket = io();
  socket.on('connect', function(){
    socket.emit('clearCodeReal', function(){
    });
    socket.on('clearCodeRealRes', function(msg){
      alert(msg);
      socket.close();
    });
  });
}

// shows the id's of the posted code solutions
function checkUsedId(){
  var socket = io();
  socket.on('connect', function(){
    socket.emit('checkUsedId', function(){
    });
    socket.on('checkUsedIdRes', function(msg){
      console.log(msg);
      tabs = '\t';
      string = tabs;
      for (var i = 0; i < msg.length-1; i++) {
        string = string + msg[i] + '\n' + tabs;
      }
      string = string + msg[msg.length-1]
      alert(`The used id's are:\n ${string}`);
      socket.close();
    });
  });
}
// returns the id of the shortest code solution in code_real.py
function shortestCode(){
  var socket = io();
  socket.on('connect', function(){
    socket.emit('getShortestCode', function(){
    });
    socket.on('getShortestCodeRes', function(msg){
      console.log(msg.id);
      if (msg.shortList.length > 1) {
        var tabs = '\t\t\t\t\t'
        var string = tabs;
        for (var i = 0; i < msg.shortList.length -1; i++) {
            string = string + msg.shortList[i] + '\n' + tabs;
        }
        string = string + msg.shortList[msg.shortList.length-1];
        alert(`\t\tThe shortest code is ${msg.length} lines long.\n The following teams got solved it with that many steps: \n ${string}`)
      } else {
        alert(`The shortest code is ${msg.length} lines long and is written by: \n ${msg.idName}.`);
      }
      unique_id = msg.id
      unique_name = msg.idName
      socket.close();
    });
  });
}

// clear content of UsedIds-variable
function clearUsedIdsVar(){
  var socket = io();
  socket.on('connect', function(){
    socket.emit('clearUsedIdsVar', function(){
    });
    socket.on('clearUsedIdsVarRes', function(msg){
      usedIds = {};
      console.log(usedIds);
      socket.close();
    });
  });
}
