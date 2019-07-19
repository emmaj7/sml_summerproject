
//
// Functions related to running the code on the car itself. + functions for the admin page.
// Written by: Mikael Glamheden
// Last updated: 2019-07-17
//

// Post the python code to the server.
function postCodeToCar(){
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
}

// stops all processes running on the car.
function cancelCar(){
  let socket = io();
  socket.on('connect',function(){
    console.log('Connected');
    socket.emit('kill-process');
    console.log('killing process');
    socket.close();
  });
}
// execute code on car.
function runOnCar(){
  getShortestCode(runOnCarFunction);
}
function runOnCarFunction(){
  let socket = io();
  socket.on('connect',function(){
    console.log('Connected');
    console.log('Session id: ' + unique_id); // Unique id for each html page opened.
    socket.emit('runCodeOnCar', JSON.stringify({'id': unique_id,
                                                'goal': goal_coords}));
    console.log('Running code on car');
    alert(`Executing code written by team ${unique_name} on the car.`);
    socket.close();
  });
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
function checkAvailableId(){
  var socket = io();
  socket.on('connect', function(){
    socket.emit('checkAvailableId', function(){
    });
    socket.on('checkAvailableIdRes', function(msg){
      console.log(msg);
      alert(msg);
      socket.close();
    });
  });
}

// returns the id of the shortest code solution in code_real.py
function getShortestCode(callback){
  var socket = io();
  socket.on('connect', function(){
    socket.emit('getShortestCode', function(){
    });
    socket.on('getShortestCodeRes', function(msg){
      console.log(msg.id);
      // alert(`The shortest code is ${msg.length} lines long and is written by ${msg.idName}.`);
      unique_id = msg.id
      unique_name = msg.idName
      console.log('unique_id : ', unique_id);
      socket.close();
      console.log('msg : ', msg);
      callback();
    });
  });
}
