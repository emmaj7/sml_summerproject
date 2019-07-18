// Runs code in simulation environment.
// Written by: Mikael Glamheden
// Last updated: 2019-07-18

function runCode(){
  // Generate python code and run it.
  window.LoopTrap = 1000;
  Blockly.JavaScript.INFINITE_LOOP_TRAP =
  'if (--window.LoopTrap == 0) throw "Infinite loop.";\n';
  // var code = Blockly.JavaScript.workspaceToCode(workspace);
  var code = Blockly.Python.workspaceToCode(workspace);
  console.log(code);

  document.getElementById('code').innerHTML = code; // sends code to box on screen.

  // Post code to server.
  var codeObj = {code:code, id: unique_id};
  $.post("/postcode", codeObj, function(codeObj, status){
    console.log(`${codeObj.code} and status is ${status}`)
  });
  // Run simulation. Function in simulationWindow.ejs
  return runSimulation();

}
