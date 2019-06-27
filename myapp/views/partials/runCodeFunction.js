

// Runs code in simulation environment.
function runCode(){
  // Generate JavaScript code and run it.
  window.LoopTrap = 1000;
  Blockly.JavaScript.INFINITE_LOOP_TRAP =
  'if (--window.LoopTrap == 0) throw "Infinite loop.";\n';
  var code = Blockly.JavaScript.workspaceToCode(workspace);

  document.getElementById('code').innerHTML = code; // sends code to box on screen.
  console.log(code);

  // Post code to server.
  var codeObj = {code:code, id: unique_id};
  $.post("/postcode", codeObj, function(codeObj, status){
    console.log(`${codeObj.code} and status is ${status}`)
  });

  // Run simulation. Function in simulationWindow.ejs
  runSimulation();

  // Blockly.JavaScript.INFINITE_LOOP_TRAP = null;
  // try {
  //   eval(code);
  // } catch (error) {
  //   alert(error);
  // }
}
