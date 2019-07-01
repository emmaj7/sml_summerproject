// Runs code in simulation environment.
// Written by: Mikael Glamheden
// Date: 2019-06-27

function runCode(){
  // Generate JavaScript code and run it.
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
  var onCar = false;
  // Run simulation. Function in simulationWindow.ejs
  runSimulation(onCar);


  // Blockly.JavaScript.INFINITE_LOOP_TRAP = null;
  // try {
  //   eval(code);
  // } catch (error) {
  //   alert(error);
  // }
}
