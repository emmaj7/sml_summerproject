
function runCode(){
  // Generate JavaScript code and run it.
  window.LoopTrap = 1000;
  Blockly.JavaScript.INFINITE_LOOP_TRAP =
  'if (--window.LoopTrap == 0) throw "Infinite loop.";\n';
  var code = Blockly.JavaScript.workspaceToCode(workspace);

  var codeObj = {code:code, id: unique_id};
  $.post("/postcode", codeObj, function(codeObj, status){
    console.log(`${codeObj.code} and status is ${status}`)
  });
  // Run simulation
  runCar()
  // Blockly.JavaScript.INFINITE_LOOP_TRAP = null;
  // try {
  //   eval(code);
  // } catch (error) {
  //   alert(error);
  // }


}
