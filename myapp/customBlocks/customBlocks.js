Blockly.Blocks['forward_block'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("drive forward");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(290);
    this.setTooltip("Makes the rover drive forward");
    this.setHelpUrl("");
  }
};

// Blockly.JavaScript['forward_block'] = function(block) {
//     var code = 'car.drive_forward()\n';
//     // var FileSaver = require('.//file-saver');
//     // var blob = new Blob([code], {type: "text/plain;charset=utf-8"});
//     // FileSaver.saveAs(blob, "forthecarlol.txt");
//     // var blob = new Blob(["körFramåt()"], {type: "text/plain;charset=utf-8"});
//     // FileSaver.saveAs(blob, "hello world.txt");
//     return code;
// };

Blockly.Python['forward_block'] = function(block) {
    var code = 'rover.drive_forward()\n';
    // var FileSaver = require('.//file-saver');
    // var blob = new Blob([code], {type: "text/plain;charset=utf-8"});
    // FileSaver.saveAs(blob, "forthecarlol.txt");
    // var blob = new Blob(["körFramåt()"], {type: "text/plain;charset=utf-8"});
    // FileSaver.saveAs(blob, "hello world.txt");
    return code;
};

Blockly.Blocks['backward_block'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("drive backwards");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(290);
    this.setTooltip("Makes the rover drive backwards");
    this.setHelpUrl("");
  }
};

Blockly.Python['backward_block'] = function(block) {
    var code = 'rover.drive_backwards()\n';
    // var FileSaver = require('.//file-saver');
    // var blob = new Blob([code], {type: "text/plain;charset=utf-8"});
    // FileSaver.saveAs(blob, "forthecarlol.txt");
    // var blob = new Blob(["körFramåt()"], {type: "text/plain;charset=utf-8"});
    // FileSaver.saveAs(blob, "hello world.txt");
    return code;
};

Blockly.Blocks['turn_block'] = {
init: function() {
  this.appendDummyInput()
      .appendField("turn")
      .appendField(new Blockly.FieldDropdown([["left","left"], ["right","right"]]), "DIRECTION");
  this.setPreviousStatement(true, null);
  this.setNextStatement(true, null);
  this.setColour(290);
this.setTooltip("Makes the rover turn left or right");
this.setHelpUrl("");
}
};

// Blockly.JavaScript['turn_block'] = function (block) {
//     direction = block.getFieldValue('DIRECTION');
//     if (direction == 'left') {
//         var code = 'car.turn_left()\n'
//     }
//     if (direction == 'right') {
//         var code = 'car.turn_right()\n'
//     }
//     return code;
// };

Blockly.Python['turn_block'] = function (block) {
    direction = block.getFieldValue('DIRECTION');
    if (direction == 'left') {
        var code = 'rover.turn_left()\n'
    }
    if (direction == 'right') {
        var code = 'rover.turn_right()\n'
    }
    return code;
};
