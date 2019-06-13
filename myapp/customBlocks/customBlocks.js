Blockly.Blocks['forward_block'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("kör framåt");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(290);
    this.setTooltip("Få bilen att köra framåt");
    this.setHelpUrl("");
  }
};

Blockly.JavaScript['forward_block'] = function(block) {
    var code = 'car.drive_forward()\n';
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
      .appendField("sväng")
      .appendField(new Blockly.FieldDropdown([["vänster","left"], ["höger","right"]]), "DIRECTION");
  this.setPreviousStatement(true, null);
  this.setNextStatement(true, null);
  this.setColour(290);
this.setTooltip("Få bilen att svänga till höger eller vänster!");
this.setHelpUrl("");
}
};

Blockly.JavaScript['turn_block'] = function (block) {
    direction = block.getFieldValue('DIRECTION');
    if (direction == 'left') {
        var code = 'car.turn_left()\n'
    }
    if (direction == 'right') {
        var code = 'car.turn_right()\n'
    }
    return code;
};
