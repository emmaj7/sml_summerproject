// Contains custom blocks for toolbox
// Written by: Emma Johansson
// Last updated: 2019-07-09

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

Blockly.Python['forward_block'] = function(block) {
    var code = 'rover.drive_forward()\n';
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
    return code;
};

// Block containing dropdown menu with 'left' at the top
Blockly.Blocks['turn_block_left'] = {
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


Blockly.Python['turn_block_left'] = function (block) {
    direction = block.getFieldValue('DIRECTION');
    if (direction == 'left') {
        var code = 'rover.turn_left()\n'
    }
    if (direction == 'right') {
        var code = 'rover.turn_right()\n'
    }
    return code;
};

// Block containing dropdown menu with 'right' at the top
Blockly.Blocks['turn_block_right'] = {
init: function() {
  this.appendDummyInput()
      .appendField("turn")
      .appendField(new Blockly.FieldDropdown([["right","right"], ["left","left"]]), "DIRECTION");
  this.setPreviousStatement(true, null);
  this.setNextStatement(true, null);
  this.setColour(290);
this.setTooltip("Makes the rover turn left or right");
this.setHelpUrl("");
}
};

Blockly.Python['turn_block_right'] = function (block) {
    direction = block.getFieldValue('DIRECTION');
    if (direction == 'left') {
        var code = 'rover.turn_left()\n'
    }
    if (direction == 'right') {
        var code = 'rover.turn_right()\n'
    }
    return code;
};
