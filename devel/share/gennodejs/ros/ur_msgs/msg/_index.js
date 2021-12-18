
"use strict";

let Digital = require('./Digital.js');
let MasterboardDataMsg = require('./MasterboardDataMsg.js');
let ToolDataMsg = require('./ToolDataMsg.js');
let RobotStateRTMsg = require('./RobotStateRTMsg.js');
let IOStates = require('./IOStates.js');
let RobotModeDataMsg = require('./RobotModeDataMsg.js');
let Analog = require('./Analog.js');

module.exports = {
  Digital: Digital,
  MasterboardDataMsg: MasterboardDataMsg,
  ToolDataMsg: ToolDataMsg,
  RobotStateRTMsg: RobotStateRTMsg,
  IOStates: IOStates,
  RobotModeDataMsg: RobotModeDataMsg,
  Analog: Analog,
};
