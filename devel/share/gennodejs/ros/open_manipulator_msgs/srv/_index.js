
"use strict";

let SetJointPosition = require('./SetJointPosition.js')
let SetDrawingTrajectory = require('./SetDrawingTrajectory.js')
let GetKinematicsPose = require('./GetKinematicsPose.js')
let GetJointPosition = require('./GetJointPosition.js')
let SetActuatorState = require('./SetActuatorState.js')
let SetKinematicsPose = require('./SetKinematicsPose.js')

module.exports = {
  SetJointPosition: SetJointPosition,
  SetDrawingTrajectory: SetDrawingTrajectory,
  GetKinematicsPose: GetKinematicsPose,
  GetJointPosition: GetJointPosition,
  SetActuatorState: SetActuatorState,
  SetKinematicsPose: SetKinematicsPose,
};
