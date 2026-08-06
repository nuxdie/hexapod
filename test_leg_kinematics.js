"use strict";

const assert = require("node:assert/strict");
const kinematics = require("./leg_kinematics.js");

const model = {
  baseX: 40,
  femurPivotX: 8,
  femurPivotY: -20,
  femurPivotZ: 8,
  femurLength: 46,
  tibiaLength: 75,
  tibiaAxialY: 15
};

function close(actual, expected, message) {
  assert.ok(Math.abs(actual - expected) < 1e-8, `${message}: expected ${expected}, got ${actual}`);
}

function closePoint(actual, expected, message) {
  close(actual.x, expected.x, `${message} x`);
  close(actual.y, expected.y, `${message} y`);
  close(actual.z, expected.z, `${message} z`);
}

const leftNeutral = kinematics.forward({ coxa: 0, femur: 0, tibia: 90 }, model, 1);
closePoint(leftNeutral.axle, { x: 40, y: 0, z: 0 }, "left axle");
closePoint(leftNeutral.femurShaft, { x: 48, y: -20, z: 8 }, "left femur shaft");
closePoint(leftNeutral.tibiaShaft, { x: 94, y: -20, z: 8 }, "left tibia shaft");
closePoint(leftNeutral.foot, { x: 94, y: -5, z: 83 }, "left foot");

const rightNeutral = kinematics.forward({ coxa: 0, femur: 0, tibia: 90 }, model, -1);
closePoint(rightNeutral.femurShaft, { x: 48, y: 20, z: 8 }, "right femur shaft");
closePoint(rightNeutral.foot, { x: 94, y: 5, z: 83 }, "right foot");

for (const mirror of [1, -1]) {
  for (const angles of [
    { coxa: 0, femur: 0, tibia: 90 },
    { coxa: 18, femur: 12, tibia: 72 },
    { coxa: -24, femur: -8, tibia: 105 }
  ]) {
    const foot = kinematics.forward(angles, model, mirror).foot;
    const solved = kinematics.solve(foot, model, mirror);
    assert.equal(solved.valid, true, solved.reason);
    close(solved.angles.coxa, angles.coxa, "coxa round trip");
    close(solved.angles.femur, angles.femur, "femur round trip");
    close(solved.angles.tibia, angles.tibia, "tibia round trip");
  }
}

assert.equal(kinematics.solve({ x: 300, y: 0, z: 0 }, model, 1).valid, false);
console.log("Offset-axis leg kinematics tests passed.");
