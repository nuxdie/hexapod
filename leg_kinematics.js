(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.HexapodKinematics = api;
})(typeof globalThis === "object" ? globalThis : this, function () {
  "use strict";

  function radians(degrees) {
    return degrees * Math.PI / 180;
  }

  function degrees(radiansValue) {
    return radiansValue * 180 / Math.PI;
  }

  function clamp(value, minimum, maximum) {
    return Math.max(minimum, Math.min(maximum, value));
  }

  function mirroredGeometry(model, mirror) {
    return {
      base: { x: model.baseX, y: 0, z: 0 },
      femurPivot: {
        x: model.femurPivotX,
        y: model.femurPivotY * mirror,
        z: model.femurPivotZ
      },
      tibiaAxialY: model.tibiaAxialY * mirror
    };
  }

  function rotateZ(vector, angle) {
    const cosine = Math.cos(angle);
    const sine = Math.sin(angle);
    return {
      x: vector.x * cosine - vector.y * sine,
      y: vector.x * sine + vector.y * cosine,
      z: vector.z
    };
  }

  function add(left, right) {
    return { x: left.x + right.x, y: left.y + right.y, z: left.z + right.z };
  }

  function validModel(model) {
    return [
      model.baseX,
      model.femurPivotX,
      model.femurPivotY,
      model.femurPivotZ,
      model.femurLength,
      model.tibiaLength,
      model.tibiaAxialY
    ].every(Number.isFinite)
      && model.baseX >= 0
      && model.femurLength > 0
      && model.tibiaLength > 0;
  }

  function forward(angles, model, mirror) {
    if (!validModel(model)) throw new Error("invalid leg geometry");
    const offsets = mirroredGeometry(model, mirror);
    const coxa = radians(angles.coxa);
    const femur = radians(angles.femur);
    const tibia = radians(angles.tibia);
    const axle = offsets.base;
    const femurShaft = add(axle, rotateZ(offsets.femurPivot, coxa));
    const femurVector = rotateZ({
      x: model.femurLength * Math.cos(femur),
      y: 0,
      z: model.femurLength * Math.sin(femur)
    }, coxa);
    const tibiaShaft = add(femurShaft, femurVector);
    const tibiaVector = rotateZ({
      x: model.tibiaLength * Math.cos(femur + tibia),
      y: offsets.tibiaAxialY,
      z: model.tibiaLength * Math.sin(femur + tibia)
    }, coxa);
    const foot = add(tibiaShaft, tibiaVector);
    return {
      servo: { x: 0, y: 0, z: 0 },
      axle,
      femurShaft,
      tibiaShaft,
      foot
    };
  }

  function solve(point, model, mirror) {
    if (!validModel(model) || ![point.x, point.y, point.z].every(Number.isFinite)) {
      return { valid: false, reason: "Geometry and target values must be finite positive measurements." };
    }
    const offsets = mirroredGeometry(model, mirror);
    const relativeX = point.x - offsets.base.x;
    const relativeY = point.y - offsets.base.y;
    const horizontalReach = Math.hypot(relativeX, relativeY);
    const lateralOffset = offsets.femurPivot.y + offsets.tibiaAxialY;
    if (horizontalReach < Math.abs(lateralOffset) - 0.001) {
      return { valid: false, reason: "Target lies inside the rotating lateral-offset radius." };
    }

    const rotatingX = Math.sqrt(Math.max(0, horizontalReach * horizontalReach - lateralOffset * lateralOffset));
    const planarX = rotatingX - offsets.femurPivot.x;
    const planarZ = point.z - offsets.base.z - offsets.femurPivot.z;
    const reach = Math.hypot(planarX, planarZ);
    const minimumReach = Math.abs(model.femurLength - model.tibiaLength);
    const maximumReach = model.femurLength + model.tibiaLength;
    if (reach < minimumReach - 0.001 || reach > maximumReach + 0.001) {
      return {
        valid: false,
        reason: `Planar reach ${reach.toFixed(1)} mm is outside ${minimumReach.toFixed(1)}-${maximumReach.toFixed(1)} mm.`
      };
    }

    const tibia = Math.acos(clamp(
      (reach * reach - model.femurLength * model.femurLength - model.tibiaLength * model.tibiaLength)
        / (2 * model.femurLength * model.tibiaLength),
      -1,
      1
    ));
    const femur = Math.atan2(planarZ, planarX)
      - Math.atan2(model.tibiaLength * Math.sin(tibia), model.femurLength + model.tibiaLength * Math.cos(tibia));
    const targetAzimuth = Math.atan2(relativeY, relativeX);
    const coxa = targetAzimuth - Math.atan2(lateralOffset, rotatingX);
    const angles = { coxa: degrees(coxa), femur: degrees(femur), tibia: degrees(tibia) };
    return { valid: true, angles, joints: forward(angles, model, mirror) };
  }

  return { forward, solve };
});
