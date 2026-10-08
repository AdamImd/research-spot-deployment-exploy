// Run with: node tests/test_kinematics.mjs
import assert from "node:assert/strict";
import { kinematicPose } from "../src/spot_deploy/web/kinematics.js";

const model = {
  root: "base", joint_names: ["turn", "slide"], tips: [],
  joints: [
    {name: "turn", parent: "base", child: "arm", type: "revolute",
      xyz: [1, 0, 0], rpy: [0, 0, 0], axis: [0, 0, 1]},
    {name: "fixed", parent: "arm", child: "tip", type: "fixed",
      xyz: [1, 0, 0], rpy: [0, 0, 0], axis: [1, 0, 0]},
    {name: "slide", parent: "tip", child: "grip", type: "prismatic",
      xyz: [0, 0, 0], rpy: [0, 0, Math.PI / 2], axis: [1, 0, 0]},
  ],
};
function near(actual, expected) {
  assert.equal(actual.length, expected.length);
  actual.forEach((v, i) => assert.ok(Math.abs(v - expected[i]) < 1e-12,
    `${actual} != ${expected}`));
}
let pose = kinematicPose(model, [Math.PI / 2, 0.2], [1, 0, 0, 0]);
assert.deepEqual(Object.keys(pose.nodes), ["base", "arm", "tip", "grip"]);
near(pose.nodes.tip.p, [1, 1, 0]);
near(pose.nodes.grip.p, [0.8, 1, 0]);
pose = kinematicPose(model, [Math.PI / 2, 0.2], [Math.SQRT1_2, 0, 0, Math.SQRT1_2]);
near(pose.nodes.tip.p, [-1, 1, 0]);
near(pose.nodes.grip.p, [-1, 0.8, 0]);
console.log("PASS: revolute, fixed terminal, prismatic, origin rotation, and body quaternion FK");
