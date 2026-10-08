/* Body-centred forward kinematics, including fixed and terminal links. */
const add = (a, b) => a.map((v, i) => v + b[i]),
  mulv = (r, v) =>
    [0, 1, 2].map(
      (i) => r[i * 3] * v[0] + r[i * 3 + 1] * v[1] + r[i * 3 + 2] * v[2],
    );
const mul = (a, b) =>
  Array.from({ length: 9 }, (_, i) => {
    let r = Math.floor(i / 3),
      c = i % 3;
    return a[r * 3] * b[c] + a[r * 3 + 1] * b[c + 3] + a[r * 3 + 2] * b[c + 6];
  });
function axisRotation(axis, a) {
  let norm = Math.hypot(...axis),
    [x, y, z] = axis.map((v) => v / norm),
    c = Math.cos(a),
    s = Math.sin(a),
    d = 1 - c;
  return [
    c + x * x * d,
    x * y * d - z * s,
    x * z * d + y * s,
    y * x * d + z * s,
    c + y * y * d,
    y * z * d - x * s,
    z * x * d - y * s,
    z * y * d + x * s,
    c + z * z * d,
  ];
}
function quaternion(q) {
  let [w, x, y, z] = q;
  return [
    1 - 2 * (y * y + z * z),
    2 * (x * y - z * w),
    2 * (x * z + y * w),
    2 * (x * y + z * w),
    1 - 2 * (x * x + z * z),
    2 * (y * z - x * w),
    2 * (x * z - y * w),
    2 * (y * z + x * w),
    1 - 2 * (x * x + y * y),
  ];
}
function rpy(v) {
  return mul(
    mul(axisRotation([0, 0, 1], v[2]), axisRotation([0, 1, 0], v[1])),
    axisRotation([1, 0, 0], v[0]),
  );
}
export function kinematicPose(model, q, quat) {
  let nodes = Object.assign(Object.create(null), { [model.root]: { p: [0, 0, 0], r: quaternion(quat) } }),
    edges = [],
    points = [];
  for (const j of model.joints) {
    let parent = nodes[j.parent];
    if (!parent) continue;
    let p = add(parent.p, mulv(parent.r, j.xyz)),
      r = mul(parent.r, rpy(j.rpy)),
      idx = model.joint_names.indexOf(j.name),
      a = idx < 0 ? 0 : q[idx];
    if (j.type === "prismatic")
      p = add(
        p,
        mulv(
          r,
          j.axis.map((v) => v * a),
        ),
      );
    else if (j.type !== "fixed") r = mul(r, axisRotation(j.axis, a));
    nodes[j.child] = { p, r };
    edges.push([parent.p, p]);
    points.push({ p, idx });
  }
  for (const tip of model.tips) {
    let node = nodes[tip.parent];
    if (node) edges.push([node.p, add(node.p, mulv(node.r, tip.xyz))]);
  }
  return { nodes, edges, points };
}
