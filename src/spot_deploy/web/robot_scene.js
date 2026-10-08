import * as THREE from "./vendor/three/three.module.min.js";
import { kinematicPose } from "./kinematics.js";

const layerColors = { measured: 0x6bddc3, target: 0xf5c454, command: 0xb29bf4 };
const kinds = ["measured", "target", "command"];

export class RobotScene {
  constructor(canvas, model, redraw) {
    this.canvas = canvas;
    this.model = model;
    this.redraw = redraw;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.scene = new THREE.Scene();
    this.camera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.01, 20);
    this.camera.up.set(0, 0, 1);
    this.scene.add(new THREE.HemisphereLight(0xe5f0ff, 0x364050, 2.0));
    for (const [p, intensity] of [[[1, -2, 3], 2.5], [[-2, 1, 1], 1.5]]) {
      const light = new THREE.DirectionalLight(0xffffff, intensity);
      light.position.set(...p);
      this.scene.add(light);
    }
    const grid = new THREE.GridHelper(2.4, 12, 0x465b6a, 0x293b49);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -0.58;
    this.scene.add(grid);
    const axes = new THREE.AxesHelper(0.18);
    axes.position.set(-0.7, -0.55, -0.57);
    this.scene.add(axes);
    this.hiddenLinks = new Set();
    this.geometries = new Map();
    this.textures = new Map();
    this.layers = {};
    for (const kind of kinds) {
      const root = new THREE.Group(), links = new Map(), frames = new Map();
      for (const link of model.links) {
        const group = new THREE.Group();
        group.matrixAutoUpdate = false;
        root.add(group);
        links.set(link.name, group);
        if (kind === "measured") {
          const frame = new THREE.AxesHelper(link.visuals.length ? 0.035 : 0.05);
          frame.matrixAutoUpdate = false;
          root.add(frame);
          frames.set(link.name, frame);
        }
      }
      const lines = new THREE.LineSegments(new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({ color: layerColors[kind], transparent: true,
          opacity: kind === "measured" ? 0.7 : 0.4 }));
      root.add(lines);
      this.scene.add(root);
      this.layers[kind] = { root, links, frames, lines };
    }
    this.selection = new THREE.BoxHelper(undefined, 0x9cffe7);
    this.selection.material.transparent = true;
    this.selection.material.opacity = 0.65;
    this.selection.visible = false;
    this.scene.add(this.selection);
    if (!model.sha256) {
      const body = new THREE.Mesh(new THREE.BoxGeometry(0.86, 0.27, 0.18),
        new THREE.MeshStandardMaterial({ color: 0xe4bb51, roughness: 0.7 }));
      this.layers.measured.links.get(model.root).add(body);
    }
    canvas.addEventListener("webglcontextlost", event => {
      event.preventDefault();
      document.getElementById("mesh-status").textContent = "3D graphics context lost; refresh the viewer.";
    });
  }

  setLinkVisible(name, visible) {
    if (visible) this.hiddenLinks.delete(name);
    else this.hiddenLinks.add(name);
  }

  async geometry(spec) {
    const key = spec.url || JSON.stringify(spec);
    if (!this.geometries.has(key)) {
      this.geometries.set(key, (async () => {
        if (spec.kind === "box") return new THREE.BoxGeometry(...spec.size);
        if (spec.kind === "sphere") return new THREE.SphereGeometry(spec.radius, 24, 16);
        if (spec.kind === "cylinder") {
          const geometry = new THREE.CylinderGeometry(spec.radius, spec.radius, spec.length, 24);
          geometry.rotateX(Math.PI / 2); // URDF cylinders use Z, Three uses Y.
          return geometry;
        }
        const response = await fetch(spec.url);
        if (!response.ok) throw Error("mesh unavailable");
        const data = await response.arrayBuffer();
        if (data.byteLength !== spec.vertex_count * 32) throw Error("invalid mesh size");
        const buffer = new THREE.InterleavedBuffer(new Float32Array(data), 8);
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute("position", new THREE.InterleavedBufferAttribute(buffer, 3, 0));
        geometry.setAttribute("normal", new THREE.InterleavedBufferAttribute(buffer, 3, 3));
        geometry.setAttribute("uv", new THREE.InterleavedBufferAttribute(buffer, 2, 6));
        (spec.groups || []).forEach((group, i) => geometry.addGroup(group.start, group.count, i));
        geometry.computeBoundingBox();
        geometry.computeBoundingSphere();
        return geometry;
      })());
    }
    return this.geometries.get(key);
  }

  async material(spec, rgba) {
    const color = rgba || spec?.rgba || [0.65, 0.68, 0.72, 1];
    const material = new THREE.MeshStandardMaterial({
      color: new THREE.Color().setRGB(...color.slice(0, 3), THREE.SRGBColorSpace),
      opacity: color[3], transparent: color[3] < 1, roughness: 0.55, metalness: 0.15,
      side: THREE.DoubleSide,
    });
    if (spec?.texture && !rgba) {
      if (!this.textures.has(spec.texture))
        this.textures.set(spec.texture, new THREE.TextureLoader().loadAsync(spec.texture));
      const texture = (await this.textures.get(spec.texture)).clone();
      texture.colorSpace = THREE.SRGBColorSpace;
      texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
      texture.repeat.set(...(spec.texture_scale || [1, 1]));
      texture.needsUpdate = true;
      material.map = texture;
    }
    return material;
  }

  async load() {
    const work = [];
    for (const link of this.model.links) {
      for (const visual of link.visuals) {
        const spec = visual.geometry;
        if (spec.error) continue;
        work.push((async () => {
          const geometry = await this.geometry(spec);
          const groups = spec.groups?.length ? spec.groups : [{ material: null }];
          const materials = await Promise.all(groups.map(group =>
            this.material(spec.materials?.[group.material], visual.rgba)));
          for (const kind of kinds) {
            const mesh = new THREE.Mesh(geometry, kind === "measured"
              ? (spec.groups?.length ? materials : materials[0])
              : new THREE.MeshBasicMaterial({ color: layerColors[kind], transparent: true,
                  opacity: 0.24, depthWrite: false, side: THREE.FrontSide,
                  polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1 }));
            mesh.name = link.name;
            mesh.position.set(...visual.xyz);
            mesh.rotation.set(...visual.rpy, "ZYX");
            mesh.scale.set(...(spec.scale || [1, 1, 1]));
            mesh.renderOrder = kind === "measured" ? 0 : 1;
            this.layers[kind].links.get(link.name).add(mesh);
          }
        })());
      }
    }
    await Promise.all(work);
    this.redraw();
  }

  draw(quat, layers, camera, selected, meshes, showFrames) {
    const width = this.canvas.clientWidth, height = this.canvas.clientHeight;
    if (!width || !height) return;
    this.renderer.setSize(width, height, false);
    const scale = Math.min(width / 2.1, height / 1.5) * camera.zoom;
    Object.assign(this.camera, { left: -width / scale / 2, right: width / scale / 2,
      top: height / scale / 2, bottom: -height / scale / 2 });
    const centre = new THREE.Vector3(0, 0, -0.10);
    this.camera.position.set(3 * Math.cos(camera.yaw) * Math.cos(camera.pitch),
      3 * Math.sin(camera.yaw) * Math.cos(camera.pitch), 3 * Math.sin(camera.pitch));
    this.camera.position.add(centre);
    this.camera.lookAt(centre);
    this.camera.updateProjectionMatrix();
    for (const kind of kinds) {
      const layer = this.layers[kind], state = layers[kind];
      layer.root.visible = Boolean(state);
      if (!state) continue;
      const pose = kinematicPose(this.model, state.positions, quat);
      for (const link of this.model.links) {
        const group = layer.links.get(link.name), node = pose.nodes[link.name];
        if (!node) { group.visible = false; continue; }
        const [r, p] = [node.r, node.p];
        group.matrix.set(r[0], r[1], r[2], p[0], r[3], r[4], r[5], p[1],
          r[6], r[7], r[8], p[2], 0, 0, 0, 1);
        group.matrixWorldNeedsUpdate = true;
        group.visible = meshes && !this.hiddenLinks.has(link.name);
        const marker = layer.frames.get(link.name);
        if (marker) {
          marker.matrix.copy(group.matrix);
          marker.matrixWorldNeedsUpdate = true;
          marker.visible = !this.hiddenLinks.has(link.name) &&
            (showFrames || !meshes || group.children.length === 0);
        }
      }
      layer.lines.visible = showFrames || !meshes || !this.model.visual_count;
      if (layer.lines.visible) {
        const points = pose.edges.filter((edge, i) => !this.hiddenLinks.has(
          this.model.joints[i]?.child ?? this.model.tips[i - this.model.joints.length]?.parent)).flat(2);
        layer.lines.geometry.dispose();
        layer.lines.geometry = new THREE.BufferGeometry();
        layer.lines.geometry.setAttribute("position", new THREE.Float32BufferAttribute(points, 3));
      }
    }
    const selectedJoint = this.model.joints.find(j => j.name === this.model.joint_names[selected]);
    const selectedLink = selectedJoint && this.layers.measured.links.get(selectedJoint.child);
    this.selection.visible = Boolean(layers.measured && meshes && selectedLink?.visible && selectedLink.children.length);
    this.scene.updateMatrixWorld(true);
    if (this.selection.visible) this.selection.setFromObject(selectedLink);
    this.renderer.render(this.scene, this.camera);
    // Displayed diagnostics are also accessible to browser acceptance checks.
    this.canvas.dataset.renderedLinks = String(this.model.links.length - this.hiddenLinks.size);
    this.canvas.dataset.triangles = String(this.renderer.info.render.triangles);
  }
}
