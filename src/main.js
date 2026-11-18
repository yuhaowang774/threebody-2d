const YOSHIDA = (() => {
  const w1 = 1 / (2 - Math.pow(2, 1 / 3));
  return [w1, w1, 1 - 2 * w1, w1];
})();

class GravityCage {
  constructor(config = {}) {
    this.enabled = config.enabled || false;
    this.boundaryRadius = config.boundaryRadius || 1600;
    this.warningRadius = config.warningRadius || 700;
    this.strength = config.strength || 1.0;
    this.exponent = config.exponent || 4;
    this.softening = config.softening || 0.001;
    this.maxMultiplier = config.maxMultiplier || 100;
    this.dampingFactor = config.dampingFactor || 0.1;
    this.center = { x: 0, y: 0 };
    this.cageEnergy = 0;
    this.showBoundaries = false;
    this.userShowBoundaries = false;
    this.boundaryHideTimer = null;
  }

  showBoundariesTemporarily() {
    if (this.userShowBoundaries) return;
    this.showBoundaries = true;
    if (this.boundaryHideTimer) {
      clearTimeout(this.boundaryHideTimer);
    }
    this.boundaryHideTimer = setTimeout(() => {
      if (!this.userShowBoundaries) {
        this.showBoundaries = false;
      }
    }, 5000);
  }

  setUserShowBoundaries(show) {
    this.userShowBoundaries = show;
    this.showBoundaries = show;
    if (show && this.boundaryHideTimer) {
      clearTimeout(this.boundaryHideTimer);
      this.boundaryHideTimer = null;
    }
  }

  updateCenter(com) {
    this.center.x = com.x;
    this.center.y = com.y;
  }

  computeDistance(body, displayScale) {
    const dx = (body.x - this.center.x) / displayScale;
    const dy = (body.y - this.center.y) / displayScale;
    return Math.sqrt(dx * dx + dy * dy);
  }

  computeDamping(body, displayScale) {
    if (!this.enabled) return { dampingX: 0, dampingY: 0 };
    const r = this.computeDistance(body, displayScale);
    const boundaryR = this.boundaryRadius / displayScale;
    const warningR = this.warningRadius / displayScale;
    if (r < warningR) return { dampingX: 0, dampingY: 0 };
    const effectiveR = r - warningR;
    const effectiveBoundary = boundaryR - warningR;
    const ratio = effectiveR / effectiveBoundary;
    const dampingStrength = this.dampingFactor * Math.pow(ratio, 2);
    return {
      dampingX: -dampingStrength * body.dx,
      dampingY: -dampingStrength * body.dy,
    };
  }

  computePotential(body, displayScale) {
    if (!this.enabled) return 0;
    const r = this.computeDistance(body, displayScale);
    const boundaryR = this.boundaryRadius / displayScale;
    const warningR = this.warningRadius / displayScale;
    if (r < warningR) return 0;
    const rNorm = r / boundaryR;
    if (rNorm >= 1 - this.softening) {
      return this.strength * body.m * this.maxMultiplier;
    }
    const effectiveR = r - warningR;
    const effectiveBoundary = boundaryR - warningR;
    const effectiveRNorm = effectiveR / effectiveBoundary;
    const factor =
      Math.pow(effectiveRNorm, this.exponent) /
      (1 - Math.pow(effectiveRNorm, this.exponent) + this.softening);
    return this.strength * body.m * Math.min(factor, this.maxMultiplier);
  }

  computeAcceleration(body, displayScale) {
    if (!this.enabled) return { ax: 0, ay: 0 };
    const dx = (body.x - this.center.x) / displayScale;
    const dy = (body.y - this.center.y) / displayScale;
    const r = Math.sqrt(dx * dx + dy * dy);
    if (r < 1e-10) return { ax: 0, ay: 0 };
    const boundaryR = this.boundaryRadius / displayScale;
    const warningR = this.warningRadius / displayScale;
    const rNorm = r / boundaryR;
    if (r < warningR) return { ax: 0, ay: 0 };
    if (rNorm >= 1 - this.softening) {
      const forceMag = (this.strength * this.maxMultiplier) / r;
      return { ax: (-forceMag * dx) / r, ay: (-forceMag * dy) / r };
    }
    const effectiveR = r - warningR;
    const effectiveBoundary = boundaryR - warningR;
    const effectiveRNorm = effectiveR / effectiveBoundary;
    const rNormExp = Math.pow(effectiveRNorm, this.exponent - 1);
    const denom =
      1 - Math.pow(effectiveRNorm, this.exponent) + this.softening;
    const denomSq = denom * denom;
    const factor =
      (this.exponent * rNormExp) / effectiveBoundary / denomSq;
    const forceMag =
      this.strength * Math.min(factor, this.maxMultiplier / r);
    return { ax: -forceMag * dx, ay: -forceMag * dy };
  }

  computeTotalCageEnergy(bodies, displayScale) {
    if (!this.enabled) return 0;
    let total = 0;
    for (const body of bodies) {
      total += this.computePotential(body, displayScale);
    }
    this.cageEnergy = total;
    return total;
  }

  getBoundaryStatus(body, displayScale) {
    const r = this.computeDistance(body, displayScale);
    const boundaryR = this.boundaryRadius / displayScale;
    const warningR = this.warningRadius / displayScale;
    const rNorm = r / boundaryR;
    const wNorm = r / warningR;
    if (rNorm >= 1 - this.softening) return "critical";
    if (rNorm >= 0.9) return "danger";
    if (wNorm >= 1) return "warning";
    return "safe";
  }

  getEffectiveStrength(body, displayScale) {
    if (!this.enabled) return 0;
    const r = this.computeDistance(body, displayScale);
    const boundaryR = this.boundaryRadius / displayScale;
    const warningR = this.warningRadius / displayScale;
    if (r < warningR) return 0;
    const rNorm = r / boundaryR;
    if (rNorm >= 1 - this.softening) return this.maxMultiplier;
    const effectiveR = r - warningR;
    const effectiveBoundary = boundaryR - warningR;
    const effectiveRNorm = effectiveR / effectiveBoundary;
    const factor =
      Math.pow(effectiveRNorm, this.exponent) /
      (1 - Math.pow(effectiveRNorm, this.exponent) + this.softening);
    return Math.min(factor, this.maxMultiplier);
  }
}

function calcKineticEnergy(m, vx, vy) {
  return 0.5 * m * (vx * vx + vy * vy);
}

function calcPotentialEnergy(m1, m2, distance, G) {
  return (-G * m1 * m2) / distance;
}

function calcAngularMomentum(m, x, y, vx, vy, centerX, centerY) {
  const rx = x - centerX;
  const ry = y - centerY;
  return m * (rx * vy - ry * vx);
}

function calcSystemAngularMomentum(bodies, centerX, centerY) {
  let L = 0;
  for (const b of bodies) {
    L += calcAngularMomentum(b.m, b.x, b.y, b.dx, b.dy, centerX, centerY);
  }
  return L;
}

function calcSystemEnergy(bodies, G, displayScale) {
  let kinetic = 0;
  let potential = 0;
  for (let i = 0; i < bodies.length; i++) {
    kinetic += calcKineticEnergy(bodies[i].m, bodies[i].dx, bodies[i].dy);
    for (let j = i + 1; j < bodies.length; j++) {
      const dx = (bodies[j].x - bodies[i].x) / displayScale;
      const dy = (bodies[j].y - bodies[i].y) / displayScale;
      const dist = Math.sqrt(dx * dx + dy * dy);
      potential += calcPotentialEnergy(bodies[i].m, bodies[j].m, dist, G);
    }
  }
  return { kinetic, potential, total: kinetic + potential };
}

const starTextures = [new Image(), new Image(), new Image()];
const starColors = ["#ffcc00", "#ff3333", "#aaccff"];
starTextures[0].src = "medres/ostar.JPG";
starTextures[1].src = "medres/mstar.JPG";
starTextures[2].src = "medres/kstar.JPG";

const glowTextureCache = new Map();
function createGlowTexture(color, baseRadius) {
  const key = `${color}_${baseRadius}`;
  if (glowTextureCache.has(key)) return glowTextureCache.get(key);
  const size = Math.ceil(baseRadius * 6);
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d");
  const cx = size / 2;
  const r = baseRadius;
  const glowRadius = r * 3;
  const coronaGrad = ctx.createRadialGradient(
    cx,
    cx,
    r,
    cx,
    cx,
    glowRadius,
  );
  coronaGrad.addColorStop(0, color + "66");
  coronaGrad.addColorStop(0.5, color + "22");
  coronaGrad.addColorStop(1, "transparent");
  ctx.beginPath();
  ctx.arc(cx, cx, glowRadius, 0, Math.PI * 2);
  ctx.fillStyle = coronaGrad;
  ctx.fill();
  const atmosphereGrad = ctx.createRadialGradient(
    cx,
    cx,
    r * 0.5,
    cx,
    cx,
    r * 1.5,
  );
  atmosphereGrad.addColorStop(0, color + "aa");
  atmosphereGrad.addColorStop(0.5, color + "44");
  atmosphereGrad.addColorStop(1, "transparent");
  ctx.beginPath();
  ctx.arc(cx, cx, r * 1.5, 0, Math.PI * 2);
  ctx.fillStyle = atmosphereGrad;
  ctx.fill();
  glowTextureCache.set(key, canvas);
  return canvas;
}

const collisionTextures = [
  "medres/fstar.JPG",
  "medres/gstar.JPG",
  "medres/kstar.JPG",
  "medres/mstar.JPG",
  "medres/ostar.JPG",
  "medres/astar.jpg",
  "medres/bstar.JPG",
];
const collisionTextureCache = {};

function getCollisionTexture(index) {
  if (!collisionTextureCache[index]) {
    const img = new Image();
    img.src = collisionTextures[index];
    collisionTextureCache[index] = img;
  }
  return collisionTextureCache[index];
}

function extractDominantColor(img) {
  try {
    const canvas = document.createElement("canvas");
    const ctx = canvas.getContext("2d");
    canvas.width = 32;
    canvas.height = 32;
    ctx.drawImage(img, 0, 0, 32, 32);
    const data = ctx.getImageData(0, 0, 32, 32).data;
    let r = 0,
      g = 0,
      b = 0,
      count = 0;
    for (let i = 0; i < data.length; i += 4) {
      const a = data[i + 3];
      if (a > 128) {
        r += data[i];
        g += data[i + 1];
        b += data[i + 2];
        count++;
      }
    }
    if (count === 0) return null;
    r = Math.round(r / count);
    g = Math.round(g / count);
    b = Math.round(b / count);
    return (
      "#" + [r, g, b].map((v) => v.toString(16).padStart(2, "0")).join("")
    );
  } catch (e) {
    return null;
  }
}

starTextures.forEach((img, i) => {
  img.onload = () => {
    const color = extractDominantColor(img);
    if (color) starColors[i] = color;
  };
});

const bgImage = new Image();
bgImage.src = "medres/eso0932a.webp";

const createBody = (x, y, dx, dy, m, textureIndex) => ({
  x,
  y,
  dx,
  dy,
  m,
  r: 5 + m * 2,
  textureIndex,
});

class CircularTrail {
  constructor(maxLength) {
    this.xs = new Float64Array(maxLength);
    this.ys = new Float64Array(maxLength);
    this.maxLength = maxLength;
    this.head = 0;
    this.count = 0;
  }
  push(x, y) {
    if (this.count < this.maxLength) {
      this.xs[this.count] = x;
      this.ys[this.count] = y;
      this.count++;
    } else {
      this.xs[this.head] = x;
      this.ys[this.head] = y;
      this.head = (this.head + 1) % this.maxLength;
    }
  }
  get length() {
    return this.count;
  }
  forEach(fn) {
    for (let i = 0; i < this.count; i++) {
      const idx = (this.head + i) % this.count;
      fn(this.xs[idx], this.ys[idx], i);
    }
  }
  clear() {
    this.head = 0;
    this.count = 0;
  }
  resize(newMaxLength) {
    if (newMaxLength === this.maxLength) return;
    const newXs = new Float64Array(newMaxLength);
    const newYs = new Float64Array(newMaxLength);
    const copyCount = Math.min(this.count, newMaxLength);
    const startOffset =
      this.count > newMaxLength ? this.count - newMaxLength : 0;
    for (let i = 0; i < copyCount; i++) {
      const srcIdx = (this.head + startOffset + i) % this.count;
      newXs[i] = this.xs[srcIdx];
      newYs[i] = this.ys[srcIdx];
    }
    this.xs = newXs;
    this.ys = newYs;
    this.maxLength = newMaxLength;
    this.head = 0;
    this.count = copyCount;
  }
}

class WebGLTrailRenderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.gl = canvas.getContext("webgl", {
      alpha: true,
      premultipliedAlpha: false,
    });
    if (!this.gl) {
      console.warn("WebGL not supported, falling back to 2D");
      this.enabled = false;
      return;
    }
    this.enabled = true;
    this.initShaders();
    this.initBuffers();
    this.resize();
  }
  initShaders() {
    const gl = this.gl;
    const vsSource = `
      attribute vec2 a_position;
      attribute float a_alpha;
      uniform vec2 u_resolution;
      uniform vec2 u_offset;
      uniform float u_scale;
      varying float v_alpha;
      void main() {
        vec2 screenPos = a_position * u_scale + u_offset;
        vec2 clipSpace = (screenPos / u_resolution) * 2.0 - 1.0;
        gl_Position = vec4(clipSpace * vec2(1, -1), 0, 1);
        v_alpha = a_alpha;
      }
    `;
    const fsSource = `
      precision mediump float;
      uniform vec4 u_color;
      varying float v_alpha;
      void main() {
        gl_FragColor = vec4(u_color.rgb, u_color.a * v_alpha);
      }
    `;
    const vs = this.createShader(gl.VERTEX_SHADER, vsSource);
    const fs = this.createShader(gl.FRAGMENT_SHADER, fsSource);
    this.program = gl.createProgram();
    gl.attachShader(this.program, vs);
    gl.attachShader(this.program, fs);
    gl.linkProgram(this.program);
    this.positionLoc = gl.getAttribLocation(this.program, "a_position");
    this.alphaLoc = gl.getAttribLocation(this.program, "a_alpha");
    this.resolutionLoc = gl.getUniformLocation(
      this.program,
      "u_resolution",
    );
    this.offsetLoc = gl.getUniformLocation(this.program, "u_offset");
    this.scaleLoc = gl.getUniformLocation(this.program, "u_scale");
    this.colorLoc = gl.getUniformLocation(this.program, "u_color");
  }
  createShader(type, source) {
    const gl = this.gl;
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    return shader;
  }
  initBuffers() {
    const gl = this.gl;
    this.positionBuffer = gl.createBuffer();
    this.alphaBuffer = gl.createBuffer();
    this.maxPoints = 2000;
    this.positions = new Float32Array(this.maxPoints * 2);
    this.alphas = new Float32Array(this.maxPoints);
  }
  resize() {
    this.canvas.width = window.innerWidth;
    this.canvas.height = window.innerHeight;
    this.gl.viewport(0, 0, this.canvas.width, this.canvas.height);
  }
  clear() {
    const gl = this.gl;
    gl.clearColor(0, 0, 0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT);
  }
  drawTrail(trail, color, offsetX, offsetY, scale) {
    if (!this.enabled || trail.length < 2) return;
    const gl = this.gl;
    const len = trail.length;
    let idx = 0;
    for (let i = 0; i < len; i++) {
      const bufIdx = (trail.head + i) % trail.count;
      this.positions[idx++] = trail.xs[bufIdx];
      this.positions[idx++] = trail.ys[bufIdx];
      this.alphas[i] = (i / len) * 0.8;
    }
    gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
    gl.bufferData(
      gl.ARRAY_BUFFER,
      this.positions.subarray(0, len * 2),
      gl.DYNAMIC_DRAW,
    );
    gl.bindBuffer(gl.ARRAY_BUFFER, this.alphaBuffer);
    gl.bufferData(
      gl.ARRAY_BUFFER,
      this.alphas.subarray(0, len),
      gl.DYNAMIC_DRAW,
    );
    gl.useProgram(this.program);
    gl.uniform2f(
      this.resolutionLoc,
      this.canvas.width,
      this.canvas.height,
    );
    gl.uniform2f(this.offsetLoc, offsetX, offsetY);
    gl.uniform1f(this.scaleLoc, scale);
    gl.uniform4f(this.colorLoc, color[0], color[1], color[2], color[3]);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
    gl.enableVertexAttribArray(this.positionLoc);
    gl.vertexAttribPointer(this.positionLoc, 2, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.alphaBuffer);
    gl.enableVertexAttribArray(this.alphaLoc);
    gl.vertexAttribPointer(this.alphaLoc, 1, gl.FLOAT, false, 0, 0);
    gl.lineWidth(2);
    gl.drawArrays(gl.LINE_STRIP, 0, len);
  }
  drawTrails(trails, colors, offsetX, offsetY, scale) {
    if (!this.enabled) return;
    this.clear();
    const gl = this.gl;
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    for (let i = 0; i < trails.length; i++) {
      if (trails[i] && trails[i].length > 1 && colors[i]) {
        this.drawTrail(trails[i], colors[i], offsetX, offsetY, scale);
      }
    }
  }
}

class StarField {
  constructor() {
    this.stars = [];
    this.generate();
  }

  generate() {
    const count = 50 + Math.floor(Math.random() * 151);
    this.stars = [];
    for (let i = 0; i < count; i++) {
      const depth = Math.random();
      const baseSize = 1 + Math.random() * 2;
      const size = baseSize * (0.3 + depth * 0.7);
      const baseBrightness = 0.6 + Math.random() * 0.4;
      const brightness = baseBrightness * (0.4 + depth * 0.6);
      const colorRand = Math.random();
      let color;
      if (colorRand < 0.6) {
        color = { r: 255, g: 255, b: 255 };
      } else if (colorRand < 0.85) {
        color = { r: 200, g: 220, b: 255 };
      } else if (colorRand < 0.95) {
        color = { r: 255, g: 240, b: 200 };
      } else {
        color = { r: 255, g: 200, b: 200 };
      }
      this.stars.push({
        screenX: Math.random(),
        screenY: Math.random(),
        size,
        brightness,
        depth,
        color,
        twinkleFreq: 0.1 + Math.random() * 0.4,
        twinklePhase: Math.random() * Math.PI * 2,
        twinkleAmp: 0.1 + Math.random() * 0.2,
      });
    }
  }

  draw(ctx, offsetX, offsetY, scale, time) {
    const viewWidth = ctx.canvas.width;
    const viewHeight = ctx.canvas.height;
    for (const star of this.stars) {
      const parallaxFactor = 0.1 + star.depth * 0.3;
      const baseX = star.screenX * viewWidth;
      const baseY = star.screenY * viewHeight;
      const screenX = baseX - offsetX * parallaxFactor * 0.01;
      const screenY = baseY - offsetY * parallaxFactor * 0.01;
      const wrappedX = ((screenX % viewWidth) + viewWidth) % viewWidth;
      const wrappedY = ((screenY % viewHeight) + viewHeight) % viewHeight;
      const twinkle = Math.sin(
        time * star.twinkleFreq * Math.PI * 2 + star.twinklePhase,
      );
      const currentBrightness =
        star.brightness * (1 + twinkle * star.twinkleAmp);
      const currentSize = star.size * (0.8 + twinkle * 0.1);
      const alpha = Math.min(1, Math.max(0.1, currentBrightness));
      const r = Math.round(star.color.r * currentBrightness);
      const g = Math.round(star.color.g * currentBrightness);
      const b = Math.round(star.color.b * currentBrightness);
      ctx.beginPath();
      ctx.arc(
        wrappedX,
        wrappedY,
        Math.max(0.5, currentSize),
        0,
        Math.PI * 2,
      );
      ctx.fillStyle = `rgba(${r},${g},${b},${alpha})`;
      ctx.fill();
      if (star.size > 1.5 && star.depth > 0.6) {
        const glowSize = currentSize * 2;
        const glow = ctx.createRadialGradient(
          wrappedX,
          wrappedY,
          0,
          wrappedX,
          wrappedY,
          glowSize,
        );
        glow.addColorStop(0, `rgba(${r},${g},${b},${alpha * 0.3})`);
        glow.addColorStop(1, `rgba(${r},${g},${b},0)`);
        ctx.beginPath();
        ctx.arc(wrappedX, wrappedY, glowSize, 0, Math.PI * 2);
        ctx.fillStyle = glow;
        ctx.fill();
      }
    }
  }
}

class NBodySim {
  constructor(canvas, bodies, G = 1, displayScale = 200) {
    Object.assign(this, {
      canvas,
      ctx: canvas.getContext("2d"),
      bodies,
      G,
      displayScale,
      dt: 0.001,
      speedMultiplier: 1,
      scale: 1,
      targetScale: 1,
      offsetX: 0,
      offsetY: 0,
      showCom: false,
      showTrail: true,
      useMinDist: true,
      enableCollision: false,
      mergingBodies: [],
      timer: null,
      totalTime: 0,
      trailLength: 1000,
      trails: bodies.map(() => new CircularTrail(1000)),
      lastTime: 0,
      domUpdateCounter: 0,
      domElements: {
        collisionInfo: null,
        collisionType: null,
        mergedMass: null,
        impactSpeed: null,
        momentumX: null,
        momentumY: null,
      },
      trailColors: [],
      autoZoom: {
        enabled: true,
        sensitivity: 0.01,
        minScale: 0.01,
        maxScale: 100,
        padding: 2,
        smoothing: 0.1,
        paused: false,
        pauseTimer: null,
        resumeDelay: 5000,
      },
    });
    this.initDomElements();
    this.initTrailColors();
    this.trailRenderer = new WebGLTrailRenderer(
      document.getElementById("trailCanvas"),
    );
    this.gravityCage = new GravityCage({
      enabled: true,
      boundaryRadius: 1600,
      warningRadius: 700,
      strength: 1.0,
      exponent: 4,
      softening: 0.001,
      maxMultiplier: 100,
    });
    const initCom = this.computeCenterOfMass();
    this.gravityCage.updateCenter(initCom);
    this.initState = structuredClone(bodies);
    this.starField = new StarField();
    this.resize();
    this.initEvents();
  }

  initDomElements() {
    const ids = [
      "timeDisplay",
      "collisionInfo",
      "collisionType",
      "mergedMass",
      "impactSpeed",
      "momentumX",
      "momentumY",
    ];
    for (const id of ids) {
      this.domElements[id] = document.getElementById(id);
    }
  }

  initTrailColors() {
    const hexToRgb = (hex) => {
      const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(
        hex,
      );
      return result
        ? [
            parseInt(result[1], 16) / 255,
            parseInt(result[2], 16) / 255,
            parseInt(result[3], 16) / 255,
            1.0,
          ]
        : [1, 1, 1, 1];
    };
    for (let i = 0; i < this.bodies.length; i++) {
      const body = this.bodies[i];
      const color =
        body.textureIndex >= 0
          ? starColors[body.textureIndex]
          : body.color || "#ffffff";
      this.trailColors[i] = hexToRgb(color);
    }
  }

  updateTrailColors() {
    const hexToRgb = (hex) => {
      const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(
        hex,
      );
      return result
        ? [
            parseInt(result[1], 16) / 255,
            parseInt(result[2], 16) / 255,
            parseInt(result[3], 16) / 255,
            1.0,
          ]
        : [1, 1, 1, 1];
    };
    for (let i = 0; i < this.bodies.length; i++) {
      const body = this.bodies[i];
      const color =
        body.textureIndex >= 0
          ? starColors[body.textureIndex]
          : body.color || "#ffffff";
      this.trailColors[i] = hexToRgb(color);
    }
  }

  resize() {
    this.canvas.width = window.innerWidth;
    this.canvas.height = window.innerHeight;
    if (this.trailRenderer) this.trailRenderer.resize();
    this.draw();
  }

  initEvents() {
    const c = this.canvas;
    c.addEventListener("wheel", (e) => {
      e.preventDefault();
      this.pauseAutoZoom();
      const com = this.computeCenterOfMass();
      const screenX = com.x * this.scale + this.offsetX;
      const screenY = com.y * this.scale + this.offsetY;
      const s = Math.max(
        0.01,
        Math.min(100, this.scale * (e.deltaY > 0 ? 0.9 : 1.1)),
      );
      this.scale = s;
      this.targetScale = s;
      this.offsetX = screenX - com.x * this.scale;
      this.offsetY = screenY - com.y * this.scale;
      this.draw();
    });
  }

  pauseAutoZoom() {
    const az = this.autoZoom;
    if (!az.enabled) return;
    az.paused = true;
    if (az.pauseTimer) {
      clearTimeout(az.pauseTimer);
    }
    az.pauseTimer = setTimeout(() => {
      az.paused = false;
      az.pauseTimer = null;
    }, az.resumeDelay);
  }

  computeCenterOfMass() {
    const bodies = this.bodies;
    let totalMass = 0,
      massWeightedX = 0,
      massWeightedY = 0;
    for (let i = 0; i < bodies.length; i++) {
      const body = bodies[i];
      totalMass += body.m;
      massWeightedX += body.m * body.x;
      massWeightedY += body.m * body.y;
    }
    return { x: massWeightedX / totalMass, y: massWeightedY / totalMass };
  }

  computeAcc(body, bodies) {
    const ds = this.displayScale;
    const minDist = 0.5;
    const minSq = minDist * minDist;
    let ax = 0,
      ay = 0;
    for (let i = 0; i < bodies.length; i++) {
      const b = bodies[i];
      if (b === body) continue;
      const dx = (b.x - body.x) / ds;
      const dy = (b.y - body.y) / ds;
      let distSq = dx * dx + dy * dy;
      if (this.useMinDist) distSq = Math.max(distSq, minSq);
      const dist = Math.sqrt(distSq);
      const force = (this.G * b.m) / (dist * distSq);
      ax += force * dx;
      ay += force * dy;
    }
    return { ax, ay };
  }

  computeCageAccelerations() {
    if (!this.gravityCage.enabled) return null;
    const accelerations = [];
    let totalForceX = 0;
    let totalForceY = 0;
    let totalMass = 0;
    for (let i = 0; i < this.bodies.length; i++) {
      const body = this.bodies[i];
      const cageAcc = this.gravityCage.computeAcceleration(
        body,
        this.displayScale,
      );
      const damping = this.gravityCage.computeDamping(
        body,
        this.displayScale,
      );
      accelerations.push({
        ax: cageAcc.ax + damping.dampingX,
        ay: cageAcc.ay + damping.dampingY,
      });
      totalForceX += (cageAcc.ax + damping.dampingX) * body.m;
      totalForceY += (cageAcc.ay + damping.dampingY) * body.m;
      totalMass += body.m;
    }
    const compensationAx = -totalForceX / totalMass;
    const compensationAy = -totalForceY / totalMass;
    for (let i = 0; i < accelerations.length; i++) {
      accelerations[i].ax += compensationAx;
      accelerations[i].ay += compensationAy;
    }
    return accelerations;
  }

  step() {
    const bodies = this.bodies;
    const dt = this.dt;
    const displayScale = this.displayScale;
    const yoshidaCoeffs = YOSHIDA;
    if (this.gravityCage.enabled) {
      const com = this.computeCenterOfMass();
      this.gravityCage.updateCenter(com);
    }
    for (
      let stepIndex = 0;
      stepIndex < yoshidaCoeffs.length;
      stepIndex++
    ) {
      const w = yoshidaCoeffs[stepIndex];
      const h = dt * w;
      const cageAccs = this.computeCageAccelerations();
      for (let i = 0; i < bodies.length; i++) {
        const body = bodies[i];
        const acc = this.computeAcc(body, bodies);
        let ax = acc.ax;
        let ay = acc.ay;
        if (cageAccs) {
          ax += cageAccs[i].ax;
          ay += cageAccs[i].ay;
        }
        body.dx += ax * h;
        body.dy += ay * h;
      }
      for (let i = 0; i < bodies.length; i++) {
        const body = bodies[i];
        body.x += body.dx * h * displayScale;
        body.y += body.dy * h * displayScale;
      }
    }
    this.totalTime += dt;
    this.updateTrails();
    if (this.enableCollision) {
      this.checkCollisions();
    }
  }

  checkBoundaryBreach() {
    if (!this.gravityCage.enabled) return false;
    const boundaryR = this.gravityCage.boundaryRadius;
    for (const body of this.bodies) {
      const dist = this.gravityCage.computeDistance(
        body,
        this.displayScale,
      );
      if (dist >= boundaryR / this.displayScale) {
        return true;
      }
    }
    return false;
  }

  checkCollisions() {
    const bodies = this.bodies;
    for (let i = 0; i < bodies.length; i++) {
      for (let j = i + 1; j < bodies.length; j++) {
        const b1 = bodies[i];
        const b2 = bodies[j];
        const dx = b2.x - b1.x;
        const dy = b2.y - b1.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        const collisionThreshold = b1.r + b2.r;
        if (dist < collisionThreshold && dist > 0) {
          const totalMass = b1.m + b2.m;
          const newBody = {
            x: (b1.x * b1.m + b2.x * b2.m) / totalMass,
            y: (b1.y * b1.m + b2.y * b2.m) / totalMass,
            dx: (b1.dx * b1.m + b2.dx * b2.m) / totalMass,
            dy: (b1.dy * b1.m + b2.dy * b2.m) / totalMass,
            m: totalMass,
            r: Math.pow(Math.pow(b1.r, 3) + Math.pow(b2.r, 3), 1 / 3),
            textureIndex: -1,
            color: b1.m >= b2.m ? b1.color : b2.color,
          };
          bodies.splice(j, 1);
          bodies[i] = newBody;
          this.trails.splice(j, 1);
          this.trails[i] = new CircularTrail(this.trailLength);
          this.lastCollisionInfo = {
            type: "merge",
            impactSpeed: Math.sqrt(
              Math.pow(b2.dx - b1.dx, 2) + Math.pow(b2.dy - b1.dy, 2),
            ),
            mergedMass: totalMass,
            momentumX: totalMass * newBody.dx,
            momentumY: totalMass * newBody.dy,
          };
          return;
        }
      }
    }
  }

  updateTrails() {
    if (!this.showTrail) return;
    for (let i = 0; i < this.bodies.length; i++) {
      const body = this.bodies[i];
      this.trails[i].push(body.x, body.y);
    }
  }

  draw() {
    const {
      ctx,
      canvas,
      bodies,
      scale,
      offsetX: ox,
      offsetY: oy,
      showCom,
      showTrail,
      trails,
    } = this;
    const time = performance.now() * 0.001;
    this.lastTime = time;
    ctx.fillStyle = "#000";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (this.starField) {
      this.starField.draw(ctx, ox, oy, scale, time);
    }
    if (bgImage.complete && bgImage.naturalWidth > 0) {
      const imgRatio = bgImage.naturalWidth / bgImage.naturalHeight;
      const canvasRatio = canvas.width / canvas.height;
      let drawW, drawH, drawX, drawY;
      if (imgRatio > canvasRatio) {
        drawH = canvas.height;
        drawW = drawH * imgRatio;
        drawX = (canvas.width - drawW) / 2;
        drawY = 0;
      } else {
        drawW = canvas.width;
        drawH = drawW / imgRatio;
        drawX = 0;
        drawY = (canvas.height - drawH) / 2;
      }
      ctx.globalAlpha = 0.3;
      ctx.drawImage(bgImage, drawX, drawY, drawW, drawH);
      ctx.globalAlpha = 1;
    }
    ctx.save();
    ctx.translate(ox, oy);
    ctx.scale(scale, scale);
    if (showTrail) {
      const segments = 240;
      for (let i = 0; i < bodies.length; i++) {
        const trail = trails[i];
        if (trail.length < 2) continue;
        const body = bodies[i];
        const bodyColor =
          body.textureIndex >= 0
            ? starColors[body.textureIndex]
            : body.color || "#ffffff";
        ctx.strokeStyle = bodyColor;
        ctx.lineWidth = 2 / scale;
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        const len = trail.length;
        const segmentSize = Math.max(1, Math.ceil(len / segments));
        for (let s = 0; s < segments; s++) {
          const start = s === 0 ? 0 : s * segmentSize - 1;
          const end = Math.min(s * segmentSize + segmentSize, len);
          if (start >= len) break;
          const progress = (s * segmentSize + segmentSize / 2) / len;
          const alpha = Math.pow(progress, 0.7) * 0.85;
          ctx.globalAlpha = alpha;
          ctx.beginPath();
          for (let j = start; j < end; j++) {
            const idx = (trail.head + j) % trail.count;
            if (j === start) {
              ctx.moveTo(trail.xs[idx], trail.ys[idx]);
            } else {
              ctx.lineTo(trail.xs[idx], trail.ys[idx]);
            }
          }
          ctx.stroke();
        }
        ctx.globalAlpha = 1;
      }
    }
    for (const b of bodies) {
      const r = b.r;
      const x = b.x;
      const y = b.y;
      const brightness = 0.6 + b.m * 0.1;
      const glowColor =
        b.textureIndex >= 0
          ? starColors[b.textureIndex]
          : b.color || "#ffffff";
      const glowTexture = createGlowTexture(glowColor, r);
      const glowSize = glowTexture.width;
      ctx.drawImage(
        glowTexture,
        x - glowSize / 2,
        y - glowSize / 2,
        glowSize,
        glowSize,
      );
      ctx.save();
      ctx.beginPath();
      ctx.arc(x, y, r, 0, Math.PI * 2);
      ctx.clip();
      const texture = starTextures[b.textureIndex];
      if (texture && texture.complete && texture.naturalWidth > 0) {
        const rotation = time * 0.1;
        ctx.translate(x, y);
        ctx.rotate(rotation);
        ctx.drawImage(texture, -r, -r, r * 2, r * 2);
        ctx.rotate(-rotation);
        ctx.translate(-x, -y);
      } else {
        const coreGrad = ctx.createRadialGradient(
          x - r * 0.2,
          y - r * 0.2,
          0,
          x,
          y,
          r,
        );
        coreGrad.addColorStop(0, "#ffffff");
        coreGrad.addColorStop(0.3, "#ffffff");
        coreGrad.addColorStop(0.6, glowColor);
        coreGrad.addColorStop(1, glowColor + "ee");
        ctx.fillStyle = coreGrad;
        ctx.fill();
      }
      ctx.restore();
      ctx.beginPath();
      ctx.arc(x, y, r * 1.2, 0, Math.PI * 2);
      ctx.strokeStyle = "rgba(255,255,255,0.08)";
      ctx.lineWidth = r * 0.1;
      ctx.stroke();
    }
    if (showCom) {
      const com = this.computeCenterOfMass();
      const comRadius = 6;
      const pulse = 1 + Math.sin(time * 3) * 0.15;
      const rotateAngle = time * 0.5;
      const outerGlow = ctx.createRadialGradient(
        com.x,
        com.y,
        0,
        com.x,
        com.y,
        comRadius * 6 * pulse,
      );
      outerGlow.addColorStop(0, "rgba(0,255,255,0.3)");
      outerGlow.addColorStop(0.3, "rgba(0,200,255,0.15)");
      outerGlow.addColorStop(0.6, "rgba(100,150,255,0.05)");
      outerGlow.addColorStop(1, "transparent");
      ctx.beginPath();
      ctx.arc(com.x, com.y, comRadius * 6 * pulse, 0, Math.PI * 2);
      ctx.fillStyle = outerGlow;
      ctx.fill();
      const innerGlow = ctx.createRadialGradient(
        com.x,
        com.y,
        0,
        com.x,
        com.y,
        comRadius * 2.5,
      );
      innerGlow.addColorStop(0, "rgba(255,255,255,0.9)");
      innerGlow.addColorStop(0.3, "rgba(150,255,255,0.6)");
      innerGlow.addColorStop(0.7, "rgba(0,200,255,0.3)");
      innerGlow.addColorStop(1, "transparent");
      ctx.beginPath();
      ctx.arc(com.x, com.y, comRadius * 2.5, 0, Math.PI * 2);
      ctx.fillStyle = innerGlow;
      ctx.fill();
      ctx.beginPath();
      ctx.arc(com.x, com.y, comRadius, 0, Math.PI * 2);
      const coreGrad = ctx.createRadialGradient(
        com.x - comRadius * 0.3,
        com.y - comRadius * 0.3,
        0,
        com.x,
        com.y,
        comRadius,
      );
      coreGrad.addColorStop(0, "#ffffff");
      coreGrad.addColorStop(0.5, "#aaffff");
      coreGrad.addColorStop(1, "#00ddff");
      ctx.fillStyle = coreGrad;
      ctx.fill();
      ctx.strokeStyle = "rgba(0,255,255,0.8)";
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.save();
      ctx.translate(com.x, com.y);
      ctx.rotate(rotateAngle);
      for (let i = 0; i < 4; i++) {
        ctx.rotate(Math.PI / 2);
        ctx.beginPath();
        ctx.moveTo(comRadius + 3, 0);
        ctx.lineTo(comRadius + 10 + Math.sin(time * 4) * 2, 0);
        const lineGrad = ctx.createLinearGradient(
          comRadius + 3,
          0,
          comRadius + 12,
          0,
        );
        lineGrad.addColorStop(0, "rgba(0,255,255,0.8)");
        lineGrad.addColorStop(1, "rgba(0,255,255,0)");
        ctx.strokeStyle = lineGrad;
        ctx.lineWidth = 2;
        ctx.stroke();
      }
      ctx.restore();
      ctx.beginPath();
      ctx.arc(com.x, com.y, comRadius * 3.5, 0, Math.PI * 2);
      ctx.strokeStyle = `rgba(0,255,255,${0.2 + Math.sin(time * 2) * 0.1})`;
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    if (this.gravityCage.enabled && this.gravityCage.showBoundaries) {
      const cage = this.gravityCage;
      const cx = cage.center.x;
      const cy = cage.center.y;
      const warningR = cage.warningRadius;
      const boundaryR = cage.boundaryRadius;
      ctx.beginPath();
      ctx.arc(cx, cy, warningR, 0, Math.PI * 2);
      ctx.strokeStyle = `rgba(255, 200, 50, ${0.3 + Math.sin(time * 2) * 0.1})`;
      ctx.lineWidth = 2 / scale;
      ctx.setLineDash([10, 5]);
      ctx.stroke();
      ctx.setLineDash([]);
      const warningGrad = ctx.createRadialGradient(
        cx,
        cy,
        warningR - 50,
        cx,
        cy,
        warningR + 50,
      );
      warningGrad.addColorStop(0, "transparent");
      warningGrad.addColorStop(0.5, "rgba(255, 200, 50, 0.05)");
      warningGrad.addColorStop(1, "transparent");
      ctx.beginPath();
      ctx.arc(cx, cy, warningR, 0, Math.PI * 2);
      ctx.fillStyle = warningGrad;
      ctx.fill();
      const pulseIntensity = 0.4 + Math.sin(time * 3) * 0.2;
      const boundaryGrad = ctx.createRadialGradient(
        cx,
        cy,
        boundaryR - 100,
        cx,
        cy,
        boundaryR + 100,
      );
      boundaryGrad.addColorStop(0, "transparent");
      boundaryGrad.addColorStop(
        0.7,
        `rgba(255, 50, 50, ${pulseIntensity * 0.1})`,
      );
      boundaryGrad.addColorStop(
        1,
        `rgba(255, 50, 50, ${pulseIntensity * 0.3})`,
      );
      ctx.beginPath();
      ctx.arc(cx, cy, boundaryR, 0, Math.PI * 2);
      ctx.fillStyle = boundaryGrad;
      ctx.fill();
      ctx.beginPath();
      ctx.arc(cx, cy, boundaryR, 0, Math.PI * 2);
      ctx.strokeStyle = `rgba(255, 50, 50, ${pulseIntensity})`;
      ctx.lineWidth = 3 / scale;
      ctx.stroke();
    }

    ctx.restore();
    this.domUpdateCounter++;
    if (this.domUpdateCounter >= 10) {
      this.domUpdateCounter = 0;
      this.updateTimeDisplay();
      this.updateEnergyDisplay();
    }
  }

  updateTimeDisplay() {
    const el = this.domElements.timeDisplay;
    if (!el) return;
    let t = this.totalTime;
    if (t < 60) {
      el.textContent = t.toFixed(2) + "秒";
    } else if (t < 3600) {
      const m = Math.floor(t / 60);
      const s = (t % 60).toFixed(1);
      el.textContent = m + "分" + s + "秒";
    } else {
      const h = Math.floor(t / 3600);
      const m = Math.floor((t % 3600) / 60);
      const s = ((t % 3600) % 60).toFixed(0);
      el.textContent = h + "时" + m + "分" + s + "秒";
    }
  }

  updateEnergyDisplay() {
    if (this.lastCollisionInfo) {
      const {
        collisionInfo,
        collisionType,
        mergedMass,
        impactSpeed,
        momentumX,
        momentumY,
      } = this.domElements;
      if (collisionInfo) collisionInfo.style.display = "block";
      if (collisionType) collisionType.textContent = "合并";
      if (mergedMass)
        mergedMass.textContent =
          this.lastCollisionInfo.mergedMass.toFixed(4);
      if (impactSpeed)
        impactSpeed.textContent =
          this.lastCollisionInfo.impactSpeed.toFixed(4);
      if (momentumX)
        momentumX.textContent =
          this.lastCollisionInfo.momentumX.toFixed(4);
      if (momentumY)
        momentumY.textContent =
          this.lastCollisionInfo.momentumY.toFixed(4);
    }
  }

  run() {
    const sim = this;
    function frame() {
      for (let i = 0; i < sim.speedMultiplier; i++) {
        sim.step();
      }
      sim.centerOnCom();
      sim.updateAutoZoom();
      sim.draw();
      sim.timer = requestAnimationFrame(frame);
    }
    frame();
  }

  autoReset(reason) {
    const newBodies = generateRandomBodies();
    this.bodies = newBodies;
    this.totalTime = 0;
    this.trails = newBodies.map(
      () => new CircularTrail(this.trailLength),
    );
    this.scale = 1;
    this.initState = structuredClone(newBodies);
    const com = this.computeCenterOfMass();
    this.gravityCage.updateCenter(com);
    this.centerOnCom();
    updateControlPanel(newBodies);
    this.draw();
  }

  start() {
    if (!this.timer) this.run();
  }
  stop() {
    cancelAnimationFrame(this.timer);
    this.timer = null;
  }

  centerOnCom() {
    const com = this.computeCenterOfMass();
    this.offsetX = this.canvas.width / 2 - com.x * this.scale;
    this.offsetY = this.canvas.height / 2 - com.y * this.scale;
  }

  computeBoundingSphere() {
    const bodies = this.bodies;
    if (bodies.length === 0)
      return { centerX: 0, centerY: 0, radius: 100 };
    if (bodies.length === 1) {
      return {
        centerX: bodies[0].x,
        centerY: bodies[0].y,
        radius: bodies[0].r + 50,
      };
    }
    const com = this.computeCenterOfMass();
    let maxDist = 0;
    for (const body of bodies) {
      const dx = body.x - com.x;
      const dy = body.y - com.y;
      const dist = Math.sqrt(dx * dx + dy * dy) + body.r;
      if (dist > maxDist) maxDist = dist;
    }
    return {
      centerX: com.x,
      centerY: com.y,
      radius: Math.max(maxDist, 50),
    };
  }

  computeMaxBodyDistance() {
    const bodies = this.bodies;
    let maxDist = 0;
    for (let i = 0; i < bodies.length; i++) {
      for (let j = i + 1; j < bodies.length; j++) {
        const dx = bodies[j].x - bodies[i].x;
        const dy = bodies[j].y - bodies[i].y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist > maxDist) maxDist = dist;
      }
    }
    return maxDist;
  }

  computeTargetScale() {
    const az = this.autoZoom;
    if (!az.enabled) return this.scale;
    const bounding = this.computeBoundingSphere();
    const canvasSize = Math.min(this.canvas.width, this.canvas.height);
    const targetRadius = bounding.radius * az.padding;
    let targetScale = (canvasSize * 0.45) / targetRadius;
    targetScale = Math.max(
      az.minScale,
      Math.min(az.maxScale, targetScale),
    );
    return targetScale;
  }

  updateAutoZoom() {
    const az = this.autoZoom;
    if (!az.enabled || az.paused) return;
    this.targetScale = this.computeTargetScale();
    const diff = this.targetScale - this.scale;
    if (Math.abs(diff) > 0.0001) {
      const step = diff * az.sensitivity;
      this.scale += step;
      const com = this.computeCenterOfMass();
      const screenX = this.canvas.width / 2;
      const screenY = this.canvas.height / 2;
      this.offsetX = screenX - com.x * this.scale;
      this.offsetY = screenY - com.y * this.scale;
    }
  }
}

const canvas = document.getElementById("canvas");
const cx = window.innerWidth / 2;
const cy = window.innerHeight / 2;
const scale = 200;

function generateRandomBodies() {
  const G = 1;
  const bodies = [];
  const masses = [];
  const positions = [];
  const velocities = [];

  for (let i = 0; i < 3; i++) {
    const mass = 0.5 + Math.random() * 4.5;
    masses.push(mass);
    const angle = (i * 2 * Math.PI) / 3 + Math.random() * 0.5;
    const radius = 200 + Math.random() * 300;
    positions.push({
      x: cx + radius * Math.cos(angle),
      y: cy + radius * Math.sin(angle),
    });
  }

  let potentialEnergy = 0;
  for (let i = 0; i < 3; i++) {
    for (let j = i + 1; j < 3; j++) {
      const dx = (positions[j].x - positions[i].x) / scale;
      const dy = (positions[j].y - positions[i].y) / scale;
      const dist = Math.max(Math.sqrt(dx * dx + dy * dy), 0.5);
      potentialEnergy -= (G * masses[i] * masses[j]) / dist;
    }
  }

  const targetKinetic = Math.abs(potentialEnergy) * 0.3;
  const kineticPerBody = targetKinetic / 3;

  for (let i = 0; i < 3; i++) {
    const speed = Math.sqrt((2 * kineticPerBody) / masses[i]);
    const velAngle = Math.random() * 2 * Math.PI;
    velocities.push({
      dx: speed * Math.cos(velAngle),
      dy: speed * Math.sin(velAngle),
    });
  }

  const totalMomentumX = masses.reduce(
    (sum, m, i) => sum + m * velocities[i].dx,
    0,
  );
  const totalMomentumY = masses.reduce(
    (sum, m, i) => sum + m * velocities[i].dy,
    0,
  );
  for (let i = 0; i < 3; i++) {
    velocities[i].dx -= totalMomentumX / (3 * masses[i]);
    velocities[i].dy -= totalMomentumY / (3 * masses[i]);
  }

  for (let i = 0; i < 3; i++) {
    bodies.push(
      createBody(
        positions[i].x,
        positions[i].y,
        velocities[i].dx,
        velocities[i].dy,
        masses[i],
        i,
      ),
    );
  }
  return bodies;
}

const bodies = generateRandomBodies();
const sim = new NBodySim(canvas, bodies, 1, scale);

function updateControlPanel(bodies) {
  bodies.forEach((b, i) => {
    document.getElementById(`m${i + 1}`).value = b.m.toFixed(2);
    document.getElementById(`v${i + 1}x`).value = b.dx.toFixed(4);
    document.getElementById(`v${i + 1}y`).value = b.dy.toFixed(4);
  });
}

sim.centerOnCom();
sim.draw();
updateControlPanel(bodies);
sim.start();

const toggleBtn = document.getElementById("togglePanel");
const controlPanel = document.getElementById("controlPanel");
const panelContainer = document.getElementById("panelContainer");
let panelExpanded = false;

let isDragging = false;
let hasMoved = false;
let dragStartX, dragStartY, containerStartX, containerStartY;

panelContainer.addEventListener("mousedown", (e) => {
  if (e.target.tagName === "INPUT") return;
  isDragging = true;
  hasMoved = false;
  dragStartX = e.clientX;
  dragStartY = e.clientY;
  const rect = panelContainer.getBoundingClientRect();
  containerStartX = rect.left;
  containerStartY = rect.top;
  panelContainer.style.cursor = "grabbing";
});

document.addEventListener("mousemove", (e) => {
  if (!isDragging) return;
  const deltaX = e.clientX - dragStartX;
  const deltaY = e.clientY - dragStartY;
  if (Math.abs(deltaX) > 3 || Math.abs(deltaY) > 3) {
    hasMoved = true;
  }
  const newX = containerStartX + deltaX;
  const newY = containerStartY + deltaY;
  panelContainer.style.left = newX + "px";
  panelContainer.style.top = newY + "px";
  panelContainer.style.right = "auto";
  panelContainer.style.transform = "none";
});

document.addEventListener("mouseup", () => {
  if (isDragging) {
    isDragging = false;
    panelContainer.style.cursor = "move";
  }
});

toggleBtn.addEventListener("click", (e) => {
  if (hasMoved) {
    hasMoved = false;
    return;
  }
  e.stopPropagation();
  panelExpanded = !panelExpanded;
  controlPanel.classList.toggle("show", panelExpanded);
  toggleBtn.classList.toggle("expanded", panelExpanded);
  toggleBtn.textContent = panelExpanded ? "✕" : "⚙";
});

document.getElementById("start").onclick = () => sim.start();
document.getElementById("stop").onclick = () => sim.stop();

function resetSimulation(newBodies) {
  sim.stop();
  sim.bodies = newBodies;
  sim.totalTime = 0;
  sim.trails = newBodies.map(() => new CircularTrail(sim.trailLength));
  sim.scale = 1;
  sim.centerOnCom();
  updateControlPanel(newBodies);
  sim.draw();
}

document.getElementById("reset").onclick = () => {
  const currentG = sim.G;
  const currentDt = sim.dt;
  const currentSpeed = sim.speedMultiplier;
  const newBodies = generateRandomBodies();
  sim.initState = structuredClone(newBodies);
  resetSimulation(newBodies);
  sim.G = currentG;
  sim.dt = currentDt;
  sim.speedMultiplier = currentSpeed;
  sim.start();
};

document.getElementById("restore").onclick = () => {
  resetSimulation(structuredClone(sim.initState));
  sim.start();
};

document.getElementById("speedRange").oninput = (e) => {
  document.getElementById("speedVal").textContent = e.target.value + "x";
  document.getElementById("speedInput").value = e.target.value;
  sim.speedMultiplier = +e.target.value;
};
document.getElementById("speedInput").oninput = (e) => {
  let val = Math.max(1, Math.min(50, +e.target.value || 1));
  document.getElementById("speedVal").textContent = val + "x";
  document.getElementById("speedRange").value = val;
  sim.speedMultiplier = val;
};
document.getElementById("btnTrail").onclick = (e) => {
  sim.showTrail = !sim.showTrail;
  e.target.classList.toggle("active", sim.showTrail);
};
function updatePhysicsRealismButton() {
  const isPhysicsRealism =
    !sim.useMinDist && sim.enableCollision && !sim.gravityCage.enabled;
  document
    .getElementById("btnCom")
    .classList.toggle("active", isPhysicsRealism);
}
document.getElementById("btnCom").onclick = (e) => {
  const isActive = e.target.classList.toggle("active");
  if (isActive) {
    sim._savedUseMinDist = sim.useMinDist;
    sim._savedEnableCollision = sim.enableCollision;
    sim._savedCageEnabled = sim.gravityCage.enabled;
    sim.useMinDist = false;
    sim.enableCollision = true;
    sim.gravityCage.enabled = false;
    document.getElementById("btnMinDist").classList.remove("active");
    document.getElementById("btnCollision").classList.add("active");
    document.getElementById("btnCageEnabled").classList.remove("active");
  } else {
    sim.useMinDist =
      sim._savedUseMinDist !== undefined ? sim._savedUseMinDist : true;
    sim.enableCollision =
      sim._savedEnableCollision !== undefined
        ? sim._savedEnableCollision
        : false;
    sim.gravityCage.enabled =
      sim._savedCageEnabled !== undefined ? sim._savedCageEnabled : true;
    document
      .getElementById("btnMinDist")
      .classList.toggle("active", sim.useMinDist);
    document
      .getElementById("btnCollision")
      .classList.toggle("active", sim.enableCollision);
    document
      .getElementById("btnCageEnabled")
      .classList.toggle("active", sim.gravityCage.enabled);
  }
};
document.getElementById("btnMinDist").onclick = (e) => {
  sim.useMinDist = !sim.useMinDist;
  e.target.classList.toggle("active", sim.useMinDist);
  updatePhysicsRealismButton();
};
document.getElementById("btnCollision").onclick = (e) => {
  sim.enableCollision = !sim.enableCollision;
  e.target.classList.toggle("active", sim.enableCollision);
  updatePhysicsRealismButton();
};
document.getElementById("trailLength").oninput = (e) => {
  let val = Math.max(10, Math.min(5000, +e.target.value || 100));
  document.getElementById("trailLengthVal").textContent = val;
  document.getElementById("trailLengthRange").value = val;
  sim.trailLength = val;
  sim.trails.forEach((trail) => trail.resize(val));
};
document.getElementById("trailLengthRange").oninput = (e) => {
  let val = Math.max(10, Math.min(5000, +e.target.value || 100));
  document.getElementById("trailLengthVal").textContent = val;
  document.getElementById("trailLength").value = val;
  sim.trailLength = val;
  sim.trails.forEach((trail) => trail.resize(val));
};

function updateBodyParams() {
  const getVal = (id) => +document.getElementById(id).value || 0;
  sim.bodies.forEach((b, i) => {
    b.m = getVal(`m${i + 1}`) || 1;
    b.dx = getVal(`v${i + 1}x`);
    b.dy = getVal(`v${i + 1}y`);
    b.r = 5 + b.m * 2;
  });
  sim.G = getVal("gravity") || 1;
  if (!sim.timer) sim.draw();
}

[
  "m1",
  "m2",
  "m3",
  "v1x",
  "v1y",
  "v2x",
  "v2y",
  "v3x",
  "v3y",
  "gravity",
].forEach((id) => {
  document.getElementById(id).oninput = updateBodyParams;
});

document.getElementById("btnCageEnabled").onclick = (e) => {
  sim.gravityCage.enabled = !sim.gravityCage.enabled;
  e.target.classList.toggle("active", sim.gravityCage.enabled);
  if (sim.gravityCage.enabled) {
    const com = sim.computeCenterOfMass();
    sim.gravityCage.updateCenter(com);
    sim.gravityCage.showBoundariesTemporarily();
  }
  if (!sim.timer) sim.draw();
  updatePhysicsRealismButton();
};

document.getElementById("btnCageShowBoundaries").onclick = (e) => {
  const newState = !sim.gravityCage.userShowBoundaries;
  sim.gravityCage.setUserShowBoundaries(newState);
  e.target.classList.toggle("active", newState);
  if (!sim.timer) sim.draw();
};

const syncCageParam = (
  id,
  rangeId,
  prop,
  parser = parseFloat,
  showBoundary = false,
) => {
  const input = document.getElementById(id);
  const range = document.getElementById(rangeId);
  const update = (v) => {
    const val = parser(v);
    sim.gravityCage[prop] = val;
    if (input) input.value = val;
    if (range) range.value = val;
    if (showBoundary) {
      sim.gravityCage.showBoundariesTemporarily();
    }
    if (!sim.timer) sim.draw();
  };
  if (input) input.oninput = (e) => update(e.target.value);
  if (range) range.oninput = (e) => update(e.target.value);
};

syncCageParam(
  "cageBoundary",
  "cageBoundaryRange",
  "boundaryRadius",
  parseInt,
  true,
);
syncCageParam(
  "cageWarning",
  "cageWarningRange",
  "warningRadius",
  parseInt,
  true,
);

document.getElementById("btnAutoZoom").onclick = (e) => {
  sim.autoZoom.enabled = !sim.autoZoom.enabled;
  e.target.classList.toggle("active", sim.autoZoom.enabled);
  if (!sim.timer) sim.draw();
};

document.getElementById("resetGravityCage").onclick = (e) => {
  const btn = e.target;
  btn.style.background =
    "linear-gradient(135deg, #2a5a2a 0%, #1a4a1a 100%)";
  btn.style.borderColor = "#4caf50";
  btn.style.color = "#fff";
  setTimeout(() => {
    btn.style.background = "";
    btn.style.borderColor = "";
    btn.style.color = "";
  }, 200);
  document.getElementById("btnCageEnabled").classList.add("active");
  document
    .getElementById("btnCageShowBoundaries")
    .classList.remove("active");
  sim.gravityCage.enabled = true;
  const defaults = {
    boundaryRadius: 1600,
    warningRadius: 700,
  };
  sim.gravityCage.boundaryRadius = defaults.boundaryRadius;
  sim.gravityCage.warningRadius = defaults.warningRadius;
  document.getElementById("cageBoundary").value = defaults.boundaryRadius;
  document.getElementById("cageBoundaryRange").value =
    defaults.boundaryRadius;
  document.getElementById("cageWarning").value = defaults.warningRadius;
  document.getElementById("cageWarningRange").value =
    defaults.warningRadius;
  sim.gravityCage.setUserShowBoundaries(false);
  if (!sim.timer) sim.draw();
};

const ScaleBar = {
  lineEl: null,
  zoomEl: null,
  baseWidth: 0,
  currentWidth: 0,
  targetWidth: 0,
  lastScale: 1,
  init() {
    this.lineEl = document.getElementById("scaleBarLine");
    this.zoomEl = document.getElementById("scaleZoomIndicator");
    this.baseWidth = window.innerWidth / 2;
    this.currentWidth = this.baseWidth;
    this.targetWidth = this.baseWidth;
    this.update(1);
  },
  update(scale) {
    if (!this.lineEl) return;
    const scaleDiff = Math.abs(scale - this.lastScale);
    if (
      scaleDiff < 0.001 &&
      Math.abs(this.currentWidth - this.targetWidth) < 1
    ) {
      return;
    }
    this.lastScale = scale;
    this.targetWidth = this.baseWidth * scale;
    this.currentWidth += (this.targetWidth - this.currentWidth) * 0.2;
    if (Math.abs(this.currentWidth - this.targetWidth) < 1) {
      this.currentWidth = this.targetWidth;
    }
    this.lineEl.style.width = this.currentWidth + "px";
    if (this.zoomEl) {
      this.zoomEl.textContent = scale.toFixed(2) + "x";
    }
  },
};

ScaleBar.init();

setInterval(() => {
  ScaleBar.update(sim.scale);
}, 100);

window.addEventListener("resize", () => sim.resize());
