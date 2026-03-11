# 引力软化和引力牢笼技术文档

## 一、引力软化 (Gravity Softening)

### 1.1 问题背景

在 N 体模拟中，当两个天体距离非常接近时，引力公式 `F = G*m1*m2/r²` 会导致加速度趋向无穷大，造成数值不稳定。

### 1.2 解决方案

通过设置最小距离阈值，限制距离平方的最小值：

```javascript
const minDist = 0.5;
const minSq = minDist * minDist;

computeAcc(body, bodies) {
  for (const b of bodies) {
    if (b === body) continue;
    const dx = (b.x - body.x) / displayScale;
    const dy = (b.y - body.y) / displayScale;
    let distSq = dx * dx + dy * dy;
    
    // 软化：限制最小距离
    if (this.useMinDist) distSq = Math.max(distSq, minSq);
    
    const dist = Math.sqrt(distSq);
    const force = (this.G * b.m) / (dist * distSq);
    ax += force * dx;
    ay += force * dy;
  }
}
```

### 1.3 数学原理

| 无软化 | 有软化 |
|--------|--------|
| `F ∝ 1/r²` | `F ∝ 1/max(r², r_min²)` |

当 `r < r_min` 时，力不再发散，而是保持恒定值 `F = G*m/r_min²`。

### 1.4 移植要点

```javascript
class NBodySim {
  constructor() {
    this.useMinDist = true;    // 是否启用软化
    this.minDist = 0.5;        // 最小距离阈值
  }

  computeAcceleration(body, bodies) {
    const minSq = this.minDist * this.minDist;
    // ... 计算加速度时应用软化
  }
}
```

---

## 二、引力牢笼 (Gravity Cage)

### 2.1 问题背景

开放边界条件下，天体可能逃逸到无穷远处，导致模拟失去意义。需要一种机制将天体约束在有限区域内。

### 2.2 核心思想

在质心周围创建一个"软边界"，当天体接近边界时施加向内的排斥力，且距离越近力越大。

### 2.3 类结构

```javascript
class GravityCage {
  constructor(config = {}) {
    this.enabled = config.enabled || false;           // 是否启用
    this.boundaryRadius = config.boundaryRadius || 1600;  // 边界半径(像素)
    this.warningRadius = config.warningRadius || 700;     // 警告半径(像素)
    this.strength = config.strength || 1.0;            // 力强度
    this.exponent = config.exponent || 4;              // 力增长指数
    this.softening = config.softening || 0.001;        // 数值软化
    this.maxMultiplier = config.maxMultiplier || 100;  // 最大力乘数
    this.dampingFactor = config.dampingFactor || 0.1;  // 阻尼系数
    this.center = { x: 0, y: 0 };                      // 牢笼中心(跟随质心)
  }
}
```

### 2.4 力的计算

#### 归一化距离

```
rNorm = r / boundaryR           // 相对于边界的位置
effectiveRNorm = (r - warningR) / (boundaryR - warningR)  // 有效区域内的位置
```

#### 势能函数

```javascript
computePotential(body, displayScale) {
  if (r < warningR) return 0;  // 警告半径内无势能
  
  // 接近边界时势能趋向无穷
  const factor = Math.pow(effectiveRNorm, this.exponent) / 
                 (1 - Math.pow(effectiveRNorm, this.exponent) + this.softening);
  
  return this.strength * body.m * Math.min(factor, this.maxMultiplier);
}
```

#### 加速度计算

```javascript
computeAcceleration(body, displayScale) {
  if (r < warningR) return { ax: 0, ay: 0 };  // 警告半径内无力
  
  // 力的梯度（势能对距离的导数）
  const rNormExp = Math.pow(effectiveRNorm, this.exponent - 1);
  const denom = 1 - Math.pow(effectiveRNorm, this.exponent) + this.softening;
  const factor = (this.exponent * rNormExp) / effectiveBoundary / (denom * denom);
  
  const forceMag = this.strength * Math.min(factor, this.maxMultiplier / r);
  return { ax: -forceMag * dx, ay: -forceMag * dy };  // 指向中心
}
```

### 2.5 力的特性曲线

```
力强度
  │
  │         ╱ ← 边界处力趋向无穷
  │        ╱
  │       ╱
  │      ╱
  │     ╱
  │____╱
  │   ↑
  │   warningR    boundaryR
  └───────────────────────→ 距离
```

- **warningR 内**：无力，天体自由运动
- **warningR 到 boundaryR**：力平滑增长
- **接近 boundaryR**：力急剧增大（指数增长）

### 2.6 阻尼机制

防止天体在边界附近反复弹跳：

```javascript
computeDamping(body, displayScale) {
  if (r < warningR) return { dampingX: 0, dampingY: 0 };
  
  const ratio = (r - warningR) / (boundaryR - warningR);
  const dampingStrength = this.dampingFactor * Math.pow(ratio, 2);
  
  return {
    dampingX: -dampingStrength * body.dx,
    dampingY: -dampingStrength * body.dy
  };
}
```

阻尼力与速度方向相反，大小随距离边界越近而增大。

### 2.7 动量补偿

确保牢笼力不改变系统总动量：

```javascript
computeCageAccelerations() {
  let totalForceX = 0, totalForceY = 0, totalMass = 0;
  
  for (const body of this.bodies) {
    const cageAcc = this.gravityCage.computeAcceleration(body, this.displayScale);
    const damping = this.gravityCage.computeDamping(body, this.displayScale);
    totalForceX += (cageAcc.ax + damping.dampingX) * body.m;
    totalForceY += (cageAcc.ay + damping.dampingY) * body.m;
    totalMass += body.m;
  }
  
  // 补偿加速度，使总力为零
  const compensationAx = -totalForceX / totalMass;
  const compensationAy = -totalForceY / totalMass;
  
  // 应用补偿
  for (const acc of accelerations) {
    acc.ax += compensationAx;
    acc.ay += compensationAy;
  }
}
```

### 2.8 状态检测

```javascript
getBoundaryStatus(body, displayScale) {
  const rNorm = r / boundaryR;
  const wNorm = r / warningR;
  
  if (rNorm >= 1 - this.softening) return "critical";  // 即将逃逸
  if (rNorm >= 0.9) return "danger";                   // 危险
  if (wNorm >= 1) return "warning";                    // 警告
  return "safe";                                       // 安全
}
```

---

## 三、移植清单

### 3.1 引力软化

- [ ] 添加 `useMinDist` 开关
- [ ] 添加 `minDist` 参数
- [ ] 在加速度计算中应用 `Math.max(distSq, minSq)`

### 3.2 引力牢笼

- [ ] 复制 `GravityCage` 类
- [ ] 在主模拟类中添加 `gravityCage` 实例
- [ ] 实现 `computeCageAccelerations()` 方法
- [ ] 在积分步骤中合并牢笼加速度
- [ ] 每步更新牢笼中心到质心位置

### 3.3 积分步骤整合

```javascript
step() {
  // 1. 更新牢笼中心
  if (this.gravityCage.enabled) {
    this.gravityCage.updateCenter(this.computeCenterOfMass());
  }
  
  // 2. Yoshida 积分
  for (const w of YOSHIDA) {
    const h = dt * w;
    
    // 计算加速度 = 引力加速度 + 牢笼加速度
    const cageAccs = this.computeCageAccelerations();
    for (const body of bodies) {
      const acc = this.computeAcc(body, bodies);
      let ax = acc.ax + (cageAccs?.[i].ax || 0);
      let ay = acc.ay + (cageAccs?.[i].ay || 0);
      body.dx += ax * h;
      body.dy += ay * h;
    }
    
    for (const body of bodies) {
      body.x += body.dx * h * displayScale;
      body.y += body.dy * h * displayScale;
    }
  }
}
```

---

## 四、参数调优建议

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `minDist` | 0.5 | 软化距离，根据模拟尺度调整 |
| `boundaryRadius` | 视口宽度的 2-3 倍 | 足够大的活动空间 |
| `warningRadius` | boundaryRadius 的 40-50% | 预留缓冲区 |
| `exponent` | 4 | 力增长速度，越大越"硬" |
| `softening` | 0.001 | 防止数值发散 |
| `maxMultiplier` | 100 | 限制最大力 |
| `dampingFactor` | 0.1 | 阻尼强度 |
