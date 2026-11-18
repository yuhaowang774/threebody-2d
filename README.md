# 三体二维模拟器 · Yoshida 辛积分

基于 **Yoshida 四阶辛积分器**求解二维三体引力运动的实时模拟器。纯 Canvas 2D 渲染，无框架依赖，单页即可运行。

经典周期解（Figure-8）作为默认初值，可实时调整三个天体的质量与速度分量，观察混沌系统在参数扰动下的分岔与逃逸。

---

## 技术特性

- **Yoshida 四阶辛积分**：四段对称权重步（`w1, w1, w0, w1`），长期演化保持能量守恒，避免 RK4 类方法的轨道漂移。
- **引力软化**：限制最小距离平方 `max(r², r_min²)`，消除近距离相遇时的加速度发散，配合「物理真实」开关对比效果。
- **引力牢笼系统**：质心周围的软边界，`warningRadius` 内自由运动、`warningRadius → boundaryRadius` 平滑增长、接近边界时按四次幂急剧增大；带阻尼防反复弹跳 + 全域动量补偿（牢笼力不改变系统总动量）。设计细节见 [`docs/gravity_techniques.md`](docs/gravity_techniques.md)。
- **轨迹渲染**：240 段批量绘制、头尾相接的平滑渐隐拖尾，长度可调（10–5000 步）。
- **光谱型星图背景**：O/B/A/F/G/K/M 七种光谱型恒星贴图按质量分配，叠加 6000×3000 银河全景。
- **物理真实 / 碰撞检测 / 自动缩放**三个可独立开关的观测选项，右侧面板实时调参。

---

## 目录结构

```
├── index.html              # 入口（Vite 工程根）
├── src/
│   ├── main.js             # 模拟核心：Yoshida 积分 + GravityCage + 渲染
│   └── styles.css          # 控制面板与画布样式
├── public/medres/          # 星图与银河背景（构建时原样复制）
├── docs/
│   └── gravity_techniques.md   # 引力软化 / 引力牢笼技术文档
├── vite.config.js          # base: "./"，端口 5215
└── package.json
```

---

## 开发

```bash
npm install
npm run dev      # http://localhost:5215
npm run build    # 产物输出到 dist/
npm run preview  # 预览构建产物
```

## 部署

构建产物挂在 `/tools/threebody-2d/` 子路径下运行。`vite.config.js` 已设 `base: "./"`，`src/main.js` 中的 `medres/*.jpg` 使用相对路径引用，产物放到任意子目录都能直接打开。
