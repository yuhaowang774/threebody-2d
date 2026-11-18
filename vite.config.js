import { defineConfig } from "vite";

export default defineConfig({
  // 部署在平台的 /tools/<作品标识>/ 子路径下：
  // 用相对路径引用资源，产物放到任意子目录都能直接跑
  base: "./",
  /**
   * 开发服务器端口：本作品固定 **5215**。
   * 端口分配：5204 整站网关 / 5210 solar-system / 5211 earth-system /
   *           5212 weather-lab / 5213 skydome-lab / 5214 landform-lab / **5215 本作品**。
   * 规则（权威文档）：tools-src/earth-system/.codebuddy/rules/dev-server-ports.md
   * strictPort：端口被占直接报错退出，不静默换端口。
   */
  server: {
    port: 5215,
    strictPort: true,
  },
  preview: {
    port: 5215,
    strictPort: true,
  },
});
