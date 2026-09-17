# Changelog

## 2026-09-16 — 2D 录制页面相机显示最小修复

### 原因

18005 工作站 `/ws/stream` 发送二进制 JPEG，而 18004 录制页面只解析
旧版 `{left: base64, left_detected: boolean}` JSON；解析异常被忽略，导致
包括 `CP0X663000FH` 在内的相机虽正常采集，页面仍无画面。

### 修改

- `calibration_replay/static/app.js`：支持 Blob / ArrayBuffer JPEG，保留旧 JSON 格式。
- 替换帧及关闭连接时释放 object URL；关闭旧连接时清除消息回调。
- `calibration_replay/static/index.html`：移除固定 8131 的过时连接提示，增加脚本版本查询参数以刷新缓存。
- 18005 `/ws/stream` 周期性发送后端真实棋盘格检测结果；页面显示“检测中 / 已检出 / 未检出”三种状态。
- `tests/test_camera_stream.cjs`：覆盖二进制帧、检测状态、旧 JSON、无效 JSON、URL 清理及重新连接。

### 验证及范围

- Node 回归测试和 JavaScript 语法检查。
- 实机只读验证：当前相机 `CP0X663000FH` 正常连接，WebSocket 返回有效 JPEG。
- 未修改相机采集、标定算法、机械臂控制或服务配置；无需重启服务。
- 已打开的页面需要刷新；检测状态约每 0.5 秒更新一次，不会在每帧运行角点算法。
