# LiDAR-Flare-Notifier-CTRL_script

跳伞着陆动作提醒装置的伴随目录：需求、主机侧工具、测试数据与临时产物；交付物在 `.../LiDAR-Flare-Notifier-CTRL`。

## 1. 目录结构

```text
LiDAR-Flare-Notifier-CTRL_script/
|-- AGENTS.md
|-- PLAN.md        需求、设计决策与边界
|-- scripts/       主机侧长期工具
|-- data/          测试数据与记录
`-- temp/          一次性脚本与临时产物
```

## 2. 工作区规范

### 2.1 角色

- 你的职责是，且仅限于，为本项目开发固件。

### 2.2 目录职责

- 各子目录规则见其 `AGENTS.md`；子目录中的操作规范以其 `AGENTS.md` 为准。
- 一次性脚本和临时数据输出放 `temp/`，长期工具放 `scripts/`，长期数据放 `data/`。

## 2.3. 外部目录

- 配对仓库：`../LiDAR-Flare-Notifier-CTRL`（git 子模块）。
- 改子模块前先查 `git -C ../LiDAR-Flare-Notifier-CTRL status`。
- 子模块内只包括交付物，不添加本地工具、说明或临时文件。
- 应用逻辑只放 `App/`；`main.c` 只在 `USER CODE` 区内改。
- 构建产物与中间文件只写 `%TEMP%` 或本目录 `temp/`。

## 2.4. 行为约束

- 一次性脚本放 `temp/`，长期工具放 `scripts/`，长期数据放 `data/`。
- 进攻性地编程；假定硬件可靠，模块按预期工作，设计契约均有其理由。
- 为潜在异常/风险/失败模式加入保护措施时，询问自己：
  1. 它能否在实际生产环境中发生？
  2. 它是否存在明确的触发条件和现实发生路径？
  3. 增加该措施是否能实质性降低风险？
  当且仅当得到三重肯定时，才开始评估成本和副作用，并决定其优先级。
