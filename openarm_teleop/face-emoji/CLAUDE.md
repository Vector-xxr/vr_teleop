# face-emoji

机器人表情 / 场景 socket 通讯客户端。通过 TCP socket 向机器人发送 JSON 控制指令，控制其面部表情与场景行为。

## 项目结构

```
face-emoji/
├── app/
│   └── face.py        # socket 客户端实现 (纯标准库)
├── doc/
│   └── api.md         # 通讯协议文档
└── CLAUDE.md          # 本文件
```

## 通讯协议 (doc/api.md)

- **服务器**：`192.168.2.199:22345` (TCP)
- **报文格式**：JSON，发送时以 `\n` 结尾，便于服务端按行解析

发送示例：

```json
{"type": "expression", "intent": "sleepy", "tts": "你好呀"}
```

### 参数取值

**表情** `type = expression`

| intent    | 含义   |
| --------- | ------ |
| neutral   | 平静   |
| happy     | 开心   |
| angry     | 生气   |
| sad       | 难过   |
| surprised | 惊讶   |
| sleepy    | 困倦   |
| love      | 喜欢   |
| think     | 思考   |
| cool      | 调皮   |
| dizzy     | 眩晕   |

**场景** `type = scene`

| intent   | 含义       |
| -------- | ---------- |
| approach | 有人靠近   |
| known    | 识别熟人   |
| listen   | 开始聆听   |
| talk     | 开始说话   |
| wave     | 挥手打招呼 |
| idle     | 进入待机   |
| wake     | 模拟唤醒   |
| multi    | 多声源     |

`tts` 字段为可选的语音播报文本。

## 运行环境

- Windows + miniconda，虚拟环境 **my310**
- 仅依赖 Python 标准库（`socket`、`json`），无需额外安装

## 使用方法

由于输出含中文，Windows 下建议先激活环境并设置 UTF-8：

```powershell
conda activate my310
$env:PYTHONIOENCODING="utf-8"

# 发送表情
python app/face.py --intent happy --tts "你好呀"

# 发送场景
python app/face.py --type scene --intent wave

# 指定服务器地址 / 端口
python app/face.py --host 192.168.2.199 --port 22345 --intent love

# 查看所有可用 intent 取值
python app/face.py --list

# 交互模式 (循环发送: <type> <intent> [tts])
python app/face.py -i
```

### 命令行参数

| 参数                | 说明                                 | 默认值          |
| ------------------- | ------------------------------------ | --------------- |
| `--host`            | 服务器 IP                            | `192.168.2.199` |
| `--port`            | 端口                                 | `22345`         |
| `--type`            | 报文类型 `expression` / `scene`      | `expression`    |
| `--intent`          | 意图取值 (见上表)                    | 必填            |
| `--tts`             | 语音播报文本 (可选)                  | 无              |
| `--timeout`         | 连接 / 读取超时秒数                  | `5.0`           |
| `-i, --interactive` | 进入交互模式                         | -               |
| `-l, --list`        | 列出所有 intent 取值                 | -               |

## 作为模块调用

```python
from app.face import send_once, FaceClient

# 一次性发送
send_once("192.168.2.199", 22345, "expression", "happy", tts="你好呀")

# 复用连接发送多条
with FaceClient() as c:
    c.send({"type": "expression", "intent": "love"})
    c.send({"type": "scene", "intent": "wave"})
```

## 注意事项

- **不要用 `conda run -n my310 python ...`**：conda 在 Windows 上会用 GBK 重新编码中文输出，导致 `UnicodeEncodeError`。请改用 `conda activate my310` 后再运行，或直接调用环境内的 `python.exe`。
- 连接报 `[WinError 10061] 目标计算机积极拒绝` 表示机器人服务端未开启或不在同一网络，属正常的连接失败提示，代码逻辑无误。
