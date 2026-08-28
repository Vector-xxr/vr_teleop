# Left Job Socket API

- **地址**：`<机器人IP>:47925`（例：`192.168.2.100:47925`）
- **传输**：明文 TCP（TLS 默认关；需要时设 `OPENARM_LEFT_JOB_TLS=1`）
- **鉴权**：带固定 `token`（默认 `openarm-job-api`）；`id` / `ts` 可选
- **`sig`**：暂不需要

---

## 通用请求

```json
{
  "v": 1,
  "cmd": "status",
  "id": "a1b2c3d4e5f6",
  "ts": 1710000000.123,
  "token": "openarm-job-api"
}
```

---

## status

```json
{"v":1,"cmd":"status","id":"…","ts":1710000000.123,"token":"openarm-job-api"}
```

应答：

```json
{"v":1,"ok":true,"cmd":"status","id":"…","error":null,"busy":0,"job":null,"startup_done":1,"last":"idle","jobs":["1"],"subs":0}
```

---

## run（执行 job）

```json
{"v":1,"cmd":"run","job":"1","id":"…","ts":1710000000.123,"token":"openarm-job-api"}
```

应答：

```json
{"v":1,"ok":true,"cmd":"run","id":"…","error":null,"accepted":true,"job":"1","mode":"trajectory_only","from":"192.168.2.50:54321"}
```

忙碌：

```json
{"v":1,"ok":false,"cmd":"run","id":"…","error":"busy","job":"1"}
```

---

## subscribe / unsubscribe

```json
{"v":1,"cmd":"subscribe","id":"…","ts":1710000000.123,"token":"openarm-job-api"}
```

```json
{"v":1,"cmd":"unsubscribe","id":"…","ts":1710000000.123,"token":"openarm-job-api"}
```

订阅推送：

```json
{"v":1,"ok":true,"cmd":"status","event":"status","error":null,"busy":1,"job":"1","startup_done":1,"last":"running","jobs":["1"],"subs":1}
```

---

## abort / shutdown

```json
{"v":1,"cmd":"abort","id":"…","ts":1710000000.123,"token":"openarm-job-api"}
```

```json
{"v":1,"cmd":"shutdown","id":"…","ts":1710000000.123,"token":"openarm-job-api"}
```

---

## 测试

```bash
$PY -m left_job_orchestrator.test_socket --host 192.168.2.100 status
$PY -m left_job_orchestrator.test_socket --host 192.168.2.100 run --job 1
```
