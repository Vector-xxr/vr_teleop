
问一下 meta quest3从头上摘下来就熄屏，影响遥操作，你这个问题是怎么解决的  
可以在 quest3 里面设置常亮

看不见背景如何处理
设置背景透明


---
## 如何设置 quest3 显示机器人摄像头

pip install opencv-python websockets asyncio


### python脚本读取摄像头发给quest3
```python
import asyncio
import websockets
import json
import cv2
import base64
import time

# --- 配置 ---
PORT = 8442
CAMERA_ID = 0  # 如果有多个摄像头，尝试改 1 或 2
IMAGE_WIDTH = 640  # 降低分辨率以减少延迟 (640x480 或 320x240)
JPEG_QUALITY = 70  # 图片质量 (0-100)，越低越快

# 全局变量
connected_clients = set()

async def camera_stream_task():
    """专门负责读取摄像头并广播的协程"""
    cap = cv2.VideoCapture(CAMERA_ID)
    
    if not cap.isOpened():
        print("❌ 无法打开摄像头，请检查连接")
        return

    print(f"📷 摄像头已启动 (Res: {IMAGE_WIDTH}px width)")

    try:
        while True:
            if not connected_clients:
                # 如果没有客户端连接，休眠以节省资源
                await asyncio.sleep(0.5)
                continue

            ret, frame = cap.read()
            if not ret:
                print("无法获取画面")
                await asyncio.sleep(0.1)
                continue

            # 1. 调整大小 (降低带宽需求)
            height, width = frame.shape[:2]
            new_height = int(height * (IMAGE_WIDTH / width))
            frame = cv2.resize(frame, (IMAGE_WIDTH, new_height))

            # 2. 压缩为 JPEG
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            
            # 3. 转为 Base64
            img_str = base64.b64encode(buffer).decode('utf-8')
            
            # 4. 构建消息
            message = json.dumps({
                "type": "video",
                "data": img_str
            })

            # 5. 广播给所有连接的客户端 (Quest 3)
            # 使用 list() 创建副本以防在迭代时集合发生变化
            for ws in list(connected_clients):
                try:
                    await ws.send(message)
                except websockets.exceptions.ConnectionClosed:
                    pass
            
            # 控制帧率，约 30 FPS
            await asyncio.sleep(0.033)

    except Exception as e:
        print(f"Camera Error: {e}")
    finally:
        cap.release()

async def handler(websocket):
    """处理 WebSocket 连接"""
    print(f"✅ 新客户端连接: {websocket.remote_address}")
    connected_clients.add(websocket)
    
    try:
        async for message in websocket:
            # 这里接收 Quest 3 发来的手柄数据
            data = json.loads(message)
            
            # 简单打印手柄数据 (可以替换为控制机器人的代码)
            if "leftController" in data and data["leftController"]:
                l_pos = data["leftController"]["position"]
                print(f"\r📥 收到手柄数据 - Left X: {l_pos['x']:.2f}", end="")
            
    except websockets.exceptions.ConnectionClosed:
        print(f"❌ 客户端断开")
    finally:
        connected_clients.remove(websocket)

async def main():
    # 启动 WebSocket 服务器
    server = await websockets.serve(handler, "0.0.0.0", PORT)
    print(f"🚀 服务器运行在 ws://localhost:{PORT}")
    
    # 并行运行：WebSocket服务 + 摄像头推流
    await asyncio.gather(
        server.wait_closed(),
        camera_stream_task()
    )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("程序已停止")
```





### 更新 HTML 客户端

```
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Quest 3 Robot Vision Client</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0; padding: 20px;
            background-color: #f0f0f0;
        }
        body.ar-active {
            background-color: rgba(0, 0, 0, 0) !important;
        }
        .container {
            max-width: 900px;
            margin: 0 auto;
            background-color: white;
            padding: 20px;
            border-radius: 10px;
            text-align: center;
        }
        
        /* 视频窗口样式 */
        .video-container {
            margin: 20px auto;
            width: 640px;
            max-width: 100%;
            border: 3px solid #333;
            border-radius: 8px;
            background: #000;
            position: relative;
            z-index: 1000; /* 确保在最上层 */
        }
        
        #robotView {
            width: 100%;
            display: block;
            /* 初始占位高度，避免布局跳动 */
            min-height: 300px; 
        }

        .data-panel {
            display: inline-block; width: 40%; 
            padding: 10px; border: 1px solid #ccc; 
            margin: 10px; background: rgba(255,255,255,0.9);
            text-align: left;
        }
        
        .status-badge {
            display: inline-block; padding: 5px 10px;
            border-radius: 15px; background: #eee;
            font-weight: bold; font-size: 14px;
        }
        .connected { background: #d4edda; color: #155724; }
        .disconnected { background: #f8d7da; color: #721c24; }

        button {
            padding: 15px 30px; font-size: 18px;
            background: #007bff; color: white;
            border: none; border-radius: 8px;
            margin-top: 20px;
        }
    </style>
</head>
<body>

    <div class="container">
        <h1>🤖 Robot FPV Control</h1>
        
        <div id="connectionStatus" class="status-badge disconnected">Disconnected</div>
        <div id="xrStatus" class="status-badge" style="background:#e2e3e5">Checking WebXR...</div>

        <!-- 视频显示区域 -->
        <div class="video-container">
            <img id="robotView" src="" alt="Waiting for camera stream..." />
        </div>

        <!-- 手柄数据显示区域 -->
        <div id="controllersData">
            <div class="data-panel" id="leftData">Wait for Left...</div>
            <div class="data-panel" id="rightData">Wait for Right...</div>
        </div>

        <button id="startBtn" onclick="startTracking()" disabled>Start AR & Camera</button>
    </div>

    <script>
        const serverIp = window.location.hostname || 'localhost';
        const wsUrl = window.location.protocol.startsWith('https') ? `wss://${serverIp}:8442` : `ws://${serverIp}:8442`;
        
        let websocket = null;
        let xrSession = null;
        let xrRefSpace = null;
        const videoImg = document.getElementById('robotView');

        // 初始化
        window.onload = checkXR;

        function checkXR() {
            if ('xr' in navigator) {
                navigator.xr.isSessionSupported('immersive-ar').then(supported => {
                    const status = document.getElementById('xrStatus');
                    if(supported) {
                        status.textContent = "✅ AR Ready";
                        status.style.background = "#d4edda";
                        document.getElementById('startBtn').disabled = false;
                    } else {
                        status.textContent = "❌ AR Not Supported";
                    }
                });
            }
        }

        function connectWS() {
            websocket = new WebSocket(wsUrl);
            websocket.onopen = () => {
                document.getElementById('connectionStatus').className = 'status-badge connected';
                document.getElementById('connectionStatus').textContent = 'Connected';
            };
            websocket.onclose = () => {
                document.getElementById('connectionStatus').className = 'status-badge disconnected';
                document.getElementById('connectionStatus').textContent = 'Disconnected';
            };
            
            // --- 关键：处理收到的视频数据 ---
            websocket.onmessage = (event) => {
                try {
                    const msg = JSON.parse(event.data);
                    // 如果消息类型是视频
                    if (msg.type === 'video') {
                        videoImg.src = "data:image/jpeg;base64," + msg.data;
                    }
                } catch (e) {
                    console.error("Parse error", e);
                }
            };
        }

        async function startTracking() {
            connectWS();
            
            if (!navigator.xr) return;

            const session = await navigator.xr.requestSession('immersive-ar', {
                requiredFeatures: ['local-floor'],
                optionalFeatures: ['dom-overlay'],
                domOverlay: { root: document.body } // 让网页悬浮在 AR 中
            });
            
            xrSession = session;
            document.body.classList.add('ar-active');
            document.getElementById('startBtn').style.display = 'none'; // 隐藏按钮

            // WebGL Setup (用于透明背景)
            const canvas = document.createElement('canvas');
            const gl = canvas.getContext('webgl2', { alpha: true, xrCompatible: true });
            session.updateRenderState({ baseLayer: new XRWebGLLayer(session, gl) });

            // 获取参考空间
            xrRefSpace = await session.requestReferenceSpace('local-floor');

            // 循环
            session.requestAnimationFrame(onXRFrame);

            function onXRFrame(time, frame) {
                const session = frame.session;
                
                // 1. 清除画布 (透明)
                gl.bindFramebuffer(gl.FRAMEBUFFER, session.renderState.baseLayer.framebuffer);
                gl.clearColor(0, 0, 0, 0);
                gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

                // 2. 获取手柄数据
                const pose = frame.getViewerPose(xrRefSpace);
                if (pose) {
                    processControllers(session, frame);
                }
                
                session.requestAnimationFrame(onXRFrame);
            }
        }

        function processControllers(session, frame) {
            const sources = session.inputSources;
            let payload = { leftController: null, rightController: null };
            let hasData = false;

            for (let source of sources) {
                const gripPose = frame.getPose(source.gripSpace, xrRefSpace);
                if (gripPose) {
                    const p = gripPose.transform.position;
                    const r = gripPose.transform.orientation;
                    
                    // 获取扳机键
                    let trigger = 0;
                    if (source.gamepad && source.gamepad.buttons.length > 0) {
                        trigger = source.gamepad.buttons[0].pressed ? 1 : 0;
                    }

                    const data = {
                        position: {x: p.x, y: p.y, z: p.z},
                        rotation: {x: r.x, y: r.y, z: r.z, w: r.w}, // 发送四元数由Python处理更方便
                        trigger: trigger
                    };

                    // 更新 UI
                    document.getElementById(source.handedness + 'Data').innerHTML = 
                        `<b>${source.handedness.toUpperCase()}</b><br>
                         X: ${p.x.toFixed(2)} Y: ${p.y.toFixed(2)} Z: ${p.z.toFixed(2)}<br>
                         Trigger: ${trigger}`;

                    if (source.handedness === 'left') payload.leftController = data;
                    if (source.handedness === 'right') payload.rightController = data;
                    hasData = true;
                }
            }

            // 发送数据回 Python
            if (hasData && websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send(JSON.stringify(payload));
            }
        }
    </script>
</body>
</html>

```








---




# Meta Quest 3 Controller Data Receiver

This standalone Python application serves both the client webpage and receives controller pose and button data from Meta Quest 3, printing it to the console.

## Features

- HTTP server to serve the client webpage
- WebSocket server to receive controller data
- Prints position, rotation, grip, and trigger states
- No SSL certificates required for basic operation

## Requirements

- Python 3.7+
- websockets library

## Installation

```bash
pip install websockets
```

## Usage

### HTTP Version (Recommended for development):

1. Run the server:
```bash
python simple_server.py
# Or with specific host/port:
python simple_server.py --host 0.0.0.0 --http-port 8080 --ws-port 8081


python server.py --host 192.168.2.186 --http-port 8443 --ws-port 8442

python server.py --host  0.0.0.0 --http-port 8443 --ws-port 8442

IP代理有影响，直接连接一个没有设置代理的就成功了

```


1. Access the client page from your device:
   - On your computer, find your IP address with `ipconfig` (on Windows) or `ifconfig` (on Mac/Linux)
   - Example: if your IP is 192.168.1.100, the client URL will be `http://192.168.1.100:8080/client.html`
   - On your device, open the browser and navigate to this URL

2. Click "Start Controller Tracking" to begin
3. Controller data will be printed to the console in real-time.

### HTTPS Version (Required for some browsers):

If the HTTP version doesn't work for accessing WebXR features, you can use the original HTTPS version:

1. Run the server:
```bash
python server.py
```

2. Access the client page from your Meta Quest 3:
   - On your Meta Quest 3, open the browser and navigate to `https://[your-server-ip]:8443/client.html`
   - Accept any security warnings about self-signed certificates

## Important Notes

- For WebXR features (VR/AR), some browsers require HTTPS. If you experience issues with controller detection, try the HTTPS version.
- The HTTP version is easier to set up but may not work on all browsers/devices for WebXR features.
- Make sure both devices are on the same network.

## WebXR Browser Compatibility

WebXR support varies by browser. For Meta Quest 3, you should use one of these browsers:

1. **Oculus Browser** (Recommended)
   - Built-in browser for Quest devices
   - Full WebXR support
   - May require enabling experimental features

2. **Firefox Reality**
   - Available on Quest store
   - Good WebXR support

3. **Chrome** (on PC for testing)
   - Enable experimental WebXR flag: chrome://flags/#enable-webxr-incubations

### If WebXR is not supported:

- Make sure your browser supports WebXR
- On some browsers, you may need to enable experimental WebXR features
- Try accessing the page via HTTPS instead of HTTP
- Check that you're using a compatible browser

## Troubleshooting Connection Issues

### Common Problems and Solutions:

1. **"Connection unexpectedly terminated"**
   - With HTTPS version: This often occurs due to SSL/TLS handshake issues. Make sure you allow the self-signed certificate in the browser.

2. **WebXR not supported**
   - Make sure you're using a compatible browser (Oculus Browser, Firefox Reality)
   - Some browsers require HTTPS for WebXR access
   - Check that WebXR features are enabled in browser settings

3. **Firewall blocking ports**
   - Make sure ports 8080/8081 (HTTP version) or 8443/8442 (HTTPS version) are open
   - Temporarily disable firewall to test

4. **Network connectivity**
   - Ensure both devices are on the same WiFi network
   - Verify the IP address is correct (try pinging the server from the client device)

### Additional Debugging Steps:

1. Test if the server is accessible:
   - From another device on the same network, try accessing `http://[your-server-ip]:8080/client.html` (for HTTP version)
   
2. Verify ports are available:
   - Use `netstat -an | findstr :8080` (Windows) to check if port is in use

## Alternative Setup

If you continue having issues, you can try using a service like ngrok to expose your local server publicly:

1. Download and install ngrok from https://ngrok.com/
2. Start your server normally: `python simple_server.py`
3. In another terminal: `ngrok http http://localhost:8080`
4. Use the ngrok URL to access the client page