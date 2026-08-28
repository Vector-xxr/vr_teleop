#!/usr/bin/env python3
"""
Complete VR Controller Data Receiver for Meta Quest 3 (Fixed)
适配 Fairino 协作机器人 - 修复 Slerp 和 NoneType 错误
"""

import asyncio
import websockets
import json
import ssl
import os
import sys
import threading
import time
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse
import mimetypes
import numpy as np
# 引入 Slerp 类
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

# 导入机器人接口
fairino_path = os.path.join(os.path.dirname(__file__), 'fairino')
sys.path.insert(0, fairino_path)
import Robot

class HTTPRequestHandler(BaseHTTPRequestHandler):
    """处理静态文件请求 (Client.html)"""
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        try:
            super().end_headers()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ssl.SSLError):
            pass
    
    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()
    
    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self.path = '/client.html'
        
        file_path = os.path.join(os.path.dirname(__file__), self.path.lstrip('/'))
        
        if os.path.exists(file_path) and os.path.isfile(file_path):
            content_type, _ = mimetypes.guess_type(file_path)
            if content_type is None: content_type = 'application/octet-stream'
            try:
                with open(file_path, 'rb') as f:
                    content = f.read()
                self.send_response(200)
                self.send_header('Content-type', content_type)
                self.send_header('Content-length', str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            except Exception as e:
                print(f"File Error: {e}")
        else:
            self.send_error(404, f"File not found: {self.path}")
            
    def log_message(self, format, *args): pass

class RoboticArmController:
    def __init__(self):
        # --- 参数配置 ---
        self.pos_scale = 1.5         # 位移放大倍率 (VR 1m = Robot 1.5m)
        self.smooth_factor = 0.15    # 平滑系数 (0.05=非常平滑/延迟高, 0.5=响应快/抖动)
        self.safe_radius_mm = 850.0  # 最大安全半径 (mm)
        self.min_radius_mm = 200.0   # 最小安全半径 (mm)
        
        # --- 状态变量 ---
        self.is_controlling = False
        self.gripper_state = 0.0     # 0=Open, 1=Closed
        
        # 初始参考点 (按下 Grip 键时的瞬间状态)
        self.hand_ref_pos = None     # VR手柄参考位置 (numpy array)
        self.hand_ref_rot = None     # VR手柄参考姿态 (Rotation object)
        self.robot_ref_pos = None    # 机械臂参考位置 (numpy array mm)
        self.robot_ref_rot = None    # 机械臂参考姿态 (Rotation object)
        
        # 上一帧目标 (用于平滑滤波)
        self.last_target_pos = None
        self.last_target_rot = None

    def get_robot_state(self, robot):
        """读取机械臂当前状态，返回 (Pos_mm, Rotation_Obj)"""
        ret, pose = robot.GetActualTCPPose()
        if ret != 0:
            return None, None
        
        # Fairino 返回 [x, y, z, rx, ry, rz] (mm, degrees)
        pos = np.array(pose[:3])
        # 将欧拉角转为 Scipy Rotation 对象方便计算
        rot = R.from_euler('xyz', pose[3:], degrees=True)
        return pos, rot

    def map_vr_to_robot(self, delta_vr_m):
        """
        坐标系映射核心函数
        VR: Y向上, X向右, Z向后(朝人)
        Robot: Z向上, Y向左/右, X向前
        """
        dx, dy, dz = delta_vr_m
        
        # 映射逻辑
        rob_dx = -dz * 1000  # m转mm (前后)
        rob_dy = -dx * 1000  # (左右)
        rob_dz =  dy * 1000  # (上下)
        
        return np.array([rob_dx, rob_dy, rob_dz])

    def clamp_workspace(self, pos_mm):
        """限制机械臂在安全圆柱体内"""
        x, y, z = pos_mm
        
        # 1. 高度限制
        z = max(50.0, min(z, 800.0))
        
        # 2. 半径限制
        dist_sq = x**2 + y**2 + z**2
        dist = np.sqrt(dist_sq)
        
        if dist < self.min_radius_mm:
            scale = self.min_radius_mm / dist
            x *= scale; y *= scale; z *= scale
        elif dist > self.safe_radius_mm:
            scale = self.safe_radius_mm / dist
            x *= scale; y *= scale; z *= scale
            
        return np.array([x, y, z])

    def process_hand_data(self, robot, hand_data):
        """处理手柄数据的主循环"""
        try:
            # 1. 解析 VR 数据
            curr_hand_pos = np.array(hand_data["position"])
            curr_hand_rot = R.from_quat(hand_data["orientation"]) 
            
            keys = hand_data["keys"]
            grip_active = (keys["grip_button"] == 1)
            trigger_val = keys.get("index_trigger", 0.0)
            
            # --- 状态机管理 ---
            
            # A. 刚按下 Grip 键：初始化参考点
            if grip_active and not self.is_controlling:
                print("🟢 激活控制：记录参考点")
                rob_pos, rob_rot = self.get_robot_state(robot)
                if rob_pos is not None:
                    self.hand_ref_pos = curr_hand_pos
                    self.hand_ref_rot = curr_hand_rot
                    self.robot_ref_pos = rob_pos
                    self.robot_ref_rot = rob_rot
                    
                    # 初始化滤波器
                    self.last_target_pos = rob_pos
                    self.last_target_rot = rob_rot
                    
                    self.is_controlling = True
                else:
                    print("❌ 无法获取机械臂状态，无法激活")
                    return

            # B. 松开 Grip 键：停止控制
            if not grip_active:
                if self.is_controlling:
                    print("🔴 停止控制")
                    self.is_controlling = False
                return

            # C. 保持按下：执行控制
            if self.is_controlling:
                # --- 1. 计算位移 ---
                delta_hand = (curr_hand_pos - self.hand_ref_pos) * self.pos_scale
                delta_robot_mm = self.map_vr_to_robot(delta_hand)
                
                target_pos = self.robot_ref_pos + delta_robot_mm
                target_pos = self.clamp_workspace(target_pos)
                
                # --- 2. 计算旋转 ---
                # 相对旋转：R_target = R_robot_ref * (R_hand_ref^-1 * R_hand_curr)
                delta_rot = self.hand_ref_rot.inv() * curr_hand_rot
                target_rot = self.robot_ref_rot * delta_rot
                
                # --- 3. 平滑滤波 (修复了 Slerp 报错) ---
                
                # 位置: 简单的线性插值 (Lerp)
                smooth_pos = self.last_target_pos * (1 - self.smooth_factor) + target_pos * self.smooth_factor
                
                # 姿态: 使用 Scipy Slerp
                # 构建包含两个旋转的关键帧对象: [上一帧姿态, 当前目标姿态]
                key_rots = R.from_quat([self.last_target_rot.as_quat(), target_rot.as_quat()])
                key_times = [0, 1]
                
                # 创建插值器
                slerp_obj = Slerp(key_times, key_rots)
                
                # 取 smooth_factor 处的值 (返回的是 Rotation 对象)
                smooth_rot = slerp_obj([self.smooth_factor])[0]
                
                # 更新历史
                self.last_target_pos = smooth_pos
                self.last_target_rot = smooth_rot
                
                # --- 4. 发送指令 ---
                self.move_robot(robot, smooth_pos, smooth_rot)
                
                # --- 5. 夹爪控制 ---
                if abs(trigger_val - self.gripper_state) > 0.05:
                    self.gripper_state = trigger_val
                    # 假设 Trigger按下(1.0) = 闭合 = Robot 100
                    gripper_target = int(trigger_val * 100)
                    # robot.MoveGripper(0, gripper_target, 50, 50, 500, 0, 0, 0, 0, 0)

        except Exception as e:
            print(f"Logic Error: {e}")
            import traceback
            traceback.print_exc()

    def move_robot(self, robot, pos, rot):
        """执行运动指令"""
        try:
            # 1. 转换格式
            euler = rot.as_euler('xyz', degrees=True)
            target_pose = [pos[0], pos[1], pos[2], euler[0], euler[1], euler[2]]
            
            # 2. 获取当前关节角作为 IK 参考
            ret, current_joints = robot.GetActualJointPosDegree()
            if ret != 0: return

            # 3. 逆运动学求解
            # 0 = 绝对位姿求解 (基于参考关节角)
            ret, target_joints = robot.GetInverseKinRef(0, target_pose, current_joints)
            
            print("----target_joints", target_joints)

            if ret == 0:
                # 4. === 关键修改 ===
                # 使用 ServoJ 进行实时跟随

                """   
                @param  [in] 必选参数 joint_pos: 目标关节位置，单位 [°]
                @param  [in] 必选参数 axisPos  外部轴位置,单位mm
                @param  [in] 默认参数 acc: 加速度，范围 [0~100]，暂不开放，默认为 0.0
                @param  [in] 默认参数 vel: 速度，范围 [0~100]，暂不开放，默认为 0.0
                
                @param  [in] 默认参数 cmdT: 指令下发周期，单位s，建议范围[0.001~0.0016], 默认为0.008
                @param  [in] 默认参数 filterT: 滤波时间，单位 [s]，暂不开放， 默认为0.0

                @param  [in] 默认参数 gain: 目标位置的比例放大器，暂不开放， 默认为0.0
                @param  [in] 默认参数 id: servoJ指令ID,默认为0
                """
                
                error = robot.ServoJ(target_joints, [0.0, 0.0], 0.05, 0.05,   0.05, 0.2,   0.0, 0)
                
                # 调试日志
                if int(time.time() * 20) % 20 == 0:
                    print(f"🚀 ServoJ: {int(pos[0])}, {int(pos[1])}, {int(pos[2])} | Ret: {error}")
            
            elif ret == 112:
                # IK 无解，忽略
                print(f"❌ Robot IK 无解，忽略 : {ret}")
                pass 
            else:
                print(f"❌ Robot Error: {ret}")
                
        except Exception as e:
            print(f"Move Error: {e}")

class QuestControllerReceiver:
    def __init__(self, host='0.0.0.0', http_port=8443, ws_port=8442):
        self.host = host
        self.http_port = http_port
        self.ws_port = ws_port
        self.clients = set()
        self.ssl_context = None
        self.http_server = None
        
        self.arm_controller = RoboticArmController()
        
        # 频率控制
        self.last_cmd_time = 0
        self.cmd_interval = 0.04  # 25Hz (40ms)
        
        # 连接机器人
        print("Connecting to Robot...")
        try:
            self.robot = Robot.RPC('192.168.58.2')
            print("✅ Robot Connected!")
        except Exception as e:
            print(f"❌ Robot Connection Failed: {e}")
            self.robot = None

    def create_self_signed_cert(self):
        """生成自签名证书"""
        import subprocess, tempfile, socket
        cert_path = "quest_cert.pem"
        key_path = "quest_key.pem"
        
        if not os.path.exists(cert_path):
            print("Generating SSL Certificate...")
            hostname = socket.gethostname()
            ips = ["127.0.0.1", "0.0.0.0"]
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))
                ips.append(s.getsockname()[0])
                s.close()
            except: pass
            
            alt_names = "\n".join([f"IP.{i+1} = {ip}" for i, ip in enumerate(ips)])
            config = f"""
[req]
distinguished_name = req_distinguished_name
x509_extensions = v3_req
prompt = no
[req_distinguished_name]
CN = {hostname}
[v3_req]
basicConstraints = CA:FALSE
subjectAltName = @alt_names
[alt_names]
{alt_names}
"""
            try:
                subprocess.run(['openssl', 'genrsa', '-out', key_path, '2048'], check=True, stderr=subprocess.DEVNULL)
                with tempfile.NamedTemporaryFile(mode='w') as f:
                    f.write(config)
                    f.flush()
                    subprocess.run(['openssl', 'req', '-new', '-x509', '-key', key_path, 
                                  '-out', cert_path, '-days', '365', '-config', f.name], 
                                  check=True, stderr=subprocess.DEVNULL)
            except Exception as e:
                print(f"Cert Gen Error: {e}")
                return None

        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        try:
            context.load_cert_chain(cert_path, key_path)
            return context
        except: return None

    async def handle_client(self, websocket):
        """处理 WebSocket 连接"""
        print(f"Client Connected: {websocket.remote_address}")
        self.clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self.process_data(data)
                except json.JSONDecodeError: pass
        except Exception as e:
            print(f"WebSocket Error: {e}")
        finally:
            self.clients.discard(websocket)

    def process_data(self, data):
        """解析并分发数据"""
        now = time.time()
        if now - self.last_cmd_time < self.cmd_interval:
            return
        self.last_cmd_time = now

        # 修复 NoneType 错误：增加 get() 检查
        if self.robot and data.get('rightController'):
            right = data['rightController']
            # 双重检查，防止 right 为 None
            if not right: return

            # 提取位置
            p = right.get('position', {})
            pos = [p.get('x', 0), p.get('y', 0), p.get('z', 0)]
            
            # 提取姿态
            q = right.get('quaternion') 
            if not q:
                r_euler = right.get('rotation', {})
                temp_rot = R.from_euler('xyz', [r_euler.get('x',0), r_euler.get('y',0), r_euler.get('z',0)], degrees=True)
                orientation = temp_rot.as_quat().tolist()
            else:
                orientation = [q.get('x',0), q.get('y',0), q.get('z',0), q.get('w',1)]

            # 提取按键
            keys = {
                "grip_button": 1 if right.get('gripActive', False) else 0,
                "index_trigger": right.get('trigger', 0),
                "menu_button": 1 if right.get('menuButton', False) else 0
            }
            
            hand_data = {
                "position": pos,
                "orientation": orientation,
                "keys": keys
            }
            
            self.arm_controller.process_hand_data(self.robot, hand_data)

    def cleanup(self):
        if self.robot: self.robot = None

    def start_http_server(self):
        self.ssl_context = self.create_self_signed_cert()
        if not self.ssl_context: return False
        
        self.http_server = HTTPServer((self.host, self.http_port), HTTPRequestHandler)
        self.http_server.socket = self.ssl_context.wrap_socket(self.http_server.socket, server_side=True)
        
        t = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        t.start()
        print(f"HTTPS Server running at https://{self.host}:{self.http_port}")
        return True

    async def start_websocket_server(self):
        async with websockets.serve(self.handle_client, self.host, self.ws_port, ssl=self.ssl_context):
            print(f"WebSocket Server running at wss://{self.host}:{self.ws_port}")
            await asyncio.Future()

async def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    args = parser.parse_args()
    
    receiver = QuestControllerReceiver(host=args.host)
    if receiver.start_http_server():
        try:
            await receiver.start_websocket_server()
        except KeyboardInterrupt:
            pass
        finally:
            receiver.cleanup()

if __name__ == "__main__":
    asyncio.run(main())