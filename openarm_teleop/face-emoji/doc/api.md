

静态IP："192.168.2.199"
端口号：22345


发送JSON示例：
{
    "type": "expression",
    "intent": "sleepy",
    "tts": "你好呀"
}



参数类型：
/** 表情
- type = expression 
- intent 取值对照：
    - neutral：平静
    - happy：开心
    - angry：生气
    - sad：难过
    - surprised：惊讶
    - sleepy：困倦
    - love：喜欢
    - think：思考
    - cool：调皮
    - dizzy：眩晕
**/
  
  
/** 场景
- type = scene 
    - intent 取值对照：
    - approach：有人靠近
    - known：识别熟人
    - listen：开始聆听
    - talk：开始说话
    - wave：挥手打招呼
    - idle：进入待机
    - wake：模拟唤醒
    - multi：多声源
**/