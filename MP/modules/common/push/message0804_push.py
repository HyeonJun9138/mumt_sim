# push/message0804_push.py

from nFusion.Model.msg_0804 import MissionRestartCommand
from generator.message0804_generator import make_msg0804_body
import json




def _dict_to_obj(body: dict) -> MissionRestartCommand:
    cmd = MissionRestartCommand()
    cmd.timestamp      = body["timestamp"]
    cmd.inputMissionID = body["inputMissionID"]
    return cmd

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    """
    쨌 硫붿떆吏瑜?Push ???? GUI 濡쒓렇??洹몃?濡?李띿쓣 ???덈룄濡?      'BODY  ??/ PUSH ?꾨즺' 臾몄옄?댁쓣 UTF-8 諛붿씠?몃줈 諛섑솚?⑸땲??
    """
    msg = _dict_to_obj(body_dict)      # dict ??C# 媛앹껜
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)           # ?꾩넚

    # ?? GUI 濡쒓렇???곗씪 臾몄옄??留뚮뱾湲????????????????????
    log_line = (
        f"[0804] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0804] PUSH ?꾨즺"
    )
    return log_line.encode()

def make_random_and_push(node_messenger) -> bytes:
    return make_and_push(make_msg0804_body(), node_messenger)

