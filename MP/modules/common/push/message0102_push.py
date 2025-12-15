# modules/common/push/message0102_push.py
# auto-fixed at 2025-09-19

import json, importlib
from datetime import datetime, timezone
from System.Collections.Generic import List
from nFusion.Model.msg_0102 import *    # C# 紐⑤뜽
from nFusion.Model.CommonType import *  # 怨듯넻 ???from System import String, UInt32, UInt64

MSG_ID = "0102"
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

def _try_set(obj, name: str, value) -> bool:
    for k in (name, name[:1].upper()+name[1:] if name else name):
        try:
            if hasattr(obj, k):
                setattr(obj, k, value)
                return True
        except Exception:
            pass
    return False

def _cs(name: str):
    t = globals().get(name)
    if t is not None: return t
    for modname in (f'nFusion.Model.msg_{MSG_ID}', 'nFusion.Model.CommonType', 'nFusion.Model'):
        try:
            mod = importlib.import_module(modname)
            t = getattr(mod, name, None)
            if t is not None: return t
        except Exception:
            pass
    return None

def _new(name: str):
    t = _cs(name)
    if t is None:
        raise NameError(f'type not found: {name}')
    return t()

def _normalize_body_keys(body: dict) -> dict:
    """
    ?ㅼ뼱??諛붾뵒瑜?0102 ?쒖? ?ㅻ줈 ?뺢퇋??
      Timestamp/timestamp -> timestamp
      Status/status       -> status
      Source/RequestModuleName/... -> source
    (媛믪? 媛怨듯븯吏 ?딆쓬)
    """
    if not isinstance(body, dict):
        return {}

    out = {}
    # timestamp
    for k in ("timestamp","Timestamp","timeStamp","TimeStamp","ts","TS"):
        if k in body:
            try: out["timestamp"] = int(body[k]); break
            except Exception: pass

    # status
    for k in ("status","Status","moduleStatus","ModuleStatus","state","State","healthStatus","HealthStatus"):
        if k in body:
            try: out["status"] = int(body[k]); break
            except Exception: pass

    # source (SourceModuleName ?먭린 ??source濡??듭씪)
    for k in ("source","Source","requestModuleName","RequestModuleName","module","Module","sourceModule","SourceModule"):
        if k in body:
            v = str(body[k])
            if v:
                out["source"] = v
                break

    return out

def _dict_to_ModuleStatus(data: dict):
    obj = _new('ModuleStatus')
    # ?쒖? ??湲곗??쇰줈 ?명똿 (?꾨뱶紐???뚮Ц??紐⑤몢 ?쒕룄)
    if "timestamp" in data: _try_set(obj, "timestamp", int(data["timestamp"]))
    if "status"    in data: _try_set(obj, "status",    int(data["status"]))
    if "source"    in data:
        if not _try_set(obj, "source", str(data["source"])):
            _try_set(obj, "Source", str(data["source"]))
    return obj

def _dict_to_obj(body_dict: dict):
    return _dict_to_ModuleStatus(body_dict)

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    # 0102???붿씠?몃━?ㅽ듃 ?놁쓬 ??癒쇱? ?쒖? ?ㅻ줈 ?뺢퇋??    body_dict = _normalize_body_keys(body_dict)
    # 理쒖냼 ???ㅺ? ?섎룄濡??놁뼱??洹몃?濡?蹂대깂: 媛怨??앹꽦?섏? ?딆쓬)
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    # 濡쒓렇?먮룄 ?쒖? ?ㅻ줈 ?숈씪?섍쾶 異쒕젰
    log_line = (
        f"[0102] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0102] PUSH ?꾨즺"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    """
    ?쒕꼫?덉씠?곌? 鍮꾧굅???꾨씫?대룄 ??踰????뺢퇋?뷀븯??    BODY??timestamp/status/source 3?ㅺ? ?쒖? ?ㅻ줈 李랁엳寃??쒕떎.
    """
    try:
        from generator.message0102_generator import make_msg0102_body
        raw = make_msg0102_body()
    except Exception:
        raw = {}

    # ?대갚: 理쒖냼 ????(媛??앹꽦? ?섏? ?딆?留? ?놁쑝硫?異붽? ?쒕룄)
    if not isinstance(raw, dict):
        raw = {}
    if "timestamp" not in raw and "Timestamp" not in raw:
        raw["timestamp"] = _now_ms()
    if "status" not in raw and "Status" not in raw:
        raw["status"] = 1
    if all(k not in raw for k in ("source","Source","requestModuleName","RequestModuleName")):
        raw["source"] = "DSC"

    return make_and_push(raw, node_messenger)

