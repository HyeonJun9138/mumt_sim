# ?뚯씪: modules/common/push/message0305_push.py
# -*- coding: utf-8 -*-
# 0305 ?ш퀎???섑뻾?곹깭 ?뺣낫 PUSH (?덉젙?? ????먯깋 踰붿쐞/?앹꽦 ?덉젙??蹂닿컯)

import json, importlib, traceback
from datetime import datetime, timezone
from System import Activator

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)
MSG_ID = "0305"

def _try_set(obj, name: str, value) -> bool:
    """lowerCamel / PascalCase 紐⑤몢 ?쒕룄?섏뿬 .NET ?꾨줈?쇳떚瑜??명똿"""
    for k in (name, name[:1].upper()+name[1:] if name else name):
        try:
            if hasattr(obj, k):
                setattr(obj, k, value)
                return True
        except Exception:
            pass
    return False

def _cs(name: str):
    """
    ????먯깋 踰붿쐞 ?뺣?:
      - nFusion.Model.msg_0305
      - nFusion.Model.CommonType
      - nFusion.Model
    """
    for modname in ('nFusion.Model.msg_0305', 'nFusion.Model.CommonType', 'nFusion.Model'):
        try:
            mod = importlib.import_module(modname)
            t = getattr(mod, name, None)
            if t is not None:
                return t
        except Exception:
            pass
    return None

def _new_msg0305():
    """
    ?덉쟾???몄뒪?댁뒪 ?앹꽦:
      1) ?뚮젮吏??꾨낫紐낆쓣 ?쒖감 ?쒕룄 (?ㅽ뙣 ??怨꾩냽)
      2) 紐⑤뱢 ??public ??낅뱾???뚯븘媛硫?湲곕낯?앹꽦?먮줈 Activator.CreateInstance ?쒕룄
         ???앹꽦 ?깃났 & 'status' 怨꾩뿴 ?꾨줈?쇳떚 蹂댁쑀 ??梨꾪깮
    """
    # ???뚮젮吏??꾨낫紐??쇱씠釉뚮윭由щ퀎 ?ㅼ씠諛??몄감 怨좊젮)
    CANDIDATE = (
        "ReplanStatusInfo", "ReplanningStatusInfo",
        "MissionPlanningStatusInfo", "MissionPlanningStateInfo",
        "MissionPlanningStatus", "MissionPlanReplanStatus"
    )
    for n in CANDIDATE:
        t = _cs(n)
        if t is None:
            continue
        try:
            obj = Activator.CreateInstance(t)  # t() ???Activator濡??앹꽦 ?덉젙??            # ?꾨낫 ?꾨줈?쇳떚 議댁옱 ?뺤씤(??以??섎굹留??덉뼱??OK)
            if any(hasattr(obj, k) for k in ("missionPlanningStatus", "MissionPlanningStatus", "status", "Status")):
                return obj
        except Exception:
            continue

    # ???꾩껜 public ????ㅼ틪: ?앹꽦 ?깃났 + status 怨꾩뿴 ?꾨줈?쇳떚 蹂댁쑀 ??梨꾪깮
    tried = []
    for modname in ('nFusion.Model.msg_0305', 'nFusion.Model.CommonType', 'nFusion.Model'):
        try:
            mod = importlib.import_module(modname)
        except Exception:
            continue
        for name, t in list(mod.__dict__.items()):
            if not isinstance(t, type):
                continue
            tried.append(f"{modname}.{name}")
            try:
                obj = Activator.CreateInstance(t)
                if any(hasattr(obj, k) for k in ("missionPlanningStatus", "MissionPlanningStatus", "status", "Status")):
                    return obj
            except Exception:
                continue

    raise NameError("msg_0305 type not found (tried: " + ", ".join(tried[:20]) + ("..." if len(tried) > 20 else "") + ")")

def _dict_to_obj(body: dict):
    obj = _new_msg0305()
    # timestamp
    _try_set(obj, "timestamp", int(body.get("timestamp", _now_ms())))
    # source ?명솚
    src = body.get("source") or body.get("Source") or body.get("requestModuleName") or "MMR"
    if not _try_set(obj, "source", str(src)):
        _try_set(obj, "Source", str(src))
    # status 怨꾩뿴(理쒖슦?? missionPlanningStatus)
    st = int(body.get("missionPlanningStatus", body.get("status", 0)))
    if not (_try_set(obj, "missionPlanningStatus", st) or _try_set(obj, "status", st)):
        # 洹몃옒??紐??ｌ쑝硫?留덉?留됱쑝濡?StatusCode ?깅룄 ?쒕룄
        _try_set(obj, "StatusCode", st)
    # reason(?듭뀡)
    if "replanReason" in body:
        _try_set(obj, "replanReason", str(body["replanReason"]))
    return obj

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    try:
        msg = _dict_to_obj(body_dict or {})
        node_messenger.Push(msg)
        log = f"[0305] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n[0305] PUSH ?꾨즺"
        return log.encode("utf-8","ignore")
    except Exception as e:
        # ?덉쇅瑜?濡쒓퉭 臾몄옄?댁뿉 ?ы븿?쒖폒 ?곸쐞 濡쒓굅濡쒕룄 蹂댁씠寃?        err = f"[0305][ERR] {e}\n{traceback.format_exc()}"
        return (err + "\n").encode("utf-8","ignore")

def make_random_and_push(node_messenger) -> bytes:
    body = {"timestamp": _now_ms(), "source": "MMR", "missionPlanningStatus": 2, "replanReason":"珥덇린?꾨Т?ш퀎??}
    return make_and_push(body, node_messenger)

