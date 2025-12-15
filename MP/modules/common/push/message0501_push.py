# modules/common/push/message0501_push.py
# MissionProgress(0501) ?몄떆 - ?쒕꼫?덉씠??fallback & ?꾧꺽 留ㅽ븨

import json, importlib
from datetime import datetime, timezone
from System.Collections.Generic import List
from nFusion.Model.msg_0501 import *       # C# 紐⑤뜽 ?곗꽑 寃??from nFusion.Model.CommonType import *     # 怨듯넻 ???from System import String                  # (?꾩슂 ???ъ슜)
MSG_ID = "0501"

# ?????????????????????????? ?쒓컙/Epoch ??????????????????????????
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

# ?????????????????????????? ?좏떥(由ы뵆?됱뀡) ??????????????????????????
def _try_set(obj, name: str, value) -> bool:
    """lowerCamel / PascalCase ?묒そ?쇰줈 ?쒕룄"""
    if not name:
        return False
    for k in (name, name[:1].upper()+name[1:]):
        try:
            if hasattr(obj, k):
                setattr(obj, k, value)
                return True
        except Exception:
            pass
    return False

def _cs(name: str):
    """?꾩옱 ?꾩뿭 ??msg_0501 ??CommonType ??猷⑦듃 ?쒖꽌濡????寃??""
    t = globals().get(name)
    if t is not None:
        return t
    for modname in ('nFusion.Model.msg_0501', 'nFusion.Model.CommonType', 'nFusion.Model'):
        try:
            mod = importlib.import_module(modname)
            t = getattr(mod, name, None)
            if t is not None:
                return t
        except Exception:
            pass
    return None

def _new(name: str):
    t = _cs(name)
    if t is None:
        raise NameError(f'type not found: {name}')
    return t()

def _as_int(v, default=None):
    try:
        return int(str(v))
    except Exception:
        return default

# ????????????????????? dict ??C# 媛앹껜 留ㅽ븨 ?????????????????????
def _dict_to_CurrentIndividualMission(d: dict):
    obj = _new('CurrentIndividualMission')
    if 'individualMissionID' in d:
        _try_set(obj, 'individualMissionID', _as_int(d['individualMissionID'], 0))
    return obj

def _dict_to_IndividualMissionProgressStatus(d: dict):
    obj = _new('IndividualMissionProgressStatus')
    if 'aircraftID' in d:
        _try_set(obj, 'aircraftID', _as_int(d['aircraftID'], 0))
    if 'currentIndividualMission' in d and isinstance(d['currentIndividualMission'], dict):
        _try_set(obj, 'currentIndividualMission', _dict_to_CurrentIndividualMission(d['currentIndividualMission']))
    if 'currentIndividualMissionProgress' in d:
        _try_set(obj, 'currentIndividualMissionProgress', _as_int(d['currentIndividualMissionProgress'], 0))
    return obj

def _dict_to_MissionProgress(d: dict):
    obj = _new('MissionProgress')

    # timestamp/source
    _try_set(obj, 'timestamp', _as_int(d.get('timestamp', _now_ms()), _now_ms()))
    src_val = d.get('source', d.get('Source', d.get('requestModuleName', '')))
    if str(src_val).strip() != '':
        if not _try_set(obj, 'source', str(src_val)):
            _try_set(obj, 'Source', str(src_val))

    # ?곸쐞 ID???듭뀡)
    if 'currentMissionPlanID' in d:
        _try_set(obj, 'currentMissionPlanID', _as_int(d['currentMissionPlanID'], 0))
    if 'currentInputMissionID' in d:
        _try_set(obj, 'currentInputMissionID', _as_int(d['currentInputMissionID'], 0))

    # ?섏쐞 由ъ뒪??List<IndividualMissionProgressStatus>)
    items = d.get('individualMissionProgressStatusList', [])
    if isinstance(items, list):
        T = _cs('IndividualMissionProgressStatus') or object
        lst = List[T]()  # .NET ?쒕꽕由?由ъ뒪??        for it in items:
            try:
                lst.Add(_dict_to_IndividualMissionProgressStatus(it if isinstance(it, dict) else {}))
            except Exception:
                # 媛쒕퀎 ??ぉ ?ㅻ쪟???ㅽ궢
                pass
        _try_set(obj, 'individualMissionProgressStatusList', lst)

    return obj

# ???????????????????????? ?쒕꼫?덉씠??濡쒕뵫 ????????????????????????
def _resolve_body_generator():
    """
    媛?ν븳 寃쎈줈瑜??쒖꽌?濡??쒕룄:
      1) generator.message0501_generator.make_msg0501_body
      2) make_message_generator.make_msg0501_body
      3) nf_example_gen.make_msg0501_body
      4) make_message_generator.make_body(msg_id)  # 踰붿슜 ?쒓렇?덉쿂
    """
    candidates = [
        ('generator.message0501_generator', 'make_msg0501_body', False),
        ('make_message_generator',         'make_msg0501_body', False),
        ('nf_example_gen',                 'make_msg0501_body', False),
        ('make_message_generator',         'make_body',         True),  # needs msg_id
    ]
    for modname, funcname, needs_id in candidates:
        try:
            mod = importlib.import_module(modname)
            fn = getattr(mod, funcname, None)
            if fn is None:
                continue
            return (fn, needs_id)
        except Exception:
            continue
    return (None, False)

_GEN_FN, _GEN_NEEDS_ID = _resolve_body_generator()

def _gen_body():
    """?쒕꼫?덉씠?곕? ?듯빐 body ?앹꽦. ?ㅽ뙣 ???덉쟾??理쒖냼 蹂몃Ц 諛섑솚."""
    try:
        if _GEN_FN:
            return _GEN_FN(MSG_ID) if _GEN_NEEDS_ID else _GEN_FN()
    except Exception:
        pass
    # ?덉쟾 ?대갚(鍮?由ъ뒪???덉슜)
    return {
        "timestamp": _now_ms(),
        "Source": "DSC",
        "currentMissionPlanID": 0,
        "currentInputMissionID": 0,
        "individualMissionProgressStatusList": []
    }

# ???????????????????????? ?몃? API ????????????????????????
def _normalize_body(d: dict) -> dict:
    """?꾨씫 ?꾨뱶 湲곕낯媛?梨꾩슦湲?/ ???肄붿뼱??""
    if not isinstance(d, dict):
        return _gen_body()

    out = {}
    out['timestamp'] = _as_int(d.get('timestamp', _now_ms()), _now_ms())
    src = d.get('source', d.get('Source', d.get('requestModuleName', 'DSC')))
    out['source'] = str(src) if str(src).strip() != '' else 'DSC'

    if 'currentMissionPlanID' in d:
        out['currentMissionPlanID'] = _as_int(d['currentMissionPlanID'], 0)
    if 'currentInputMissionID' in d:
        out['currentInputMissionID'] = _as_int(d['currentInputMissionID'], 0)

    items = d.get('individualMissionProgressStatusList', [])
    norm_items = []
    if isinstance(items, list):
        for it in items:
            if isinstance(it, dict):
                # ?꾩닔/以묒슂 ?꾨뱶留??뺣룉
                one = {}
                if 'aircraftID' in it:
                    one['aircraftID'] = _as_int(it['aircraftID'], 0)
                if 'currentIndividualMission' in it and isinstance(it['currentIndividualMission'], dict):
                    sub = {}
                    if 'individualMissionID' in it['currentIndividualMission']:
                        sub['individualMissionID'] = _as_int(it['currentIndividualMission']['individualMissionID'], 0)
                    one['currentIndividualMission'] = sub
                if 'currentIndividualMissionProgress' in it:
                    one['currentIndividualMissionProgress'] = _as_int(it['currentIndividualMissionProgress'], 0)
                norm_items.append(one)
    out['individualMissionProgressStatusList'] = norm_items
    return out

def _dict_to_obj(body_dict: dict):
    return _dict_to_MissionProgress(body_dict)

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    """
    ?몃??먯꽌 body_dict瑜?二쇰㈃ .NET 紐⑤뜽濡?蹂????Push.
    諛섑솚媛? 濡쒓렇(UTF-8 bytes)
    """
    body_dict = _normalize_body(body_dict)
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[{MSG_ID}] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[{MSG_ID}] PUSH ?꾨즺"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    """
    ?쒕뜡/?섑뵆 蹂몃Ц???쒕꼫?덉씠?곕줈 留뚮뱾怨?Push.
    ?쒕꼫?덉씠?곌? 由ъ뒪?몃? 諛섑솚?대룄 泥섎━ 媛???щ윭 嫄??꾩넚).
    """
    got = _gen_body()
    logs = []
    if isinstance(got, list):
        for d in got:
            logs.append(make_and_push(d if isinstance(d, dict) else {}, node_messenger))
    else:
        logs.append(make_and_push(got if isinstance(got, dict) else {}, node_messenger))
    return b"\n".join(logs)

