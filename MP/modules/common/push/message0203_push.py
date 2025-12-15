# modules/common/push/message0203_push.py
# auto-generated at 2025-08-24T20:13:14.007841+00:00


import json, importlib
from datetime import datetime, timezone
from modules.common import db_paths
from System.Collections.Generic import List
from nFusion.Model.msg_0203 import *    # C# 紐⑤뜽(?곗꽑)
from nFusion.Model.CommonType import *     # 怨듯넻 ?????긽)
from System import Int32, Single, UInt32, UInt64
from generator.message0203_generator import make_msg0203_body
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)
MSG_ID = "0203"
def _try_set(obj, name: str, value) -> bool:
    # lowerCamel ?먮뒗 PascalCase ?????쒕룄
    for k in (name, name[:1].upper()+name[1:] if name else name):
        try:
            if hasattr(obj, k):
                setattr(obj, k, value)
                return True
        except Exception:
            pass
    return False
def _cs(name: str):
    # ?꾩옱 ?꾩뿭 ??msg_ID 紐⑤뱢 ??CommonType ??猷⑦듃 ?쒖쑝濡?寃??    t = globals().get(name)
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

# ?? Embedded TX/DB rules (self-contained) ??????????????????????????????????
TX_FIELD_WHITELIST = {
    "0201": ["timestamp", "inputMissionPackageID"],
    "0203": ["timestamp", "missionReferencePackageID"],
    "0301": ["timestamp", "missionPlanID"],
    "0302": ["timestamp", "individualMissionPackageID"],
    "0303": ["timestamp", "pathID"],
    "0304": ["timestamp", "pathID"],
}

DB_DIR_RULES = {
    "0201": "InputMissionPlan",
    "0203": "MissionReferenceInfo",
    "0301": "MissionPlan",
    "0302": "IndividualMissionPlan",
    "0303": "FlightPath",
    "0304": "FlightPath",
}

def _select_tx_fields(body: dict, fields: list) -> dict:
    """?붿씠?몃━?ㅽ듃濡??좊퀎: timestamp / source 怨꾩뿴 ?대갚 / ?섎㉧吏 ID瑜섎쭔 ?④?"""
    out = {}
    low = {k.lower(): k for k in body.keys()}

    def _get(key: str):
        kl = key.lower()
        if kl in low:
            return body[low[kl]]
        return None

    ts = _get("timestamp")
    if ts is not None:
        out["timestamp"] = int(ts)

    s  = _get("source")
    sm = _get("Source") or _get("Source")
    rq = _get("requestModuleName") or _get("requestmodulename")
    src_val = s or sm or rq
    if src_val:
        out["Source"] = str(src_val)

    for f in fields:
        if f in ("timestamp","source","Source","requestModuleName"):
            continue
        v = _get(f)
        if v is not None:
            try:
                out[f] = int(v)
            except Exception:
                out[f] = v
    return out

def _project_root_for_push_file(__file_path: str):
    from pathlib import Path
    return Path(__file_path).resolve().parents[4]

def _db_dir_for(msgid: str, __file_path: str) -> str:
    name = DB_DIR_RULES.get(msgid, f"msg_{msgid}")
    try:
        if msgid in ("0201", "0203"):
            return str(db_paths.ensure_db_payload(name))
        root = db_paths.get_active_db_root()
        target = root / name
        target.mkdir(parents=True, exist_ok=True)
        return str(target)
    except Exception:
        base = _project_root_for_push_file(__file_path)
        return str(base / "database" / name)

def _list_numeric_ids(dirname: str, prefix_first_char: str | None = None) -> list[int]:
    import os, glob
    ids = []
    for p in glob.glob(os.path.join(dirname, "*.json")):
        stem = os.path.splitext(os.path.basename(p))[0]
        if stem.isdigit():
            if prefix_first_char and stem[0] not in prefix_first_char:
                continue
            ids.append(int(stem))
    ids.sort()
    return ids

def _dict_to_Coordinate(data: dict):
    obj = _new('Coordinate')
    if "latitude" in data: _try_set(obj, "latitude", float(data["latitude"]))
    if "longitude" in data: _try_set(obj, "longitude", float(data["longitude"]))
    if "altitude" in data: _try_set(obj, "altitude", int(data["altitude"]))
    return obj

def _dict_to_TakeOverInfo(data: dict):
    obj = _new('TakeOverInfo')
    if "aircraftID" in data: _try_set(obj, "aircraftID", int(data["aircraftID"]))
    if "coordinate" in data and isinstance(data["coordinate"], dict):
        _try_set(obj, "coordinate", _dict_to_Coordinate(data["coordinate"]))
    return obj

def _dict_to_HandOverInfo(data: dict):
    obj = _new('HandOverInfo')
    if "aircraftID" in data: _try_set(obj, "aircraftID", int(data["aircraftID"]))
    if "coordinate" in data and isinstance(data["coordinate"], dict):
        _try_set(obj, "coordinate", _dict_to_Coordinate(data["coordinate"]))
    return obj

def _dict_to_RTBCoordinate(data: dict):
    obj = _new('RTBCoordinate')
    if "latitude" in data: _try_set(obj, "latitude", float(data["latitude"]))
    if "longitude" in data: _try_set(obj, "longitude", float(data["longitude"]))
    if "altitude" in data: _try_set(obj, "altitude", int(data["altitude"]))
    return obj

def _dict_to_AreaLatLon(data: dict):
    obj = _new('AreaLatLon')
    if "latitude" in data: _try_set(obj, "latitude", float(data["latitude"]))
    if "longitude" in data: _try_set(obj, "longitude", float(data["longitude"]))
    return obj

def _dict_to_AltitudeLimits(data: dict):
    obj = _new('AltitudeLimits')
    if "lowerLimit" in data: _try_set(obj, "lowerLimit", float(data["lowerLimit"]))
    if "upperLimit" in data: _try_set(obj, "upperLimit", float(data["upperLimit"]))
    return obj

def _dict_to_FlightArea(data: dict):
    obj = _new('FlightArea')
    if "flightAreaID" in data: _try_set(obj, "flightAreaID", int(data["flightAreaID"]))
    if "areaLatLonList" in data and isinstance(data["areaLatLonList"], list):
        T = _cs('AreaLatLon') or object
        lst = List[T]()
        for item in data["areaLatLonList"]: lst.Add(_dict_to_AreaLatLon(item if isinstance(item, dict) else {}))
        _try_set(obj, "areaLatLonList", lst)
    if "altitudeLimits" in data and isinstance(data["altitudeLimits"], dict):
        _try_set(obj, "altitudeLimits", _dict_to_AltitudeLimits(data["altitudeLimits"]))
    return obj

def _dict_to_ProhibitedArea(data: dict):
    obj = _new('ProhibitedArea')
    if "prohibitedAreaID" in data: _try_set(obj, "prohibitedAreaID", int(data["prohibitedAreaID"]))
    if "areaLatLonList" in data and isinstance(data["areaLatLonList"], list):
        T = _cs('AreaLatLon') or object
        lst = List[T]()
        for item in data["areaLatLonList"]: lst.Add(_dict_to_AreaLatLon(item if isinstance(item, dict) else {}))
        _try_set(obj, "areaLatLonList", lst)
    if "altitudeLimits" in data and isinstance(data["altitudeLimits"], dict):
        _try_set(obj, "altitudeLimits", _dict_to_AltitudeLimits(data["altitudeLimits"]))
    return obj

def _dict_to_FlightReferenceInfoData(data: dict):
    obj = _new('FlightReferenceInfoData')
    if "timestamp" in data: _try_set(obj, "timestamp", int(data["timestamp"]))
    if "missionReferencePackageID" in data: _try_set(obj, "missionReferencePackageID", int(data["missionReferencePackageID"]))
    if "inputTimestamp" in data: _try_set(obj, "inputTimestamp", int(data["inputTimestamp"]))
    if "takeOverInfoList" in data and isinstance(data["takeOverInfoList"], list):
        T = _cs('TakeOverInfo') or object
        lst = List[T]()
        for item in data["takeOverInfoList"]: lst.Add(_dict_to_TakeOverInfo(item if isinstance(item, dict) else {}))
        _try_set(obj, "takeOverInfoList", lst)
    if "handOverInfoList" in data and isinstance(data["handOverInfoList"], list):
        T = _cs('HandOverInfo') or object
        lst = List[T]()
        for item in data["handOverInfoList"]: lst.Add(_dict_to_HandOverInfo(item if isinstance(item, dict) else {}))
        _try_set(obj, "handOverInfoList", lst)
    if "rtbCoordinateList" in data and isinstance(data["rtbCoordinateList"], list):
        T = _cs('RTBCoordinate') or object
        lst = List[T]()
        for item in data["rtbCoordinateList"]: lst.Add(_dict_to_RTBCoordinate(item if isinstance(item, dict) else {}))
        _try_set(obj, "rtbCoordinateList", lst)
    if "flightAreaList" in data and isinstance(data["flightAreaList"], list):
        T = _cs('FlightArea') or object
        lst = List[T]()
        for item in data["flightAreaList"]: lst.Add(_dict_to_FlightArea(item if isinstance(item, dict) else {}))
        _try_set(obj, "flightAreaList", lst)
    if "prohibitedAreaList" in data and isinstance(data["prohibitedAreaList"], list):
        T = _cs('ProhibitedArea') or object
        lst = List[T]()
        for item in data["prohibitedAreaList"]: lst.Add(_dict_to_ProhibitedArea(item if isinstance(item, dict) else {}))
        _try_set(obj, "prohibitedAreaList", lst)
    return obj

def _dict_to_FlightReferenceInfo(data: dict):
    obj = _new('FlightReferenceInfo')
    if "timestamp" in data: _try_set(obj, "timestamp", int(data["timestamp"]))
    if "missionReferencePackageID" in data: _try_set(obj, "missionReferencePackageID", int(data["missionReferencePackageID"]))
    if "inputTimestamp" in data: _try_set(obj, "inputTimestamp", int(data["inputTimestamp"]))
    if "takeOverInfoList" in data and isinstance(data["takeOverInfoList"], list):
        T = _cs('TakeOverInfo') or object
        lst = List[T]()
        for item in data["takeOverInfoList"]: lst.Add(_dict_to_TakeOverInfo(item if isinstance(item, dict) else {}))
        _try_set(obj, "takeOverInfoList", lst)
    if "handOverInfoList" in data and isinstance(data["handOverInfoList"], list):
        T = _cs('HandOverInfo') or object
        lst = List[T]()
        for item in data["handOverInfoList"]: lst.Add(_dict_to_HandOverInfo(item if isinstance(item, dict) else {}))
        _try_set(obj, "handOverInfoList", lst)
    if "rtbCoordinateList" in data and isinstance(data["rtbCoordinateList"], list):
        T = _cs('RTBCoordinate') or object
        lst = List[T]()
        for item in data["rtbCoordinateList"]: lst.Add(_dict_to_RTBCoordinate(item if isinstance(item, dict) else {}))
        _try_set(obj, "rtbCoordinateList", lst)
    if "flightAreaList" in data and isinstance(data["flightAreaList"], list):
        T = _cs('FlightArea') or object
        lst = List[T]()
        for item in data["flightAreaList"]: lst.Add(_dict_to_FlightArea(item if isinstance(item, dict) else {}))
        _try_set(obj, "flightAreaList", lst)
    if "prohibitedAreaList" in data and isinstance(data["prohibitedAreaList"], list):
        T = _cs('ProhibitedArea') or object
        lst = List[T]()
        for item in data["prohibitedAreaList"]: lst.Add(_dict_to_ProhibitedArea(item if isinstance(item, dict) else {}))
        _try_set(obj, "prohibitedAreaList", lst)
    return obj




def _dict_to_obj(body_dict: dict):
    return _dict_to_FlightReferenceInfo(body_dict)

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    # TX ?붿씠?몃━?ㅽ듃媛 ?덉쑝硫?理쒖쥌 ?꾩넚 ???좊퀎(?쒕꼫?덉씠?곌? ?띾??섍쾶 留뚮뱾?대룄 理쒖냼?꾨뱶留?蹂대깂)
    wl = TX_FIELD_WHITELIST.get(MSG_ID)
    if wl and isinstance(body_dict, dict):
        body_dict = _select_tx_fields(body_dict, wl)
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0203] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0203] PUSH ?꾨즺"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    # DB 湲곕컲 硫붿떆吏??DB???뚯씪紐??レ옄).json??ID濡??ъ슜?섏뿬 理쒖냼 ?꾨뱶留??꾩넚
    if MSG_ID in DB_DIR_RULES:
        dbdir = _db_dir_for(MSG_ID, __file__)
        # 0304(?좎씤湲?pathID)??1/2/3 ?쒖옉留??꾩넚(湲곗〈 洹쒖튃 ?좎?)
        needs_prefix = "123" if MSG_ID == "0304" else None
        ids = _list_numeric_ids(dbdir, needs_prefix)
        logs = []
        for vid in ids:
            wl = TX_FIELD_WHITELIST.get(MSG_ID, [])
            body = {
                "timestamp": int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000),
                "Source": "DSC",
            }
            # ID ?꾨뱶 寃곗젙
            if "inputMissionPackageID" in wl:          body["inputMissionPackageID"] = vid
            if "missionReferencePackageID" in wl:      body["missionReferencePackageID"] = vid
            if "missionPlanID" in wl:                  body["missionPlanID"] = vid
            if "individualMissionPackageID" in wl:     body["individualMissionPackageID"] = vid
            if "pathID" in wl:                         body["pathID"] = vid
            logs.append(make_and_push(body, node_messenger))
        return b"\n".join(logs) if logs else b""
    else:
        # 鍮?DB 硫붿떆吏???쒕꼫?덉씠?????꾩슂 ???붿씠?몃━?ㅽ듃濡??좊퀎
        body = make_msg0203_body()
        # ??0102 諛⑹뼱: body媛 鍮꾧굅??dict媛 ?꾨땲硫?理쒖냼 ?명듃濡?梨꾩?
        if MSG_ID == "0102":
            if not isinstance(body, dict) or not body:
                body = {
                    "timestamp": int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000),
                    "status": 1,  # ?뺤긽
                    "Source": "DSC",
                }
        wl = TX_FIELD_WHITELIST.get(MSG_ID)
        if wl and isinstance(body, dict):
            body = _select_tx_fields(body, wl)
        return make_and_push(body, node_messenger)

