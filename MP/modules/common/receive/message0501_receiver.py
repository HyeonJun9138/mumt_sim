# modules/common/receive/message0501_receiver.py
# MissionProgress(0501) ?섏떊 由ъ떆踰?- ?덉젙 蹂??& notify

from dll_files.nFusionImports import *          # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0501 import *            # C# 紐⑤뜽
from nFusion.Model.CommonType import *          # 怨듯넻 ???from .database import received_db
from receive_center import notify
import json, traceback, sys

# ?????????????????????????? ?좏떥 ??????????????????????????
def _get(obj, *names):
    """?/?뚮Ц???쇱슜 ?띿꽦 ?덉쟾 ?묎렐"""
    for n in names:
        try:
            if hasattr(obj, n):
                return getattr(obj, n)
        except Exception:
            pass
    return None

def _as_int(v):
    try:
        # UInt32/UInt64/臾몄옄??紐⑤몢 ?섏슜
        return int(str(v))
    except Exception:
        return None

def _iter_safe(coll):
    """IEnumerable(.NET) / list 紐⑤몢 ?덉쟾 ?쒗쉶"""
    if coll is None:
        return []
    try:
        for it in coll:
            yield it
    except Exception:
        return []

# ??????????????????????? C# ??dict 蹂?????????????????????????
def _to_dict_CurrentIndividualMission(obj):
    d = {}
    v = _get(obj, 'individualMissionID', 'IndividualMissionID')
    iv = _as_int(v)
    if iv is not None:
        d['individualMissionID'] = iv
    return d

def _to_dict_IndividualMissionProgressStatus(obj):
    d = {}
    v = _get(obj, 'aircraftID', 'AircraftID')
    iv = _as_int(v)
    if iv is not None:
        d['aircraftID'] = iv

    sub = _get(obj, 'currentIndividualMission', 'CurrentIndividualMission')
    if sub is not None:
        d['currentIndividualMission'] = _to_dict_CurrentIndividualMission(sub)

    v = _get(obj, 'currentIndividualMissionProgress', 'CurrentIndividualMissionProgress')
    iv = _as_int(v)
    if iv is not None:
        d['currentIndividualMissionProgress'] = iv
    return d

def _to_dict_MissionProgress(obj, src_hint=None):
    d = {}

    v = _get(obj, 'timestamp', 'Timestamp')
    iv = _as_int(v)
    if iv is not None:
        d['timestamp'] = iv

    # source???щ윭 ?꾨낫?먯꽌 ?대갚 (紐⑤뜽 ?띿꽦 ???붿껌???대쫫/臾몄옄????RequestModuleName)
    s = _get(obj, 'source', 'Source', 'requestModuleName', 'RequestModuleName')
    if s is None or str(s).strip() == '':
        if src_hint is not None:
            try:
                s = getattr(src_hint, 'Name', None) or str(src_hint)
            except Exception:
                s = None
    if s is not None and str(s).strip() != '':
        d['source'] = str(s)

    v = _get(obj, 'currentMissionPlanID', 'CurrentMissionPlanID')
    iv = _as_int(v)
    if iv is not None:
        d['currentMissionPlanID'] = iv

    v = _get(obj, 'currentInputMissionID', 'CurrentInputMissionID')
    iv = _as_int(v)
    if iv is not None:
        d['currentInputMissionID'] = iv

    coll = _get(obj, 'individualMissionProgressStatusList', 'IndividualMissionProgressStatusList')
    items = []
    for it in _iter_safe(coll):
        try:
            items.append(_to_dict_IndividualMissionProgressStatus(it))
        except Exception:
            # 媛쒕퀎 ?꾩씠???ㅻ쪟???ㅽ궢
            pass
    if items:
        d['individualMissionProgressStatusList'] = items

    return d

# ????????????????????????? Receiver ?????????????????????????
class MissionProgressReceiver_0501(IFusionReceive[MissionProgress], IsLocal, IsSingletone):
    """0501 MissionProgress 硫붿떆吏 ?섏떊 由ъ떆踰?""

    __namespace__ = "MissionProgressReceiver_0501"

    def Receive(self, data: MissionProgress, src):
        try:
            # ?먮낯 蹂닿? (?덉쑝硫?
            try:
                received_db.set_received_0501(data)
            except Exception:
                pass

            body = _to_dict_MissionProgress(data, src_hint=src)

            # notify濡?諛붿씠?덈━ ?꾩넚(JSON UTF-8)
            payload = json.dumps(body, ensure_ascii=False).encode("utf-8", "ignore")
            notify("0501", payload)

        except Exception:
            print("[ERROR][Receive-0501] traceback ?볛넃??)
            traceback.print_exc(file=sys.stderr)

