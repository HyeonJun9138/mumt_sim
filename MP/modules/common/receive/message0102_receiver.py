# modules/common/receive/message0102_receiver.py
# auto-fixed at 2025-09-19

from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0102 import *              # C# 紐⑤뜽
from nFusion.Model.CommonType import *            # 怨듯넻 ???from .database import received_db
from receive_center import notify
import json, traceback, sys, os

# ?/?뚮Ц???덉쟾 ?묎렐
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

def _coerce_int(v):
    try:
        return int(v)
    except Exception:
        try:
            return int(getattr(v, "value"))
        except Exception:
            try:
                return int(str(v))
            except Exception:
                return None

def _to_dict_ModuleStatus(obj):
    """
    0102 ?섏떊 ???쒖? 諛붾뵒(dict):
      { "timestamp": int(ms_2000), "status": int(0|1|2), "source": "MMR|MSM|MOB|..." }
    - ?ㅼ뼇???/?뚮Ц?먃룸퀎移?쓣 紐⑤몢 ?≪닔?섏뿬 ??3?ㅻ줈 ?듭씪
    - 媛??앹꽦/媛怨??놁쓬(?덉쓣 ?뚮쭔 異붿텧)
    """
    d = {}

    # timestamp
    ts = _get(obj, 'timestamp','Timestamp','timeStamp','TimeStamp','ts','TS')
    ts_i = _coerce_int(ts)
    if ts_i is not None:
        d['timestamp'] = ts_i

    # source (SourceModuleName? ?곗? ?딆쓬)
    src = _get(
        obj,
        'source','Source',               # ?쒖?
        'requestModuleName','RequestModuleName',  # ??蹂꾩묶
        'module','Module','sourceModule','SourceModule'
    )
    if src is not None and str(src) != '':
        d['source'] = str(src)

    # status
    st = _get(
        obj,
        'status','Status',
        'moduleStatus','ModuleStatus',
        'healthStatus','HealthStatus',
        'state','State'
    )
    st_i = _coerce_int(st)
    if st_i is not None:
        d['status'] = st_i

    return d

class ModuleStatusReceiver_0102(IFusionReceive[ModuleStatus], IsLocal, IsSingletone):
    """0102 ModuleStatus 硫붿떆吏 ?섏떊 由ъ떆踰?""
    __namespace__ = "ModuleStatusReceiver_0102"

    def Receive(self, data: ModuleStatus, src):
        try:
            # ?좏깮: 理쒓렐 ?섏떊 ?먮낯?????(?덉쑝硫?
            try:
                received_db.set_received_0102(data)
            except Exception:
                pass

            body = _to_dict_ModuleStatus(data)
            # ??긽 ?쒖? ?ㅻ뱾留?notify
            notify("0102", json.dumps(body, ensure_ascii=False).encode("utf-8","ignore"))

        except Exception:
            print("[ERROR][Receive-0102] traceback ?볛넃??)
            traceback.print_exc(file=sys.stderr)

