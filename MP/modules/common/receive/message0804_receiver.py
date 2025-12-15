# receive/message0804_receiver.py
# ??????????????????????????????????????????????????????????????
from dll_files.nFusionImports import *              # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0804 import MissionRestartCommand
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ?????????? ?/?뚮Ц???덉쟾 ?묎렐 ?ы띁 ??????????
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ?????????? CLR ??dict 蹂????????????
def _mission_restart_command_to_dict(cmd: MissionRestartCommand) -> dict:
    return {
        "timestamp":     _get(cmd, "timestamp",   "Timestamp"),
        "inputMissionID": _get(cmd, "inputMissionID", "InputMissionID")
    }

# ?????????? Receiver ?대옒????????????
class MissionRestartCommandReceiver_0804(
    IFusionReceive[MissionRestartCommand], IsLocal, IsSingletone
):
    """0804 MissionRestartCommand 硫붿떆吏 ?섏떊 由ъ떆踰?""
    __namespace__ = "MissionRestartCommandReceiver_0804"

    def Receive(self, data: MissionRestartCommand, src):
        try:
            # 1) DB ???            received_db.set_received_0804(data)

            # 2) GUI??JSON 諛붾뵒 ?뺥깭濡??꾨떖
            notify(
                "0804",
                json.dumps(
                    _mission_restart_command_to_dict(data),
                    ensure_ascii=False
                ).encode()
            )

        except Exception:
            print("[ERROR][Receive-0804] traceback ?볛넃??)
            traceback.print_exc(file=sys.stderr)

