from __future__ import annotations


class RTDEError(RuntimeError):
    """reBot RTDE基础异常。"""


class RTDEConnectionError(RTDEError):
    """网络连接异常。"""


class RTDETimeoutError(RTDEError):
    """请求超时异常。"""


class RTDEProtocolError(RTDEError):
    """协议数据异常。"""


class RTDEControlError(RTDEError):
    """控制箱拒绝请求。"""

    # 保存控制箱返回的错误信息
    def __init__(
        self,
        code: str,
        message: str,
        related_sequence: int,
    ) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message
        self.related_sequence = related_sequence
