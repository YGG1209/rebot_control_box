from __future__ import annotations

import argparse
import fcntl
import os
import socket
import stat
import threading
import time
from pathlib import Path
from typing import Any

from reBotArm_control_box.control_daemon import ControlDaemon, DaemonState
from reBotArm_control_box.ipc_protocol import (
    DEFAULT_IPC_SOCKET_PATH,
    IPC_MAX_PACKET_SIZE,
    IPCMessageType,
    IPCProtocolError,
    decode_packet,
    encode_packet,
    make_error_payload,
    parse_servoj_payload,
    snapshot_to_payload,
)


class ControlIPCServer:
    # 初始化IPC服务端
    def __init__(
        self,
        daemon: Any,
        socket_path: str = DEFAULT_IPC_SOCKET_PATH,
        *,
        socket_mode: int = 0o600,
    ) -> None:
        self._daemon = daemon
        self._socket_path = Path(socket_path)
        self._lock_path = Path(f"{socket_path}.lock")
        self._socket_mode = socket_mode
        self._stop_event = threading.Event()
        self._server_socket: socket.socket | None = None
        self._client_socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._lock_fd: int | None = None
        self._owns_socket_path = False
        self._lifecycle_lock = threading.Lock()
        self._client_lock = threading.Lock()

    # 返回IPC服务运行状态
    @property
    def running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    # 创建Unix Domain Socket并启动服务线程
    def start(self) -> None:
        with self._lifecycle_lock:
            if self.running:
                raise RuntimeError("IPC server is already running")

            try:
                self._acquire_instance_lock()
                self._prepare_socket_path()
                server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
                server.bind(str(self._socket_path))
                self._owns_socket_path = True
                os.chmod(self._socket_path, self._socket_mode)
                server.listen(1)
                server.settimeout(0.2)
            except Exception:
                if "server" in locals():
                    server.close()
                self._remove_owned_socket_path()
                self._release_instance_lock()
                raise

            self._server_socket = server
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._serve_loop,
                name="rebot-ipc-server",
                daemon=True,
            )
            self._thread.start()

    # 停止服务线程并移除Socket文件
    def stop(self) -> None:
        with self._lifecycle_lock:
            self._stop_event.set()
            self._close_client()

            server = self._server_socket
            self._server_socket = None
            if server is not None:
                server.close()

            thread = self._thread
            if (
                thread is not None
                and thread.is_alive()
                and thread is not threading.current_thread()
            ):
                thread.join(timeout=2.0)
            self._thread = None
            self._remove_owned_socket_path()
            self._release_instance_lock()

    # 持续接收单个可信本机客户端
    def _serve_loop(self) -> None:
        while not self._stop_event.is_set():
            server = self._server_socket
            if server is None:
                break
            try:
                client, _ = server.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._stop_event.is_set():
                    break
                raise

            client.settimeout(0.2)
            with self._client_lock:
                self._client_socket = client
            try:
                self._serve_client(client)
            finally:
                self._close_client(client)
                self._reset_after_disconnect()

    # 处理一个客户端的请求响应循环
    def _serve_client(self, client: socket.socket) -> None:
        while not self._stop_event.is_set():
            try:
                data = client.recv(IPC_MAX_PACKET_SIZE)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break

            request_id = 0
            try:
                request = decode_packet(data)
                request_id = request.request_id
                response_type, response_payload = self._handle_request(request)
            except IPCProtocolError as error:
                response_type = IPCMessageType.ERROR
                response_payload = make_error_payload(
                    "INVALID_REQUEST",
                    str(error),
                )
            except (RuntimeError, ValueError) as error:
                response_type = IPCMessageType.ERROR
                response_payload = make_error_payload(
                    "CONTROL_REJECTED",
                    str(error),
                )
            except Exception as error:
                response_type = IPCMessageType.ERROR
                response_payload = make_error_payload(
                    "INTERNAL_ERROR",
                    str(error),
                )

            try:
                self._send_packet(
                    client,
                    encode_packet(
                        response_type,
                        request_id,
                        response_payload,
                    ),
                )
            except OSError:
                break

    # 将IPC请求转换为控制守护进程调用
    def _handle_request(self, request: Any) -> tuple[IPCMessageType, dict[str, Any]]:
        if request.message_type is IPCMessageType.PING:
            self._require_empty_payload(request.payload)
            return IPCMessageType.PONG, {"server_time_ns": time.monotonic_ns()}

        if request.message_type is IPCMessageType.SUBMIT_SERVOJ:
            sequence, target_q, speed_limits = parse_servoj_payload(
                request.payload
            )
            accepted = self._daemon.submit_servoj(
                sequence,
                target_q,
                speed_limits,
            )
            return IPCMessageType.COMMAND_ACK, {"accepted": bool(accepted)}

        if request.message_type is IPCMessageType.PAUSE:
            self._require_empty_payload(request.payload)
            self._daemon.pause()
            return IPCMessageType.PAUSE_ACK, {"accepted": True}

        if request.message_type is IPCMessageType.RESET_SESSION:
            self._require_empty_payload(request.payload)
            self._daemon.reset_session()
            return IPCMessageType.RESET_SESSION_ACK, {"accepted": True}

        if request.message_type is IPCMessageType.GET_SNAPSHOT:
            self._require_empty_payload(request.payload)
            return (
                IPCMessageType.SNAPSHOT,
                snapshot_to_payload(self._daemon.get_snapshot()),
            )

        raise IPCProtocolError(
            f"message type {request.message_type.name} is not a request"
        )

    # 检查请求是否为空对象
    @staticmethod
    def _require_empty_payload(payload: dict[str, Any]) -> None:
        if payload:
            raise IPCProtocolError("request payload must be empty")

    # 原子发送一个SEQPACKET数据包
    @staticmethod
    def _send_packet(connection: socket.socket, packet: bytes) -> None:
        sent = connection.send(packet)
        if sent != len(packet):
            raise OSError("IPC packet was only partially sent")

    # 在客户端断开后立即保持并重置会话
    def _reset_after_disconnect(self) -> None:
        try:
            if self._daemon.state in (DaemonState.IDLE, DaemonState.RUNNING):
                self._daemon.reset_session()
        except Exception:
            pass

    # 关闭当前客户端Socket
    def _close_client(self, expected: socket.socket | None = None) -> None:
        with self._client_lock:
            client = self._client_socket
            if expected is not None and client is not expected:
                return
            self._client_socket = None
        if client is not None:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            client.close()

    # 检查并清理失效的Socket路径
    def _prepare_socket_path(self) -> None:
        parent = self._socket_path.parent
        parent.mkdir(mode=0o750, parents=True, exist_ok=True)

        try:
            path_stat = self._socket_path.lstat()
        except FileNotFoundError:
            return
        if not stat.S_ISSOCK(path_stat.st_mode):
            raise RuntimeError(
                f"IPC path exists and is not a socket: {self._socket_path}"
            )
        self._socket_path.unlink(missing_ok=True)

    # 获取单实例文件锁并安全识别陈旧Socket
    def _acquire_instance_lock(self) -> None:
        self._socket_path.parent.mkdir(mode=0o750, parents=True, exist_ok=True)
        lock_fd = os.open(self._lock_path, os.O_RDWR | os.O_CREAT, 0o600)
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            os.close(lock_fd)
            raise RuntimeError(
                f"IPC server is already active: {self._socket_path}"
            ) from error
        self._lock_fd = lock_fd

    # 释放单实例文件锁
    def _release_instance_lock(self) -> None:
        lock_fd = self._lock_fd
        self._lock_fd = None
        if lock_fd is None:
            return
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
        finally:
            os.close(lock_fd)

    # 仅删除当前路径上的Socket节点
    def _remove_owned_socket_path(self) -> None:
        if not self._owns_socket_path:
            return
        self._owns_socket_path = False
        try:
            path_stat = self._socket_path.lstat()
        except FileNotFoundError:
            return
        if stat.S_ISSOCK(path_stat.st_mode):
            self._socket_path.unlink(missing_ok=True)


# 启动机械臂控制守护进程及其IPC服务
def main() -> None:
    parser = argparse.ArgumentParser(description="reBot control daemon IPC server")
    parser.add_argument(
        "--socket-path",
        default=DEFAULT_IPC_SOCKET_PATH,
        help="Unix Domain Socket path",
    )
    arguments = parser.parse_args()

    daemon = ControlDaemon()
    server = ControlIPCServer(daemon, arguments.socket_path)

    try:
        daemon.start()
        server.start()
        print(f"控制守护进程IPC已启动: {arguments.socket_path}")
        print("按 Ctrl+C 停止")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n控制守护进程IPC正在停止")
    finally:
        server.stop()
        print("请扶稳机械臂，即将失能")
        daemon.shutdown()


if __name__ == "__main__":
    main()
