import queue
import socket
import struct
import time

import numpy as np
import threading


class UDPHandler:
    def __init__(self, remote_ip_addr, remote_port, timeout_sec=0.25):
        self.ip = remote_ip_addr
        self.port = remote_port
        self.addr = (self.ip, self.port)
        self.timeout = timeout_sec

        self.data_queue = queue.Queue()
        self.thread = threading.Thread(target=self.__send_handler__)
        self.mutex = threading.Lock()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.is_running = False

    def __del__(self):
        self.close()

    def add_to_queue(self, data: float):
        try:
            self.data_queue.put(data, block=False, timeout=None)
        except queue.Full:
            print("Queue is full")

    def __send_from_queue(self, block=False, timeout=1) -> bool:
        try:
            data = self.data_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return False

        success = self.send(data=data)
        if success:
            self.data_queue.task_done()
        return success

    def __send_handler__(self):
        while self.is_running:
            success = self.__send_from_queue(block=True)
            time.sleep(0.005)    # Sleep for 5 ms to stabilize and avoid loosing packets

    def send(self, data: float or None = None) -> bool:
        if data is None:
            return False

        data = bytearray(struct.pack("f", data))
        size = self.sock.sendto(data, self.addr)
        return size == len(data)

    def start(self):
        self.is_running = True
        self.thread.start()

    def join(self):
        self.is_running = False
        if self.thread.is_alive():
            self.thread.join()

    def close(self):
        self.sock.close()
        self.join()
