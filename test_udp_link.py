#!/usr/bin/env python3
"""
Simple UDP Connectivity Tester between Laptop (WSL2) and Raspberry Pi
======================================================================
Usage:
  On Laptop (Listen mode):
    python3 test_udp_link.py listen

  On Pi (Send test ping):
    python3 test_udp_link.py ping 10.73.81.140
"""

import sys
import socket
import time

TEST_PORT = 7410

def run_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', TEST_PORT))
    print(f"==================================================================")
    print(f"  [LISTENER] Waiting for UDP packet on 0.0.0.0:{TEST_PORT}...")
    print(f"==================================================================")
    sock.settimeout(20.0)
    try:
        data, addr = sock.recvfrom(2048)
        print(f"✅ SUCCESS! Received {len(data)} bytes from {addr}: {data.decode('utf-8', errors='replace')}")
        # Reply back
        sock.sendto(b"ACK_FROM_LAPTOP", addr)
        print(f"   Sent ACK reply back to {addr}")
    except socket.timeout:
        print("❌ TIMEOUT: No UDP packets arrived within 20 seconds.")
        print("   -> Windows Defender Firewall or router client isolation is blocking incoming UDP!")
    finally:
        sock.close()

def run_ping(target_ip):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3.0)
    message = b"PING_FROM_PI"
    print(f"==================================================================")
    print(f"  [SENDER] Sending UDP packet to {target_ip}:{TEST_PORT}...")
    print(f"==================================================================")
    sock.sendto(message, (target_ip, TEST_PORT))
    try:
        reply, addr = sock.recvfrom(2048)
        print(f"✅ SUCCESS! Received reply from {addr}: {reply.decode('utf-8', errors='replace')}")
        print("   Two-way UDP communication between Pi and Laptop is 100% OPERATIONAL!")
    except socket.timeout:
        print(f"⚠️  Packet sent to {target_ip}:{TEST_PORT}, but no reply received.")
        print("   (Either laptop listener wasn't running, or firewall blocked the packet).")
    finally:
        sock.close()

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'ping':
        target = sys.argv[2] if len(sys.argv) > 2 else '10.73.81.140'
        run_ping(target)
    else:
        run_listener()

