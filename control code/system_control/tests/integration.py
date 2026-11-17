"""Real executable + fake serial devices over PTYs. Never opens physical devices."""
import csv
import errno
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tty
import unittest

BINARY = str(Path(sys.argv.pop(1)).resolve())


class Device:
    def __init__(self, kind):
        self.kind = kind
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.events = []
        self.running = True
        self.drop = False
        self.short_reply = False
        self.fault = 0
        self.position = {1: 1000, 2: 1000, 3: 1000}
        self.speed = 0
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def response(self, data):
        if self.drop:
            return
        # Split valid replies, including checksums, across reads.
        os.write(self.master, data[:3])
        time.sleep(0.001)
        if not self.short_reply:
            os.write(self.master, data[3:])

    def cylinder(self, packet):
        if sum(packet[2:-1]) & 255 != packet[-1]:
            raise AssertionError("host sent a bad checksum")
        device, op = packet[3:5]
        address = int.from_bytes(packet[5:7], "little") if op == 0x32 else 0
        self.events.append((time.monotonic(), device, op, address, packet))
        if op == 0x32 and address == 0x25:
            self.position[device] = int.from_bytes(packet[15:17], "little")
        position = self.position[device]
        data = bytearray((0xAA, 0x55, 15, device, op))
        data += struct.pack("<HhhHhHbB", address, position, position, 0, 0, 0, 25, self.fault)
        data.append(sum(data[2:]) & 255)
        self.response(data)

    def line(self, command):
        self.events.append((time.monotonic(), command))
        if command == "INFO":
            reply = "INFO PCB1_S3 1.0" if self.kind == "pcb" else "ERR COMMAND"
        elif command == "?":
            reply = f"V{self.speed}"
        elif command.startswith("V"):
            self.speed = int(command[1:]); reply = f"OK {command}"
        elif command in ("A1", "A0", "H"):
            reply = f"OK {command}"
        else:
            reply = "ERR COMMAND"
        self.response((reply + "\n").encode())

    def loop(self):
        pending = bytearray()
        while self.running:
            try:
                if not select.select([self.master], [], [], 0.02)[0]:
                    continue
                chunk = os.read(self.master, 512)
                if not chunk:
                    continue
                pending.extend(chunk)
                if self.kind != "cylinder":
                    while b"\n" in pending:
                        line, _, pending = pending.partition(b"\n")
                        self.line(line.decode().strip())
                else:
                    while len(pending) >= 3:
                        if pending[:2] != b"\x55\xaa":
                            del pending[0]; continue
                        length = pending[2] + 5
                        if len(pending) < length:
                            break
                        packet = bytes(pending[:length]); del pending[:length]
                        self.cylinder(packet)
            except OSError as e:
                if e.errno not in (errno.EIO, errno.EBADF):
                    raise
                time.sleep(0.01)

    def close(self):
        self.running = False
        self.thread.join(timeout=1)
        os.close(self.master); os.close(self.slave)


class Integration(unittest.TestCase):
    def test_demo_and_recording(self):
        with tempfile.TemporaryDirectory() as d:
            csvpath = Path(d) / "demo.csv"
            for backend in ("nano", "pcb", "none"):
                path = csvpath.with_name(backend + ".csv")
                p = subprocess.run([BINARY, "--mode", "demo", "--propulsion", backend,
                                    "--record", str(path)], capture_output=True, text=True, timeout=8)
                self.assertEqual(p.returncode, 0, p.stderr)
                with path.open() as f:
                    rows = list(csv.DictReader(f))
                if backend == "none":
                    self.assertTrue(all(r["propulsion_command_sps"] == "0" and r["propulsion_ready"] == "0" for r in rows))
                else:
                    self.assertTrue(any(int(r["propulsion_command_sps"]) > 0 for r in rows))
                    self.assertTrue(any(int(r["propulsion_command_sps"]) < 0 for r in rows))
                self.assertTrue(any(float(r["cylinder1_mm"]) > 8 for r in rows))
                self.assertTrue(all(r["simulation"] == "1" for r in rows))
                self.assertEqual(rows[-1]["active"], "0")
                original = path.read_bytes()
                p = subprocess.run([BINARY, "--mode", "demo", "--record", str(path)], capture_output=True, timeout=2)
                self.assertNotEqual(p.returncode, 0)
                self.assertEqual(path.read_bytes(), original)

    def run_hardware(self, backend, failure=None):
        propulsion, cylinders = Device(backend), Device("cylinder")
        # Nano may already have a nonzero speed: query must still be followed by V0.
        if backend == "nano":
            propulsion.speed = 640
        master, slave = pty.openpty()
        process = None
        try:
            with tempfile.TemporaryDirectory() as d:
                cfg = Path(d) / "hardware.conf"
                propulsion_config = f"propulsion={backend}\n"
                if backend != "none":
                    propulsion_config += f"propulsion_port={propulsion.path}\n"
                cfg.write_text(propulsion_config + f"cylinder_port={cylinders.path}\n"
                               "cylinder_baud=115200\ncylinder1=1,16,0,16,0.5\ncylinder2=2,16,0,16,0.5\ncylinder3=3,16,0,16,0.5\n")
                process = subprocess.Popen([BINARY, "--config", str(cfg), "--duration", "7"],
                                           stdin=slave, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                # Enable is intentionally ignored before readiness. Wait for the
                # actual state instead of racing startup with a fixed delay.
                prefix = b""
                deadline = time.monotonic() + 5
                while b"ready=1" not in prefix and time.monotonic() < deadline:
                    if select.select([process.stdout], [], [], 0.05)[0]:
                        prefix += os.read(process.stdout.fileno(), 4096)
                    if process.poll() is not None:
                        break
                self.assertIn(b"ready=1", prefix, "controller did not become ready: " + prefix.decode())
                self.assertFalse(any(e[2] == 0x32 for e in cylinders.events), "startup must only read cylinders")
                self.assertFalse(any(e[1] == "A1" for e in propulsion.events), "startup must not arm PCB")
                os.write(master, b"e"); time.sleep(0.06)
                for _ in range(15):
                    os.write(master, b"d"); time.sleep(0.03)
                if backend != "none":
                    self.assertTrue(any(e[1].startswith("V") and e[1] not in ("V0",) for e in propulsion.events))
                for key in (b"1", b"3", b"5"):
                    for _ in range(12):
                        os.write(master, key); time.sleep(0.03)
                self.assertEqual({e[1] for e in cylinders.events if e[2:4] == (0x32, 0x25)}, {1, 2, 3})
                self.assertTrue(any(e[2:4] == (0x32, 0x25) for e in cylinders.events))
                if failure:
                    if failure == "fault":
                        cylinders.fault = 2
                    else:
                        cylinders.drop = True
                    # This device may fail while propulsion still receives input.
                    for _ in range(12):
                        os.write(master, b"d"); time.sleep(0.03)
                    out, err = process.communicate(timeout=4)
                    self.assertNotEqual(process.returncode, 0, out + err)
                    self.assertIn("fault=2" if failure == "fault" else "timeout", err)
                else:
                    os.write(master, b"x1"); time.sleep(0.25)
                    writes = sum(e[2:4] == (0x32, 0x25) for e in cylinders.events)
                    os.write(master, b"1"); time.sleep(0.1)
                    self.assertEqual(writes, sum(e[2:4] == (0x32, 0x25) for e in cylinders.events))
                    self.assertEqual(propulsion.speed, 0, "latched input stop must require enable")
                    os.write(master, b"q")
                    out, err = process.communicate(timeout=4)
                    self.assertEqual(process.returncode, 0, out + err)
                self.assertEqual(propulsion.speed, 0)
                if backend == "none":
                    self.assertEqual(propulsion.events, [])
                if backend == "pcb":
                    commands = [e[1] for e in propulsion.events]
                    self.assertIn("A1", commands)
                    self.assertEqual(commands[-1], "A0")
                self.assertTrue(any(e[2:4] == (0x32, 0x1A) for e in cylinders.events))
        finally:
            if process and process.poll() is None:
                process.terminate(); process.communicate(timeout=4)
            os.close(master); os.close(slave)
            propulsion.close(); cylinders.close()

    def test_timeout_diagnostics(self):
        cylinders = Device("cylinder")
        try:
            with tempfile.TemporaryDirectory() as d:
                cfg = Path(d) / "inspect.conf"
                cfg.write_text(f"propulsion=none\ncylinder_port={cylinders.path}\ncylinder_baud=115200\n"
                               "cylinder1=1,16,0,16,0.5\ncylinder2=2,16,0,16,0.5\ncylinder3=3,16,0,16,0.5\n")
                for short in (False, True):
                    cylinders.drop = not short
                    cylinders.short_reply = short
                    result = subprocess.run([BINARY, "--mode", "inspect", "--config", str(cfg)],
                                            capture_output=True, text=True, timeout=3)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("received_bytes=3 reason=short_reply" if short else "received_bytes=0 reason=no_bytes", result.stdout)
                    self.assertIn("op=0x30", result.stdout)
        finally:
            cylinders.close()

    def test_zero_feedback_tolerance(self):
        for raw2 in (-5, -7):
            cylinders = Device("cylinder")
            cylinders.position = {1: 20, 2: raw2, 3: -2}
            master, slave = pty.openpty()
            try:
                with tempfile.TemporaryDirectory() as d:
                    cfg = Path(d) / "zero-offset.conf"
                    cfg.write_text(f"propulsion=none\nauto_zero=true\nzero_tolerance_mm=0.05\ncylinder_port={cylinders.path}\ncylinder_baud=115200\n"
                                   "cylinder1=1,16,0,16,0.5\ncylinder2=2,16,0,16,0.5\ncylinder3=3,16,0,16,0.5\n")
                    result = subprocess.run([BINARY, "--mode", "command", "--config", str(cfg), "--duration", "2"],
                                            stdin=slave, capture_output=True, text=True, timeout=4)
                    writes = [e for e in cylinders.events if e[2:4] == (0x32, 0x25)]
                    if raw2 == -5:
                        self.assertEqual(result.returncode, 0, result.stderr)
                        self.assertIn("Auto-zero complete", result.stdout)
                        self.assertLessEqual(cylinders.position[1] * 16 / 2000, 0.05)
                        self.assertEqual(cylinders.position[2], -5)
                        self.assertEqual(cylinders.position[3], -2)
                        self.assertTrue(writes)
                        self.assertTrue(all(0 <= int.from_bytes(e[4][15:17], "little", signed=True) <= 2000 for e in writes))
                    else:
                        self.assertNotEqual(result.returncode, 0)
                        self.assertIn("raw=-7", result.stderr)
                        self.assertFalse(writes)
            finally:
                os.close(master); os.close(slave); cylinders.close()

    def test_read_only_inspection_out_of_range(self):
        cylinders = Device("cylinder")
        cylinders.position = {1: 1000, 2: -1, 3: 2001}
        try:
            with tempfile.TemporaryDirectory() as d:
                cfg = Path(d) / "inspect.conf"
                cfg.write_text(f"propulsion=none\nauto_zero=true\ncylinder_port={cylinders.path}\ncylinder_baud=115200\n"
                               "cylinder1=1,16,0,16,0.5\ncylinder2=2,16,0,16,0.5\ncylinder3=3,16,0,16,0.5\n")
                result = subprocess.run([BINARY, "--mode", "inspect", "--config", str(cfg)],
                                        capture_output=True, text=True, timeout=3)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn("ID=2 raw_position=-1", result.stdout)
                self.assertIn("ID=3 raw_position=2001", result.stdout)
                self.assertEqual({e[1] for e in cylinders.events}, {1, 2, 3})
                self.assertTrue(all(e[2] == 0x30 for e in cylinders.events))
        finally:
            cylinders.close()

    def test_serial_discovery(self):
        good, silent, other = Device("cylinder"), Device("cylinder"), Device("cylinder")
        silent.drop = True
        probe = str(Path(BINARY).with_name("discovery_test"))
        try:
            result = subprocess.run([probe, silent.path, good.path, good.path], capture_output=True, text=True, timeout=3)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.strip(), good.path)
            self.assertEqual({e[1] for e in good.events}, {1, 2, 3})
            self.assertTrue(all(e[2] == 0x30 for e in good.events + silent.events))
            result = subprocess.run([probe, good.path, other.path], capture_output=True, text=True, timeout=3)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("multiple cylinder buses", result.stderr)
            self.assertIn(good.path, result.stderr)
            self.assertIn(other.path, result.stderr)
            result = subprocess.run([probe, silent.path], capture_output=True, text=True, timeout=3)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("no USB serial port replied", result.stderr)
            self.assertTrue(all(e[2] == 0x30 for e in good.events + silent.events + other.events))
        finally:
            good.close(); silent.close(); other.close()

    def test_position_commands(self):
        cylinders = Device("cylinder")
        master, slave = pty.openpty()
        process = None
        try:
            with tempfile.TemporaryDirectory() as d:
                cfg = Path(d) / "position.conf"
                cfg.write_text(f"propulsion=none\ncylinder_port={cylinders.path}\ncylinder_baud=115200\n"
                               "cylinder1=1,16,0,16,1\ncylinder2=2,16,0,16,1\ncylinder3=3,16,0,16,1\n")
                process = subprocess.Popen([BINARY, "--mode", "command", "--config", str(cfg), "--duration", "15"],
                                           stdin=slave, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                output = bytearray()
                def wait_text(text):
                    deadline = time.monotonic() + 4
                    while text.encode() not in output and time.monotonic() < deadline:
                        if select.select([process.stdout], [], [], 0.05)[0]:
                            output.extend(os.read(process.stdout.fileno(), 4096))
                    self.assertIn(text.encode(), output)
                    output.clear()
                def send(text):
                    os.write(master, text.encode())
                    time.sleep(0.08)
                wait_text("ready=1")
                send("move 8.1 8.2 7.9\n")
                wait_text("Rejected:")
                self.assertFalse(any(e[2] == 0x32 for e in cylinders.events))
                send("enable\n")
                send("move 8.1 8.2 7.9\n")
                wait_text("Reached target")
                self.assertEqual(cylinders.position, {1: 1013, 2: 1025, 3: 988})
                # Replace a moving goal with a nearer goal in both directions.
                send("move 12 12 12\n")
                send("move 8 8 8\n")
                wait_text("Reached target")
                self.assertEqual(cylinders.position, {1: 1000, 2: 1000, 3: 1000})
                send("move 12 12 12\n")
                send("move 8 17 8\n")
                wait_text("outside limits")
                time.sleep(0.2)
                count = len([e for e in cylinders.events if e[2:4] == (0x32, 0x25)])
                send("move 9 9 9\n")
                time.sleep(0.2)
                self.assertEqual(count, len([e for e in cylinders.events if e[2:4] == (0x32, 0x25)]))
                send("enable\n")
                send("move 12 12 12\n")
                send("\x1b")
                time.sleep(0.2)
                count = len([e for e in cylinders.events if e[2:4] == (0x32, 0x25)])
                send("move 9 9 9\n")
                time.sleep(0.2)
                self.assertEqual(count, len([e for e in cylinders.events if e[2:4] == (0x32, 0x25)]))
                send("quit\n")
                out, err = process.communicate(timeout=3)
                self.assertEqual(process.returncode, 0, out + err)
                self.assertTrue(any(e[2:4] == (0x32, 0x1A) for e in cylinders.events))
        finally:
            if process and process.poll() is None:
                process.terminate(); process.communicate(timeout=4)
            os.close(master); os.close(slave); cylinders.close()

    def run_auto_zero(self, action=None):
        cylinders = Device("cylinder")
        cylinders.position = {1: 100 if action else 15, 2: 100 if action else 20, 3: 100 if action else 25}
        master, slave = pty.openpty()
        process = None
        try:
            with tempfile.TemporaryDirectory() as d:
                cfg = Path(d) / "zero.conf"
                cfg.write_text(f"propulsion=none\nauto_zero=true\ncylinder_port={cylinders.path}\n"
                               "cylinder_baud=115200\ncylinder1=1,16,0,16,0.5\ncylinder2=2,16,0,16,0.5\ncylinder3=3,16,0,16,0.5\n")
                process = subprocess.Popen([BINARY, "--config", str(cfg), "--duration", "5"],
                                           stdin=slave, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                deadline = time.monotonic() + 3
                while not any(e[2:4] == (0x32, 0x25) for e in cylinders.events) and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertTrue(any(e[2:4] == (0x32, 0x25) for e in cylinders.events))
                # All axes must have been read before any movement.
                first_write = next(i for i, e in enumerate(cylinders.events) if e[2:4] == (0x32, 0x25))
                self.assertEqual({e[1] for e in cylinders.events[:first_write] if e[2] == 0x30}, {1, 2, 3})
                if action == "stop":
                    os.write(master, b"x")
                    time.sleep(0.3)
                    count = sum(e[2:4] == (0x32, 0x25) for e in cylinders.events)
                    time.sleep(0.3)
                    self.assertEqual(count, sum(e[2:4] == (0x32, 0x25) for e in cylinders.events))
                    self.assertTrue(any(v > 0 for v in cylinders.position.values()))
                    os.write(master, b"q")
                elif action == "fault":
                    cylinders.fault = 2
                out, err = process.communicate(timeout=7)
                if action == "fault":
                    self.assertNotEqual(process.returncode, 0)
                    self.assertIn("fault=2", err)
                else:
                    self.assertEqual(process.returncode, 0, out + err)
                    self.assertIn("Auto-zero cancelled" if action else "Auto-zero complete", out)
                    if not action:
                        self.assertEqual(list(cylinders.position.values()), [0, 0, 0])
                self.assertTrue(any(e[2:4] == (0x32, 0x1A) for e in cylinders.events))
        finally:
            if process and process.poll() is None:
                process.terminate(); process.communicate(timeout=4)
            os.close(master); os.close(slave); cylinders.close()

    def test_auto_zero(self):
        self.run_auto_zero()

    def test_auto_zero_cancel(self):
        self.run_auto_zero("stop")

    def test_auto_zero_fault(self):
        self.run_auto_zero("fault")

    def test_cylinders_only(self):
        self.run_hardware("none")

    def test_cylinders_only_fault(self):
        self.run_hardware("none", failure="fault")

    def test_cylinders_only_lost_replies(self):
        self.run_hardware("none", failure="drop")

    def test_nano_and_three_cylinders(self):
        self.run_hardware("nano")

    def test_pcb_and_three_cylinders(self):
        self.run_hardware("pcb")

    def test_cylinder_fault_stops_propulsion(self):
        self.run_hardware("pcb", failure="fault")

    def test_lost_cylinder_replies_stop_propulsion(self):
        self.run_hardware("pcb", failure="drop")

    def test_invalid_config_opens_no_devices(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "bad.conf"
            path.write_text("propulsion=nano\n")
            p = subprocess.run([BINARY, "--config", str(path)], capture_output=True, text=True, timeout=2)
            self.assertNotEqual(p.returncode, 0)
            self.assertIn("missing configuration", p.stderr)
            p = subprocess.run([BINARY, "--mode", "demo", "--config", str(path)], capture_output=True, text=True, timeout=2)
            self.assertIn("demo does not accept hardware", p.stderr)


if __name__ == "__main__":
    unittest.main()
