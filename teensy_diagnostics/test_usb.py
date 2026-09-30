"""Exercise the diagnostic firmware; never sends motor commands."""
import glob
import json
import statistics
import time
from pathlib import Path
import serial

ports = glob.glob('/dev/serial/by-id/*Teensy*')
if len(ports) != 1:
    raise SystemExit(f'Expected exactly one Teensy serial device; found {ports}')
port = ports[0]
results = {'port': port, 'tests': {}, 'scope': 'USB, CPU execution, 64 KiB scratch RAM, 1 kHz timer. No external pins tested.'}

def transact(s, command):
    s.write(command.encode() + b'\n')
    reply = s.readline().decode().rstrip('\r\n')
    if not reply:
        raise RuntimeError(f'Timeout for {command[:32]}')
    return reply

def status(s):
    reply = transact(s, 'STATUS')
    assert reply.startswith('STATUS '), reply
    return {k: int(v) for k, v in (field.split('=') for field in reply.split()[1:])}

with serial.Serial(port, 115200, timeout=2, write_timeout=2) as s:
    s.reset_input_buffer()
    assert transact(s, 'PING') == 'READY SIX_DOF_TEENSY40_DIAG_V1'
    results['tests']['handshake'] = 'PASS'
    ram = transact(s, 'RAMTEST')
    assert ram == 'RAMTEST bytes=65536 passes=4 errors=0', ram
    results['tests']['ram'] = ram
    a = status(s)
    time.sleep(2)
    b = status(s)
    assert a['timer_ok'] == b['timer_ok'] == 1
    elapsed = b['uptime_ms'] - a['uptime_ms']
    ticks = b['ticks'] - a['ticks']
    assert elapsed >= 1900 and abs(ticks - elapsed) <= 5, (a, b)
    results['tests']['timer'] = {'elapsed_ms': elapsed, 'ticks': ticks, 'cpu_hz': b['cpu_hz']}
    latencies = []
    for i in range(2000):
        payload = 'ECHO ' + str(i) + ' ' + ''.join(chr(33 + (i + j) % 90) for j in range(i % 220))
        start = time.perf_counter()
        assert transact(s, payload) == payload, f'Echo mismatch at {i}'
        latencies.append((time.perf_counter() - start) * 1000)
    results['tests']['echo'] = {'count': len(latencies), 'mismatches': 0,
        'median_ms': statistics.median(latencies), 'max_ms': max(latencies)}
    assert transact(s, 'INVALID') == 'ERROR unknown_command'
    assert transact(s, 'X' * 400) == 'ERROR line_too_long'
    assert transact(s, 'PING') == 'READY SIX_DOF_TEENSY40_DIAG_V1'
    s.write(b'ECHO frag')
    s.flush()
    time.sleep(.02)
    s.write(b'mented\nPING\n')
    assert s.readline().strip() == b'ECHO fragmented'
    assert s.readline().strip() == b'READY SIX_DOF_TEENSY40_DIAG_V1'
    results['tests']['parser_recovery_and_framing'] = 'PASS'
for _ in range(10):
    with serial.Serial(port, 115200, timeout=2, write_timeout=2) as s:
        assert transact(s, 'PING') == 'READY SIX_DOF_TEENSY40_DIAG_V1'
results['tests']['port_reopen'] = '10/10 PASS (not a physical reconnect test)'
results['result'] = 'PASS'
Path(__file__).with_name('test_results.json').write_text(json.dumps(results, indent=2) + '\n')
print(json.dumps(results, indent=2))
