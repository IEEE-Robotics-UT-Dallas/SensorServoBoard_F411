"""
SensorServoBoard F411 — Terminal Debug Dashboard
GDB Python extension for firmware inspection via ST-Link + OpenOCD

Usage (from GDB prompt):
  ssb              System overview
  ssb sensors      Sensor readings (ToF, Mag, Light)
  ssb servos       Servo PWM state from timer CCR registers
  ssb memory       FreeRTOS heap + micro-ROS allocation tracking
  ssb tasks        FreeRTOS task info
  ssb i2c          I2C bus status registers
  ssb gpio         XSHUT pin states
  ssb health       Automated health checks (pass/fail)
  ssb watch [sec]  Live monitoring (halt/sample/resume loop, Ctrl+C to stop)
  ssb help         This message
"""

import gdb
import struct
import time

# ── STM32F411CEU6 Register Map ───────────────────────────────────

# Timer bases
TIM2_BASE  = 0x40000000
TIM3_BASE  = 0x40000400

# Timer register offsets
TIM_CR1  = 0x00
TIM_CNT  = 0x24
TIM_PSC  = 0x28
TIM_ARR  = 0x2C
TIM_CCR1 = 0x34
TIM_CCR2 = 0x38
TIM_CCR3 = 0x3C
TIM_CCR4 = 0x40

# I2C bases
I2C1_BASE = 0x40005400
I2C2_BASE = 0x40005800
I2C3_BASE = 0x40005C00

# I2C register offsets
I2C_CR1  = 0x00
I2C_SR1  = 0x14
I2C_SR2  = 0x18

# GPIO
GPIOB_BASE = 0x40020400
GPIO_ODR   = 0x14

# RCC (for peripheral clock checks)
RCC_BASE    = 0x40023800
RCC_APB1ENR = 0x40
RCC_APB2ENR = 0x44

# ── Hardware Configuration ───────────────────────────────────────

SERVOS = [
    {"id": 1, "pin": "PA1", "ch": "TIM2_CH2", "addr": TIM2_BASE + TIM_CCR2},
    {"id": 2, "pin": "PA2", "ch": "TIM2_CH3", "addr": TIM2_BASE + TIM_CCR3},
    {"id": 3, "pin": "PA3", "ch": "TIM2_CH4", "addr": TIM2_BASE + TIM_CCR4},
    {"id": 4, "pin": "PA5", "ch": "TIM2_CH1", "addr": TIM2_BASE + TIM_CCR1},
    {"id": 5, "pin": "PA6", "ch": "TIM3_CH1", "addr": TIM3_BASE + TIM_CCR1},
]

I2C_BUSES = [
    {"name": "I2C1", "base": I2C1_BASE, "pins": "PB8/PB9",  "devs": "ToF 0, 1"},
    {"name": "I2C2", "base": I2C2_BASE, "pins": "PB10/PB3", "devs": "ToF 2, 3"},
    {"name": "I2C3", "base": I2C3_BASE, "pins": "PA8/PB4",  "devs": "ToF 4, Mag, Light"},
]

XSHUT_PINS = [
    {"name": "ToF 0", "pin": "PB0",  "bit": 0},
    {"name": "ToF 1", "pin": "PB1",  "bit": 1},
    {"name": "ToF 2", "pin": "PB12", "bit": 12},
    {"name": "ToF 3", "pin": "PB13", "bit": 13},
]

FREERTOS_HEAP_SIZE = 30 * 1024
APB1_TIMER_CLK_MHZ = 100  # 100 MHz timer clock

# ── Low-Level Helpers ────────────────────────────────────────────

def rd32(addr):
    mem = gdb.selected_inferior().read_memory(addr, 4)
    return struct.unpack('<I', bytes(mem))[0]

def rd16(addr):
    mem = gdb.selected_inferior().read_memory(addr, 2)
    return struct.unpack('<H', bytes(mem))[0]

def sym(name):
    try:
        return gdb.parse_and_eval(name)
    except gdb.error:
        return None

# ── Formatting ───────────────────────────────────────────────────

RESET = "\033[0m"
BOLD  = "\033[1m"
DIM   = "\033[2m"
GREEN = "\033[32m"
RED   = "\033[31m"
YELLOW= "\033[33m"
CYAN  = "\033[36m"

def hdr(title):
    print(f"\n{BOLD}── {title} {'─' * (52 - len(title))}{RESET}")

def ok(msg):
    print(f"  {GREEN}✓{RESET} {msg}")

def fail(msg):
    print(f"  {RED}✗{RESET} {msg}")

def warn(msg):
    print(f"  {YELLOW}!{RESET} {msg}")

def info(msg):
    print(f"  {msg}")

def bar(val, max_val, width=20):
    if max_val <= 0:
        return DIM + '░' * width + RESET
    n = min(width, max(0, int(val / max_val * width)))
    return GREEN + '█' * n + DIM + '░' * (width - n) + RESET

def pulse_to_angle(pulse_us):
    """Inverse of firmware: pulse = 500 + (angle/180) * 2000"""
    return max(0.0, min(180.0, (pulse_us - 500.0) / 2000.0 * 180.0))

# ── Data Readers ─────────────────────────────────────────────────

def read_sensors():
    data = sym('g_sensor_data')
    if data is None:
        return None
    return {
        'tof': [int(data['tof_distances'][i]) for i in range(5)],
        'mag': {'x': int(data['mag_data']['x']),
                'y': int(data['mag_data']['y']),
                'z': int(data['mag_data']['z'])},
        'light': int(data['light_lux']),
    }

def read_servos():
    results = []
    for s in SERVOS:
        try:
            ccr = rd32(s['addr'])
            results.append({**s, 'ccr': ccr, 'angle': pulse_to_angle(ccr)})
        except Exception:
            results.append({**s, 'ccr': None, 'angle': None})
    return results

def read_heap():
    r = {'total': FREERTOS_HEAP_SIZE}
    v = sym('xFreeBytesRemaining')
    if v is not None:
        r['free'] = int(v)
        r['used'] = FREERTOS_HEAP_SIZE - int(v)
    v = sym('xMinimumEverFreeBytesRemaining')
    if v is not None:
        r['min_free'] = int(v)
    v = sym('absoluteUsedMemory')
    if v is not None:
        r['uros_total'] = int(v)
    v = sym('usedMemory')
    if v is not None:
        r['uros_live'] = int(v)
    return r

def read_i2c():
    results = []
    for bus in I2C_BUSES:
        try:
            cr1 = rd16(bus['base'] + I2C_CR1)
            sr1 = rd16(bus['base'] + I2C_SR1)
            sr2 = rd16(bus['base'] + I2C_SR2)
            results.append({
                'name': bus['name'], 'pins': bus['pins'], 'devs': bus['devs'],
                'pe': bool(cr1 & 1),
                'busy': bool(sr2 & 0x02),
                'berr': bool(sr1 & 0x100),
                'arlo': bool(sr1 & 0x200),
                'af':   bool(sr1 & 0x400),
                'ovr':  bool(sr1 & 0x800),
                'sr1': sr1, 'sr2': sr2,
            })
        except Exception:
            results.append({'name': bus['name'], 'pins': bus['pins'],
                            'devs': bus['devs'], 'error': True})
    return results

def read_gpio_xshut():
    try:
        odr = rd32(GPIOB_BASE + GPIO_ODR)
        return [{'name': p['name'], 'pin': p['pin'],
                 'on': bool(odr & (1 << p['bit']))} for p in XSHUT_PINS]
    except Exception:
        return None

def read_timer_status():
    results = {}
    for name, base in [('TIM2', TIM2_BASE), ('TIM3', TIM3_BASE)]:
        try:
            cr1 = rd32(base + TIM_CR1)
            psc = rd32(base + TIM_PSC)
            arr = rd32(base + TIM_ARR)
            cnt = rd32(base + TIM_CNT)
            tick_us = (psc + 1) / APB1_TIMER_CLK_MHZ
            period_ms = (arr + 1) * tick_us / 1000.0
            freq_hz = 1000.0 / period_ms if period_ms > 0 else 0
            results[name] = {
                'running': bool(cr1 & 1), 'psc': psc, 'arr': arr,
                'cnt': cnt, 'tick_us': tick_us,
                'period_ms': period_ms, 'freq_hz': freq_hz,
            }
        except Exception:
            results[name] = {'error': True}
    return results

# ── Display Functions ────────────────────────────────────────────

def show_sensors():
    hdr("Sensors")
    data = read_sensors()
    if data is None:
        fail("g_sensor_data symbol not found")
        return

    buses = ["I2C1", "I2C1", "I2C2", "I2C2", "I2C3"]
    for i in range(5):
        d = data['tof'][i]
        if d == 0xFFFF:
            info(f"  ToF {i} ({buses[i]}): {RED}ERROR{RESET}  (0xFFFF)")
        elif d == 0:
            info(f"  ToF {i} ({buses[i]}): {DIM}  0 mm{RESET}  (no reading)")
        else:
            info(f"  ToF {i} ({buses[i]}): {d:5d} mm {bar(d, 2000)}")

    m = data['mag']
    info(f"  Mag (MLX90393):  X={m['x']:+6d}  Y={m['y']:+6d}  Z={m['z']:+6d}")
    info(f"  Light (VEML7700): {data['light']} lux")

def show_servos():
    hdr("Servos")
    servos = read_servos()
    timers = read_timer_status()

    for t_name, t in timers.items():
        if t.get('error'):
            warn(f"{t_name}: register read error")
        else:
            state = f"{GREEN}running{RESET}" if t['running'] else f"{RED}stopped{RESET}"
            info(f"  {t_name}: {state}  PSC={t['psc']}  ARR={t['arr']}  "
                 f"{t['freq_hz']:.1f} Hz  ({t['tick_us']:.2f} µs/tick)")

    for s in servos:
        if s['ccr'] is None:
            info(f"  #{s['id']} {s['pin']}/{s['ch']}: {RED}read error{RESET}")
        else:
            ang = s['angle']
            pos_bar = bar(ang, 180.0, 15)
            info(f"  #{s['id']} {s['pin']}/{s['ch']}: {ang:6.1f}° "
                 f"({s['ccr']:5d} µs) {pos_bar}")

def show_memory():
    hdr("Memory")
    h = read_heap()

    if 'free' in h:
        pct = h['used'] / h['total'] * 100
        info(f"  Heap: {h['used']:,}/{h['total']:,} bytes "
             f"({pct:.0f}%) {bar(h['used'], h['total'])}")
    else:
        fail("FreeRTOS heap symbols not found (optimized out?)")

    if 'min_free' in h:
        peak = h['total'] - h['min_free']
        peak_pct = peak / h['total'] * 100
        info(f"  Peak: {peak:,}/{h['total']:,} bytes ({peak_pct:.0f}%)  "
             f"(min free ever: {h['min_free']:,})")

    if 'uros_live' in h:
        info(f"  µROS live: {h['uros_live']:,} bytes")
    if 'uros_total' in h:
        info(f"  µROS cumulative: {h['uros_total']:,} bytes")

def show_tasks():
    hdr("Tasks")
    n = sym('uxCurrentNumberOfTasks')
    if n is not None:
        info(f"  Active tasks: {int(n)}")

    tcb = sym('pxCurrentTCB')
    if tcb is not None and int(tcb) != 0:
        try:
            name = tcb.dereference()['pcTaskName'].string()
            info(f"  Current task: {CYAN}{name}{RESET}")
        except Exception:
            info(f"  Current TCB @ {tcb}")

    try:
        out = gdb.execute('info threads', to_string=True)
        lines = [l.strip() for l in out.strip().split('\n') if l.strip()]
        if len(lines) > 1 or (lines and 'No threads' not in lines[0]):
            info("")
            for line in lines:
                marker = f"{CYAN}▸{RESET}" if line.startswith('*') else " "
                info(f"  {marker} {line}")
    except Exception:
        pass

def show_i2c():
    hdr("I2C Buses")
    buses = read_i2c()
    for b in buses:
        if b.get('error'):
            fail(f"{b['name']} ({b['pins']}): register read error")
            continue

        flags = []
        if b['pe']:
            flags.append(f"{GREEN}EN{RESET}")
        else:
            flags.append(f"{RED}OFF{RESET}")
        if b['busy']:
            flags.append(f"{YELLOW}BUSY{RESET}")
        if b['berr']:
            flags.append(f"{RED}BERR{RESET}")
        if b['arlo']:
            flags.append(f"{RED}ARLO{RESET}")
        if b['af']:
            flags.append(f"{YELLOW}NACK{RESET}")
        if b['ovr']:
            flags.append(f"{RED}OVR{RESET}")

        info(f"  {b['name']} ({b['pins']}) [{' '.join(flags)}]  → {b['devs']}")

def show_gpio():
    hdr("XSHUT Pins (GPIOB)")
    pins = read_gpio_xshut()
    if pins is None:
        fail("Could not read GPIOB ODR")
        return
    for p in pins:
        if p['on']:
            info(f"  {GREEN}●{RESET} {p['name']} ({p['pin']}): "
                 f"{GREEN}HIGH{RESET} — sensor enabled")
        else:
            info(f"  {RED}○{RESET} {p['name']} ({p['pin']}): "
                 f"{RED}LOW{RESET}  — sensor held in reset")

# ── Health Check ─────────────────────────────────────────────────

def run_health():
    hdr("Health Check")
    passed = 0
    failed_n = 0

    # FreeRTOS
    n = sym('uxCurrentNumberOfTasks')
    if n and int(n) > 0:
        ok(f"FreeRTOS running ({int(n)} tasks)")
        passed += 1
    else:
        fail("FreeRTOS: no tasks detected")
        failed_n += 1

    # Mutex
    m = sym('sensor_data_mutex')
    if m and int(m) != 0:
        ok("Sensor data mutex initialized")
        passed += 1
    else:
        fail("Sensor data mutex is NULL")
        failed_n += 1

    # Heap
    h = read_heap()
    if 'free' in h:
        if h['free'] > 2048:
            ok(f"Heap: {h['free']:,} bytes free")
            passed += 1
        elif h['free'] > 512:
            warn(f"Heap low: {h['free']:,} bytes free")
        else:
            fail(f"Heap critical: only {h['free']} bytes free!")
            failed_n += 1

        if 'min_free' in h:
            if h['min_free'] > 1024:
                ok(f"Heap watermark safe ({h['min_free']:,} bytes)")
                passed += 1
            else:
                warn(f"Heap watermark low ({h['min_free']:,} bytes)")
    else:
        fail("Heap symbols not found")
        failed_n += 1

    # Sensors
    data = read_sensors()
    if data:
        active = sum(1 for d in data['tof'] if 0 < d < 0xFFFF)
        errs = sum(1 for d in data['tof'] if d == 0xFFFF)
        if active > 0:
            ok(f"ToF sensors: {active}/5 reporting")
            passed += 1
        elif errs > 0:
            fail(f"ToF sensors: {errs}/5 returning error (0xFFFF)")
            failed_n += 1
        else:
            warn("ToF sensors: all reading 0 (not started or no hardware)")

        if data['light'] > 0:
            ok(f"Light sensor: {data['light']} lux")
            passed += 1
        else:
            warn("Light sensor: reading 0")

    # I2C
    for b in read_i2c():
        if b.get('error'):
            fail(f"{b['name']}: register read error")
            failed_n += 1
        elif b.get('berr') or b.get('arlo'):
            fail(f"{b['name']}: bus error (BERR={b.get('berr')}, ARLO={b.get('arlo')})")
            failed_n += 1
        elif b.get('pe'):
            ok(f"{b['name']}: enabled, clean")
            passed += 1
        else:
            warn(f"{b['name']}: peripheral disabled")

    # Timers
    for name, base in [('TIM2', TIM2_BASE), ('TIM3', TIM3_BASE)]:
        try:
            if rd32(base + TIM_CR1) & 1:
                ok(f"{name}: PWM running")
                passed += 1
            else:
                fail(f"{name}: PWM stopped")
                failed_n += 1
        except Exception:
            fail(f"{name}: register read error")
            failed_n += 1

    color = GREEN if failed_n == 0 else RED
    info(f"\n  {color}{BOLD}{passed} passed, {failed_n} failed{RESET}")

# ── Watch Mode ───────────────────────────────────────────────────

def watch_loop(interval):
    print(f"{BOLD}Live monitoring{RESET} (every {interval:.1f}s) — "
          f"{DIM}Ctrl+C to stop{RESET}\n")
    count = 0
    try:
        while True:
            # Clear screen and move cursor home
            print("\033[2J\033[H", end='', flush=True)
            print(f"{BOLD}SensorServoBoard F411{RESET}  "
                  f"{DIM}sample #{count}{RESET}")

            gdb.execute('monitor halt', to_string=True)
            time.sleep(0.01)

            show_sensors()
            show_servos()
            show_memory()

            gdb.execute('monitor resume', to_string=True)

            info(f"\n  {DIM}[{interval:.1f}s interval — Ctrl+C to stop]{RESET}")
            count += 1
            time.sleep(interval)

    except KeyboardInterrupt:
        print(f"\n{BOLD}Watch stopped.{RESET} Target halted for inspection.")
        try:
            gdb.execute('monitor halt', to_string=True)
        except Exception:
            pass

# ── Main Command ─────────────────────────────────────────────────

HELP_TEXT = f"""{BOLD}
SensorServoBoard F411 — Debug Dashboard{RESET}
{DIM}Terminal-native firmware inspection via ST-Link{RESET}

  {BOLD}ssb{RESET}              System overview (sensors + servos + memory + tasks)
  {BOLD}ssb sensors{RESET}      ToF distances, magnetometer, light sensor
  {BOLD}ssb servos{RESET}       PWM timer state and servo angles
  {BOLD}ssb memory{RESET}       FreeRTOS heap + micro-ROS allocation tracking
  {BOLD}ssb tasks{RESET}        FreeRTOS task list and current task
  {BOLD}ssb i2c{RESET}          I2C bus status (enabled, busy, errors)
  {BOLD}ssb gpio{RESET}         XSHUT pin states (sensor enable/disable)
  {BOLD}ssb health{RESET}       Automated pass/fail checks
  {BOLD}ssb watch{RESET} [sec]  Live monitoring loop (default 1s, Ctrl+C stops)
  {BOLD}ssb help{RESET}         This message
"""

class SSBDashboard(gdb.Command):
    """SensorServoBoard debug dashboard. Type 'ssb help' for usage."""

    def __init__(self):
        super().__init__('ssb', gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        parts = arg.strip().split()
        cmd = parts[0] if parts else ''

        try:
            if cmd == '' or cmd == 'overview':
                show_sensors()
                show_servos()
                show_memory()
                show_tasks()
                print()
            elif cmd == 'sensors':
                show_sensors()
            elif cmd == 'servos':
                show_servos()
            elif cmd in ('memory', 'mem'):
                show_memory()
            elif cmd == 'tasks':
                show_tasks()
            elif cmd == 'i2c':
                show_i2c()
            elif cmd == 'gpio':
                show_gpio()
            elif cmd == 'health':
                run_health()
            elif cmd == 'watch':
                interval = float(parts[1]) if len(parts) > 1 else 1.0
                watch_loop(interval)
            elif cmd == 'help':
                print(HELP_TEXT)
            else:
                print(f"Unknown subcommand: {cmd}")
                print(HELP_TEXT)
        except gdb.MemoryError as e:
            fail(f"Memory read failed: {e}")
            fail("Is the target halted? Try: monitor halt")
        except Exception as e:
            fail(f"Error: {e}")

    def complete(self, text, word):
        cmds = ['sensors', 'servos', 'memory', 'tasks', 'i2c',
                'gpio', 'health', 'watch', 'help']
        return [c for c in cmds if c.startswith(word)]

SSBDashboard()
print(f"{GREEN}✓{RESET} SensorServoBoard debug dashboard loaded — "
      f"type {BOLD}'ssb'{RESET} for overview, {BOLD}'ssb help'{RESET} for commands")
