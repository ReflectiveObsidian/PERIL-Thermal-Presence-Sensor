# Thermal Presence Detection for Home Assistant
# Hardware: ESP32-C3 and MLX90640 thermal camera

from secrets import SSID, PASSWORD, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, DEBUG
from sensor import N_PIXELS, encode_temps, read_sensor, build_ignore_mask
from link import send_scores

import network, socket, json, gc, array, time, os
from machine import I2C, Pin, UART
from mlx90640 import MLX90640, RefreshRate
from neopixel import NeoPixel

# ── Constants ─────────────────────────────────────────────────────
N_SLOTS             = 3       # also fixed in the UART payload (link.py, zigbee/peril.yaml)
MIN_THRESHOLD       = 0.05    # lowest accepted trigger/reset threshold
SENSOR_INTERVAL     = 500     # ms pause between sensor reads; a read itself takes 1.5-2 s,
                              # so scores (and UART updates) arrive roughly every 2 s
SLOW_READ_MS        = 2500    # a sensor read longer than this is logged as a stall
HA_INTERVAL         = 10000   # ms between MQTT state reports; a presence change reports at once
PERF_INTERVAL       = 10000   # collect perf stats + write log every 10 s
BEAT_INTERVAL       = 5000    # ms between heartbeat prints
WIFI_RETRY_INTERVAL = 30000   # ms between connect requests while WiFi is down

CONFIG_FILE  = 'config.json'
SLOT_FILE    = 'slot%d.bin'   # one saved reference frame per slot (float32 × N_PIXELS)
LOG_ENABLED  = False          # perf logging; leave off to save CPU and flash wear
LOG_FILE     = 'perf.log'
LOG_MAX_SIZE = 5000           # bytes; file is reset once it exceeds this (~100 lines)

NOT_FOUND   = b'404 Not Found'
BAD_REQUEST = b'400 Bad Request'

# ── Hardware ──────────────────────────────────────────────────────
_led = NeoPixel(Pin(8), 1)
_led[0] = (0, 0, 0)
_led.write()

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

gc.collect()
i2c = I2C(0, sda=Pin(6), scl=Pin(9), freq=400000)  # corrupt frames are caught by the Ta/VDD gate in read_sensor
mlx = MLX90640(i2c)
mlx.refresh_rate = RefreshRate.REFRESH_2_HZ
_uart = UART(1, baudrate=9600, tx=7, rx=10)

# ── State ─────────────────────────────────────────────────────────
device_name        = 'Peril'
device_hostname    = None                   # from config.json, else derived from the MAC at startup
frame              = array.array('f', (0.0 for _ in range(N_PIXELS)))
labels             = ['Slot %d' % (i + 1) for i in range(N_SLOTS)]
trigger_thresholds = [2.0] * N_SLOTS        # a matched slot stops matching once its score rises above this
reset_thresholds   = [0.5] * N_SLOTS        # ...and matches again once its score falls below this
presence           = False
matched            = []
slot_score         = [None] * N_SLOTS
slot_matched       = [False] * N_SLOTS      # hysteresis state per slot
slot_raw           = [None] * N_SLOTS       # reference image per slot (uint16 encoded temps), None if unset
ignore_zones       = []                     # list of [x1,y1,x2,y2] in sensor coords (col 0-31, row 0-23)
ignore_mask        = None                   # precomputed bytearray(N_PIXELS) derived from ignore_zones
mem_free           = 0
rssi               = -127
valid_px           = N_PIXELS               # count of usable pixels in the last frame
_mqtt_client       = None
_mqtt_fails        = 0                      # cumulative MQTT connect/publish failures (diagnostics)
_ha_prev_presence  = None                   # last presence reported to HA; None forces a report
_wifi_up           = False                  # last WiFi state seen by check_wifi()
_wifi_last_try     = 0                      # ticks_ms of the last connect request

# ── Perf stats ────────────────────────────────────────────────────
def collect_perf():
    global mem_free, rssi
    mem_free = gc.mem_free()
    try:
        rssi = wlan.status('rssi')
    except Exception:
        rssi = -127

def write_perf_log():
    try:
        try:
            if os.stat(LOG_FILE)[6] > LOG_MAX_SIZE:
                os.remove(LOG_FILE)
        except OSError:
            pass
        line = '%d,%d,%d\n' % (time.ticks_ms(), mem_free, rssi)
        with open(LOG_FILE, 'a') as f:
            f.write(line)
    except Exception as e:
        print('perf_log err:', e)

# ── Config and slot storage ───────────────────────────────────────
def set_ignore_zones(zones):
    global ignore_zones, ignore_mask
    ignore_zones = zones
    ignore_mask  = build_ignore_mask(zones) if zones else None

def save_config():
    try:
        with open('config.tmp', 'w') as f:
            json.dump({
                'name':         device_name,
                'hostname':     device_hostname,
                'labels':       labels,
                'trigger':      trigger_thresholds,
                'reset':        reset_thresholds,
                'ignore_zones': ignore_zones,
            }, f)
        os.rename('config.tmp', CONFIG_FILE)
    except Exception as e:
        print('save_config err:', e)

def load_config():
    # Returns True if config.json already held both a device name and a hostname.
    global device_name, device_hostname
    try:
        with open(CONFIG_FILE) as f:
            cfg = json.load(f)
        device_name           = cfg.get('name',     device_name)
        device_hostname       = cfg.get('hostname', device_hostname)
        labels[:]             = cfg.get('labels',   labels)
        trigger_thresholds[:] = cfg.get('trigger',  trigger_thresholds)
        reset_thresholds[:]   = cfg.get('reset',    reset_thresholds)
        set_ignore_zones(cfg.get('ignore_zones', []))
        print('Config loaded')
        return 'name' in cfg and device_hostname is not None
    except OSError:
        pass  # no config yet — first boot
    except Exception as e:
        print('load_config err:', e)
    return False

def read_slot_file(idx):
    # Returns the saved reference frame as a float array.
    # Raises OSError if the slot has never been saved.
    buf = bytearray(N_PIXELS * 4)
    with open(SLOT_FILE % idx, 'rb') as f:
        f.readinto(buf)
    return array.array('f', buf)

def save_slot(idx, raw_frame):
    try:
        tmp = array.array('f', raw_frame)
        with open('slot%d.tmp' % idx, 'wb') as f:
            f.write(tmp)
        del tmp
        os.rename('slot%d.tmp' % idx, SLOT_FILE % idx)
        print('Wrote', SLOT_FILE % idx)
    except Exception as e:
        print('save_slot err:', e)

def load_slots():
    for i in range(N_SLOTS):
        try:
            slot_raw[i] = encode_temps(read_slot_file(i))
            print('Loaded slot', i)
        except OSError:
            pass  # slot not yet saved
        except Exception as e:
            print('load_slot%d err:' % i, e)

# ── WiFi ──────────────────────────────────────────────────────────
# WiFi is optional: the sensor and the UART link to the Zigbee MCU keep running
# without it, so nothing here waits for a connection.
def wifi_connect():
    global _wifi_last_try
    _wifi_last_try = time.ticks_ms()
    try:
        wlan.connect(SSID, PASSWORD)
    except OSError as e:
        print('WiFi connect err:', e)

def check_wifi():
    # Called every loop: logs connection changes and repeats the connect
    # request every WIFI_RETRY_INTERVAL while WiFi is down.
    global _wifi_up, _ha_prev_presence
    up = wlan.isconnected()
    if up != _wifi_up:
        _wifi_up = up
        if up:
            print('WiFi connected: http://' + wlan.ifconfig()[0])
            _ha_prev_presence = None  # report to HA straight away
        else:
            print('WiFi lost')
    if not up and time.ticks_diff(time.ticks_ms(), _wifi_last_try) >= WIFI_RETRY_INTERVAL:
        print('WiFi down, retrying...')
        wifi_connect()

# ── HTTP server ───────────────────────────────────────────────────
def reply(conn, body=b'', ctype=None, status=b'200 OK'):
    head = b'HTTP/1.0 ' + status + b'\r\n'
    if ctype:
        head += b'Content-Type: ' + ctype + b'\r\n'
    conn.send(head + b'\r\n' + body)

def send_html(conn):
    reply(conn, ctype=b'text/html')
    try:
        with open('index.html', 'rb') as f:
            while True:
                chunk = f.read(512)
                if not chunk:
                    break
                conn.send(chunk)
                gc.collect()
    except OSError:
        conn.send(b'<h1>index.html missing - upload it to the device</h1>')

def send_csv(conn, values):
    reply(conn, ctype=b'text/plain')
    data = ','.join('%.2f' % v for v in values).encode()
    for i in range(0, len(data), 512):
        conn.send(data[i:i+512])

def slot_index(path):
    # Slot number from the last path segment ('/save/1' -> 1), or None if it
    # is not a valid slot.
    try:
        idx = int(path[path.rfind('/') + 1:])
    except ValueError:
        return None
    return idx if 0 <= idx < N_SLOTS else None

def recv_request(conn):
    raw = b''
    conn.settimeout(1)
    try:
        while b'\r\n\r\n' not in raw:
            chunk = conn.recv(256)
            if not chunk:
                break
            raw += chunk
    except OSError:
        pass
    if b'\r\n\r\n' not in raw:
        return '', ''
    headers_raw, body = raw.split(b'\r\n\r\n', 1)
    cl = 0
    for line in headers_raw.split(b'\r\n'):
        if line.lower().startswith(b'content-length:'):
            try:
                cl = int(line.split(b':', 1)[1].strip())
            except ValueError:
                pass
    try:
        while len(body) < cl:
            chunk = conn.recv(min(256, cl - len(body)))
            if not chunk:
                break
            body += chunk
    except OSError:
        pass
    return headers_raw.decode('utf-8', 'ignore'), body.decode('utf-8', 'ignore')

def handle(conn):
    try:
        headers, body = recv_request(conn)
        first = headers.split('\r\n')[0].split()
        if len(first) < 2:
            return
        method, path = first[0], first[1]

        if path == '/':
            send_html(conn)

        elif path == '/status':
            reply(conn, json.dumps({
                'presence':           presence,
                'matched':            matched,
                'any_slot':           any(r is not None for r in slot_raw),
                'device_name':        device_name,
                'labels':             labels,
                'trigger_thresholds': trigger_thresholds,
                'reset_thresholds':   reset_thresholds,
                'slot_score':         slot_score,
                'ignore_zones':       ignore_zones,
                'mem_free':           mem_free,
                'rssi':               rssi,
                'valid_px':           valid_px,
            }).encode(), b'application/json')

        elif path == '/debug':
            reply(conn, b'{"debug":true}' if DEBUG else b'{"debug":false}', b'application/json')

        elif path.startswith('/save/') and method == 'POST':
            idx = slot_index(path)
            if idx is None:
                reply(conn, status=NOT_FOUND)
            else:
                gc.collect()
                slot_raw[idx] = encode_temps(frame)
                slot_matched[idx] = False   # reset hysteresis state on new save
                lbl = body.strip()[:40]
                if lbl:
                    labels[idx] = lbl
                print('Saved slot', idx, '-', labels[idx])
                save_slot(idx, frame)
                save_config()
                reply(conn, b'OK')

        elif path.startswith('/delete/') and method == 'POST':
            idx = slot_index(path)
            if idx is None:
                reply(conn, status=NOT_FOUND)
            else:
                slot_raw[idx] = None
                slot_matched[idx] = False
                slot_score[idx] = None
                try:
                    os.remove(SLOT_FILE % idx)
                except OSError:
                    pass
                print('Deleted slot', idx)
                reply(conn, b'OK')

        elif path.startswith('/thresholds/') and method == 'POST':
            ok = False
            idx = slot_index(path)
            if idx is not None:
                try:
                    parts = body.strip().split(',')
                    trig = max(MIN_THRESHOLD, float(parts[0]))
                    rst  = max(MIN_THRESHOLD, float(parts[1]))
                    if rst < trig:  # hysteresis needs reset strictly below trigger
                        trigger_thresholds[idx] = trig
                        reset_thresholds[idx]   = rst
                        print('Thresholds[%d] trig=%.3f rst=%.3f' % (idx, trig, rst))
                        save_config()
                        ok = True
                    else:
                        print('thresholds rejected: reset %.3f >= trigger %.3f' % (rst, trig))
                except Exception as e:
                    print('thresholds parse err:', e)
            if ok:
                reply(conn, b'OK')
            else:
                reply(conn, ('reset must be below trigger (min %s)' % MIN_THRESHOLD).encode(),
                      status=BAD_REQUEST)

        elif path == '/frame' and DEBUG:
            send_csv(conn, frame)

        elif path.startswith('/slot_frame/') and DEBUG:
            idx = slot_index(path)
            ref = None
            if idx is not None and slot_raw[idx] is not None:
                try:
                    ref = read_slot_file(idx)
                except OSError:
                    pass
            if ref is None:
                reply(conn, status=NOT_FOUND)
            else:
                send_csv(conn, ref)

        elif path == '/ignore_zones' and method == 'POST':
            try:
                zones = json.loads(body)
                valid = []
                for z in zones:
                    if len(z) == 4:
                        x1,y1,x2,y2 = int(z[0]),int(z[1]),int(z[2]),int(z[3])
                        if 0 <= x1 <= x2 <= 31 and 0 <= y1 <= y2 <= 23:
                            valid.append([x1,y1,x2,y2])
                set_ignore_zones(valid)
                save_config()
                reply(conn, b'OK')
            except Exception as e:
                print('ignore_zones err:', e)
                reply(conn, status=BAD_REQUEST)

        else:
            reply(conn, status=NOT_FOUND)

    except OSError:
        pass  # client disconnected mid-transfer (ETIMEDOUT, ECONNRESET, etc.)
    except Exception as e:
        print('HTTP err:', e)
    finally:
        conn.close()

# ── Home Assistant reporting (MQTT) ───────────────────────────────
def _mqtt_connect():
    global _mqtt_client
    from umqtt.simple import MQTTClient
    _mqtt_client = MQTTClient(device_hostname, MQTT_HOST, port=MQTT_PORT,
                              user=MQTT_USER, password=MQTT_PASS, keepalive=60)
    _t0 = time.ticks_ms()
    _mqtt_client.connect()  # blocking: DNS resolve + TCP connect, runs in the main loop
    print('MQTT connected in %dms' % time.ticks_diff(time.ticks_ms(), _t0))
    _mqtt_discovery()
    gc.collect()

def _mqtt_discovery():
    st = device_hostname + '/state'
    dev = {'identifiers': [device_hostname], 'name': device_name}
    entities = [
        ('binary_sensor', 'presence', 'Occupancy',
         '{{ value_json.presence }}', 'occupancy', None, None),
        ('sensor', 'ip', 'IP',
         '{{ value_json.ip }}', None, None, None),
        ('sensor', 'mem_free', 'Free Memory',
         '{{ value_json.mem_free }}', None, 'B', 'measurement'),
        ('sensor', 'rssi', 'WiFi RSSI',
         '{{ value_json.rssi }}', 'signal_strength', 'dBm', 'measurement'),
        ('sensor', 'valid_px', 'Valid Pixels',
         '{{ value_json.valid_px }}', None, 'px', 'measurement'),
    ]
    for i in range(N_SLOTS):
        entities.append(('sensor', 'slot%d_score' % (i + 1), labels[i] + ' Score',
                         '{{ value_json.slot%d_score }}' % (i + 1), None, None, 'measurement'))
    for comp, key, fname, tmpl, dclass, unit, sclass in entities:
        uid = device_hostname + '_' + key
        cfg = {'name': fname, 'state_topic': st, 'value_template': tmpl,
               'unique_id': uid, 'device': dev}
        if dclass: cfg['device_class'] = dclass
        if unit:   cfg['unit_of_measurement'] = unit
        if sclass: cfg['state_class'] = sclass
        _mqtt_client.publish(('homeassistant/%s/%s/config' % (comp, uid)).encode(),
                             json.dumps(cfg).encode(), retain=True)
        gc.collect()

def report_to_ha():
    global _mqtt_client, _mqtt_fails
    if not wlan.isconnected():
        _mqtt_client = None  # reconnect and re-announce once WiFi is back
        return
    state = {
        'presence': 'ON' if presence else 'OFF',
        'ip':       wlan.ifconfig()[0],
        'mem_free': mem_free,
        'rssi':     rssi,
        'valid_px': valid_px,
    }
    for i in range(N_SLOTS):
        m = slot_score[i]
        state['slot%d_score' % (i + 1)] = round(m, 4) if m is not None else None
    _t0 = time.ticks_ms()
    try:
        if _mqtt_client is None:
            _mqtt_connect()
        _mqtt_client.publish((device_hostname + '/state').encode(),
                             json.dumps(state).encode())
    except Exception as e:
        _mqtt_fails += 1
        print('MQTT err:', e, '(fails=%d)' % _mqtt_fails)
        _mqtt_client = None
    _dt = time.ticks_diff(time.ticks_ms(), _t0)
    if _dt > 200:  # report_to_ha runs in the main loop; flag when it stalls it
        print('report_to_ha slow: %dms (mem_free=%d)' % (_dt, gc.mem_free()))

# ── Startup ───────────────────────────────────────────────────────
_config_complete = load_config()
if device_hostname is None:
    _mac = wlan.config('mac')
    device_hostname = '%s-%02X%02X' % (device_name, _mac[-2], _mac[-1])
if not _config_complete:
    save_config()
load_slots()

wlan.config(dhcp_hostname=device_hostname)
wlan.config(pm=wlan.PM_POWERSAVE)
wifi_connect()  # does not wait; check_wifi() reports when the connection comes up

srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(('', 80))
srv.listen(2)
srv.setblocking(False)
print('Started as', device_hostname)

last_read     = time.ticks_ms()
last_ha       = time.ticks_ms()
last_perf     = time.ticks_ms()

# ── Loop diagnostics ──────────────────────────────────────────────
# Heartbeat prints loop rate, memory trend (leak detection) and the longest
# single iteration since the last beat (blocking detection). If the loop is
# alive but the web UI is dead, the heartbeat keeps printing while max_loop
# and/or mem_free reveal where it is stuck or leaking.
last_beat     = time.ticks_ms()
_loop_prev    = last_beat
loop_count    = 0
max_loop_ms   = 0

# ── Main loop ─────────────────────────────────────────────────────
while True:
    _lt = time.ticks_ms()
    _iter = time.ticks_diff(_lt, _loop_prev)
    if _iter > max_loop_ms:
        max_loop_ms = _iter
    _loop_prev = _lt
    loop_count += 1
    if time.ticks_diff(_lt, last_beat) >= BEAT_INTERVAL:
        print('HB up=%ds mem_free=%d loops=%d max_loop=%dms wifi=%d mqtt_fails=%d' % (
            time.ticks_ms() // 1000, gc.mem_free(), loop_count, max_loop_ms,
            1 if wlan.isconnected() else 0, _mqtt_fails))
        last_beat   = _lt
        loop_count  = 0
        max_loop_ms = 0

    check_wifi()

    now = time.ticks_ms()
    if time.ticks_diff(now, last_read) >= SENSOR_INTERVAL:
        _rs0 = time.ticks_ms()
        try:
            _p, _m, _ss, _sm, valid_px, _ok = read_sensor(
                mlx, frame, slot_raw, slot_matched,
                trigger_thresholds, reset_thresholds, ignore_mask)
            if _ok:
                presence, matched, slot_score, slot_matched = _p, _m, _ss, _sm
            # else: corrupt frame — hold previous scores (read_sensor logs the reason)
        except Exception as e:  # I2C failure or incomplete frame — hold previous scores
            print('Sensor err:', e)
        _rs = time.ticks_diff(time.ticks_ms(), _rs0)
        if _rs > SLOW_READ_MS:  # I2C stall or repeated subpage reads
            print('read_sensor slow: %dms' % _rs)
        send_scores(_uart, presence, slot_score)
        gc.collect()
        last_read = time.ticks_ms()

    if time.ticks_diff(time.ticks_ms(), last_perf) >= PERF_INTERVAL:
        collect_perf()
        if LOG_ENABLED:
            write_perf_log()
        last_perf = time.ticks_ms()

    if presence != _ha_prev_presence or time.ticks_diff(time.ticks_ms(), last_ha) >= HA_INTERVAL:
        _ha_prev_presence = presence
        report_to_ha()
        gc.collect()
        last_ha = time.ticks_ms()

    _got_conn = False
    try:
        conn, _ = srv.accept()
        handle(conn)
        _got_conn = True
    except OSError as e:
        # EAGAIN (11) is the normal "no pending connection" on a non-blocking
        # socket. Anything else (e.g. EMFILE/ENFILE socket exhaustion, EBADF)
        # means the web server itself is failing — surface it.
        _en = e.args[0] if e.args else 0
        if _en not in (11, -11):
            print('accept err:', _en)

    if not _got_conn:
        _now = time.ticks_ms()
        _until_sensor = max(0, SENSOR_INTERVAL - time.ticks_diff(_now, last_read))
        _until_ha     = max(0, HA_INTERVAL - time.ticks_diff(_now, last_ha))
        _sleep = min(_until_sensor, _until_ha, 100)
        if _sleep > 2:
            time.sleep_ms(_sleep)
