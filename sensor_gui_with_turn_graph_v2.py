import socket
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import time
import re

# ============================================================
# CONSTANTS  — edit these as needed
# ============================================================
HOST = "192.168.1.1"
PORT = 288
MOVEMENT_SCALE = 0.12    # Robot MOV distance is millimeters; map units are centimeters.

# ============================================================
# FIELD DIMENSIONS  — fixed arena size in cm
# Origin (0, 0) = top-left corner of the field.
# Robot starts at (ROBOT_RADIUS, -ROBOT_RADIUS) so its body is fully inside.
# X grows rightward  (0 → FIELD_W).
# Y grows downward   (0 → -FIELD_H) to match the heading convention
#   where -90° points in the -Y direction.
# ============================================================
FIELD_W      = 423   # cm — horizontal (X axis)
FIELD_H      = 243   # cm — vertical   (Y axis, 0 at top, -FIELD_H at bottom)
ROBOT_WIDTH  = 33    # cm — physical robot diameter (used for map body drawing)
ROBOT_RADIUS = ROBOT_WIDTH / 2.0   # 16.5 cm

# IMU HEADING CONVERSION
# The BNO055 reports Euler heading: 0 = North (up on map), increases clockwise.
# The GUI uses math convention: 0 = East (+X), increases counter-clockwise.
# Conversion:  gui_heading = 90.0 - imu_heading
# This is applied every time an IMU: packet is received.
IMU_ENABLED = True   # Set False to fall back to pure dead-reckoning only

# Distance from any wall (cm) that triggers the 'o' warning to the robot
OOB_THRESHOLD = 15   # cm
SEND_OOB_WARNING_TO_ROBOT = False  # Keep False: firmware uses 'o' for LEFT_90.

# ============================================================
# ADC SENSOR ANGLES  — degrees relative to robot heading
#
#   0   = straight ahead (same direction the robot faces)
#   90  = directly to the robot's LEFT
#  -90  = directly to the robot's RIGHT  (or equivalently 270)
#   180 = directly behind
#
# Sensor IDs match the number in the COL:<id>,<val> protocol message.
# Change these once you know the physical mounting angles.
# ============================================================
ADC_SENSOR_ANGLES = {
    1:  10,    # Light bump center left
    2:  45,    # Light bump front left
    3: -45,    # Light bump front right
    4:  90,    # Light bump left side
    5: -90,    # Light bump right side
    6: -10,    # Light bump center right
    7:  90,    # Cliff/bottom left
    8:  35,    # Cliff/bottom front left
    9: -35,    # Cliff/bottom front right
    10: -90,   # Cliff/bottom right
}

# ADC distance scaling: the dot is projected (adc_value * ADC_DIST_SCALE) cm
# from the robot along the sensor's angle.
# ADC range is 0–4095; tune ADC_DIST_SCALE so the distances look right on your map.
# Example: scale=0.05 → adc=2000 projects the dot 100 cm out.
ADC_DIST_SCALE = 0.05   # cm per ADC count — adjust to match real sensor range

# ============================================================
# SOCKET SETUP  — comment back in when connected to robot
# ============================================================
#sock = None
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
sock.setblocking(False)

recv_buffer = ""

# Keep True if you want the map to move immediately when you click buttons.
# Set False after firmware MOV: telemetry is stable to avoid double-counting.
LOCAL_DEAD_RECKONING_ON_BUTTON_PRESS = False

# Debounce repeated/overlapping keypresses so a held key does not spam UART.
COMMAND_COOLDOWN = 0.08  # seconds
_last_sent = {}

# Continuous turns do not have a known angle locally, so the GUI should not
# rotate the map for move_left()/move_right() unless the robot sends MOV:<angle>l/r.
# The fixed 90-degree functions below do have known angles and can update the map.


# ============================================================
# SCAN CLASS
# ============================================================
class Scan:
    """
    Represents one scan taken by the robot.

    Attributes
    ----------
    scan_number : int
        Sequential index of this scan (1-based).
    movements : list[str]
        Movement commands that brought the robot here from the previous scan.
        Format: numeric prefix + direction letter, e.g. ["90r", "150f", "45l"].
        Direction letters: f=forward, b=backward, r=right turn, l=left turn.
    data : list[tuple]
        Sensor readings as (angle_deg, ping_cm, ir_raw) tuples.
    x : float
        Estimated X position on the map (cm) after dead reckoning.
    y : float
        Estimated Y position on the map (cm) after dead reckoning.
    heading : float
        Estimated heading in degrees after dead reckoning.
        0 = facing right (+X), 90 = facing up (+Y).
    """
    def __init__(self, scan_number, movements, data, x=0.0, y=0.0, heading=90.0):
        self.scan_number = scan_number
        self.movements   = movements
        self.data        = data
        self.x           = x
        self.y           = y
        self.heading     = heading

    def __repr__(self):
        return (f"Scan(#{self.scan_number}, pos=({self.x:.1f},{self.y:.1f}), "
                f"hdg={self.heading:.1f}°, cmds={self.movements}, "
                f"readings={len(self.data)})")


# ============================================================
# ADC COLLISION EVENT
# ============================================================
class ColEvent:
    """
    One COL: reading received from the robot.

    dot_x / dot_y are the projected world-space position of the detection,
    computed by projecting ADC_PROJECTION_DIST cm outward from the robot
    along the sensor's mounting angle (relative to robot heading at the
    time of the reading).

    Attributes
    ----------
    sensor_id  : int    Sensor number (from COL:<id>,<val>).
    adc_value  : int    Raw ADC value (0-4095).
    x, y       : float  Robot body position when the event arrived.
    heading    : float  Robot heading when the event arrived.
    dot_x/dot_y: float  Projected detection position on the map.
    """
    def __init__(self, sensor_id, adc_value, x, y, heading):
        self.sensor_id = sensor_id
        self.adc_value = adc_value
        self.x         = x
        self.y         = y
        self.heading   = heading

        rel_angle   = ADC_SENSOR_ANGLES.get(sensor_id, 0)
        world_rad   = math.radians(heading + rel_angle)
        dist        = adc_value * ADC_DIST_SCALE
        self.dot_x  = x + dist * math.cos(world_rad)
        self.dot_y  = y + dist * math.sin(world_rad)

    def __repr__(self):
        return (f"ColEvent(sensor={self.sensor_id}, adc={self.adc_value}, "
                f"robot=({self.x:.1f},{self.y:.1f}), "
                f"dot=({self.dot_x:.1f},{self.dot_y:.1f}))")


# ============================================================
# DEAD RECKONING
# ============================================================
def parse_movement(cmd):
    """
    Strictly parse a clean movement command like: 12f, 3r, 90l, 4b.
    Returns (value, direction).
    """
    cmd = cmd.strip().lower()
    m = re.fullmatch(r"(\d+(?:\.\d+)?)([fblr])", cmd)
    if not m:
        raise ValueError(f"Bad movement command: {cmd!r}")
    value = float(m.group(1))
    direction = m.group(2)
    return value, direction


def extract_movement_from_packet(packet):
    """
    Recover the first valid movement from a possibly polluted MOV payload.

    Examples caused by command echo overlap:
      '2rw' -> '2r'
      '6fx' -> '6f'
      '2fo' -> '2f'
      '1bw' -> '1b'

    This makes the GUI tolerant, but the better C-side fix is still:
      1) stop echoing received command bytes
      2) end MOV packets with \r\n
    """
    packet = packet.strip().lower()
    m = re.search(r"(\d+(?:\.\d+)?)([fblr])", packet)
    if not m:
        return None
    return m.group(1) + m.group(2)


def apply_movements(start_x, start_y, start_heading, movements):
    x, y, heading = start_x, start_y, start_heading
    for cmd in movements:
        if not cmd:
            continue
        try:
            value, direction = parse_movement(cmd)
        except (ValueError, IndexError):
            print(f"[WARNING] Could not parse movement command: '{cmd}'")
            continue
        if direction == 'r':
            heading -= value
        elif direction == 'l':
            heading += value
        elif direction == 'f':
            rad = math.radians(heading)
            x += value * MOVEMENT_SCALE * math.cos(rad)
            y += value * MOVEMENT_SCALE * math.sin(rad)
        elif direction == 'b':
            rad = math.radians(heading)
            x -= value * MOVEMENT_SCALE * math.cos(rad)
            y -= value * MOVEMENT_SCALE * math.sin(rad)
        heading %= 360
    return x, y, heading


# ============================================================
# SCAN STORAGE & INGESTION
# ============================================================
scans      = []
col_events = []

def ingest_scan(data):
    scan_number = len(scans) + 1
    scan = Scan(scan_number, [], data, live_x, live_y, live_heading)
    scans.append(scan)
    print(f"[INFO] Ingested {scan}")
    draw_map()
    return scan


def ingest_col_event(sensor_id, adc_value):
    event = ColEvent(sensor_id, adc_value, live_x, live_y, live_heading)
    col_events.append(event)
    print(f"[INFO] Ingested {event}")
    draw_map()
    return event


# ============================================================
# DATA STORAGE FOR GRAPHS
# ============================================================
scan_graph_data = []

SCAN_COLORS = [
    "#00aaff", "#ff6600", "#00ff88", "#e94560",
    "#ffdd00", "#cc44ff", "#ff44aa", "#44ffee",
]

def get_scan_color(i):
    return SCAN_COLORS[i % len(SCAN_COLORS)]

_active_angles    = []
_active_ping_vals = []
_active_ir_vals   = []


# ============================================================
# LIVE POSE
# ============================================================
# Start at (ROBOT_RADIUS, -ROBOT_RADIUS) so the 33 cm body is fully inside the field.
live_x       = ROBOT_RADIUS        # 16.5 cm from left wall
live_y       = -ROBOT_RADIUS       # 16.5 cm from top wall
live_heading = -90.0               # facing DOWN (-Y direction)
_imu_heading_valid = False         # True once first IMU: packet has been received
_in_scan     = False
bump_left    = 0
bump_right   = 0
_oob_warning_sent = False  # True while robot is within OOB_THRESHOLD of a wall


def _check_bounds():
    """
    Called after every live pose update.
    Warns in the GUI when the estimated robot position is near a wall.

    IMPORTANT:
    This does NOT send b'o' to the robot anymore because your firmware uses
    'o' as the LEFT_90 command. Sending b'o' here can accidentally turn the robot.
    """
    global _oob_warning_sent
    near = (live_x < OOB_THRESHOLD or
            live_x > FIELD_W - OOB_THRESHOLD or
            live_y > OOB_THRESHOLD or
            live_y < -(FIELD_H - OOB_THRESHOLD))

    if near and not _oob_warning_sent:
        if SEND_OOB_WARNING_TO_ROBOT and sock:
            try:
                # Use a byte your firmware ignores, not 'o'.
                sock.sendall(b'!')
            except Exception:
                pass
        terminal.insert(tk.END, "  [WARNING] Near boundary on map\n")
        terminal.see(tk.END)
        _oob_warning_sent = True
    elif not near and _oob_warning_sent:
        _oob_warning_sent = False


def apply_single_movement(cmd):
    """
    Apply one movement command to the live pose.

    For translation (f/b): always update XY using the current live_heading.
    For rotation (r/l):    only update heading when IMU is not active
                           (i.e. no IMU: packet has been received yet).
                           Once the IMU is streaming, live_heading is owned
                           by the IMU; dead-reckoned turns are discarded.
    """
    global live_x, live_y, live_heading

    value, direction = parse_movement(cmd)

    if direction == 'r':
        if not _imu_heading_valid:
            live_heading -= value
            live_heading %= 360
    elif direction == 'l':
        if not _imu_heading_valid:
            live_heading += value
            live_heading %= 360
    elif direction == 'f':
        rad = math.radians(live_heading)
        live_x += value * MOVEMENT_SCALE * math.cos(rad)
        live_y += value * MOVEMENT_SCALE * math.sin(rad)
    elif direction == 'b':
        rad = math.radians(live_heading)
        live_x -= value * MOVEMENT_SCALE * math.cos(rad)
        live_y -= value * MOVEMENT_SCALE * math.sin(rad)

    _check_bounds()
    draw_map()
    return True


def apply_imu_heading(imu_degrees):
    """
    Called when an IMU:<degrees> packet arrives from the robot.

    Converts the BNO055 Euler heading (0 = North / up-on-map, clockwise)
    to the GUI math convention (0 = East / +X, counter-clockwise):

        gui_heading = 90.0 - imu_heading

    This keeps the arrow pointing in the direction the robot actually faces.
    """
    global live_heading, _imu_heading_valid
    if not IMU_ENABLED:
        return
    live_heading = (90.0 - imu_degrees) % 360
    _imu_heading_valid = True
    _check_bounds()
    draw_map()


def terminal_log(msg):
    terminal.insert(tk.END, msg)
    terminal.see(tk.END)


# ============================================================
# GUI SETUP
# ============================================================
root = tk.Tk()
root.title("Sensor GUI")
root.configure(bg="#1a1a2e")
root.geometry("1400x900")

# ---- top button bar ----
btn_frame = tk.Frame(root, bg="#1a1a2e")
btn_frame.pack(side=tk.TOP, fill=tk.X, padx=8, pady=4)

BTN_STYLE = dict(bg="#16213e", fg="#e0e0e0", activebackground="#0f3460",
                 activeforeground="#e94560", relief=tk.FLAT,
                 font=("Courier New", 9, "bold"), padx=8, pady=3)

def start_scan():
    _send_command(b'm', 'm')

def stop_scan():
    _send_command(b'h', 'h')

def start_zero_to_ninety_scan():
    _send_command(b'n', 'n')

def start_ninety_to_oneeighty_scan():
    _send_command(b'l', 'l')

def clear_graph():
    global live_x, live_y, live_heading, _oob_warning_sent, _imu_heading_valid
    scan_graph_data.clear()
    _active_angles.clear(); _active_ping_vals.clear(); _active_ir_vals.clear()
    scans.clear(); col_events.clear()
    live_x, live_y, live_heading = ROBOT_RADIUS, -ROBOT_RADIUS, -90.0
    _imu_heading_valid = False
    _oob_warning_sent = False
    ax1.clear(); ax2.clear()
    style_axes(); graph_canvas.draw(); draw_map()

def _send_command(byte_cmd, label, local_movement=None):
    """
    Send exactly one command byte to the robot.

    Do NOT add '\n'. Your TM4C interrupt handler expects single-character
    commands like w/s/a/d/x/m/h/n/l/o/p. Newlines can hit the default case
    in the ISR and stop the robot.
    """
    now = time.time()

    # Always allow stop through immediately. Debounce everything else.
    if label != 'x':
        last = _last_sent.get(label, 0)
        if now - last < COMMAND_COOLDOWN:
            return
        _last_sent[label] = now

    try:
        if sock is None:
            terminal_log(f"[ERROR] Socket is None. Could not send: {label}\n")
            return

        # Temporarily use blocking mode so one-byte commands leave cleanly.
        sock.setblocking(True)
        sock.sendall(byte_cmd)
        sock.setblocking(False)

        terminal_log(f"Sent byte: {byte_cmd!r} ({label})\n")

    except Exception as e:
        terminal_log(f"[SEND ERROR] {label}: {e}\n")
        try:
            if sock:
                sock.setblocking(False)
        except Exception:
            pass
        return

    if LOCAL_DEAD_RECKONING_ON_BUTTON_PRESS and local_movement:
        try:
            apply_single_movement(local_movement)
            terminal_log(f"  [local movement applied: {local_movement}]\n")
        except ValueError as e:
            terminal_log(f"  [local movement ignored: {e}]\n")


def move_forward():
    # Keep this distance synced with whatever the robot firmware does for 'w'.
    _send_command(b'w', 'w', '50f')


def move_backward():
    # Keep this distance synced with whatever the robot firmware does for 's'.
    _send_command(b's', 's', '50b')


def move_right():
    # Continuous right turn: send only. The GUI cannot know the angle unless
    # the robot sends back something like MOV:12r or MOV:90r.
    _send_command(b'd', 'd')


def move_left():
    # Continuous left turn: send only. The GUI cannot know the angle unless
    # the robot sends back something like MOV:12l or MOV:90l.
    _send_command(b'a', 'a')


def turn_right_90():
    # Fixed 90-degree right turn: safe to dead-reckon on the map.
    _send_command(b'p', 'p')


def turn_left_90():
    # Fixed 90-degree left turn: safe to dead-reckon on the map.
    _send_command(b'o', 'o')

def stop():
    _send_command(b'x', 'x')

tk.Button(btn_frame, text="Start Scan",        command=start_scan,                     **BTN_STYLE).pack(side=tk.LEFT, padx=3)
tk.Button(btn_frame, text="Start 0-90 Scan",   command=start_zero_to_ninety_scan,      **BTN_STYLE).pack(side=tk.LEFT, padx=3)
tk.Button(btn_frame, text="Start 90-180 Scan", command=start_ninety_to_oneeighty_scan, **BTN_STYLE).pack(side=tk.LEFT, padx=3)
tk.Button(btn_frame, text="Stop Scan",         command=stop_scan,                      **BTN_STYLE).pack(side=tk.LEFT, padx=3)
tk.Button(btn_frame, text="Clear Graph",       command=clear_graph,                    **BTN_STYLE).pack(side=tk.LEFT, padx=3)
tk.Button(btn_frame, text="Right 90",  command=turn_right_90,  **BTN_STYLE).pack(side=tk.RIGHT, padx=3)
tk.Button(btn_frame, text="Left 90", command=turn_left_90, **BTN_STYLE).pack(side=tk.RIGHT, padx=3)
tk.Button(btn_frame, text="↑ Forward",  command=move_forward,  **BTN_STYLE).pack(side=tk.RIGHT, padx=3)
tk.Button(btn_frame, text="↓ Backward", command=move_backward, **BTN_STYLE).pack(side=tk.RIGHT, padx=3)
tk.Button(btn_frame, text="→ Right",    command=move_right,    **BTN_STYLE).pack(side=tk.RIGHT, padx=3)
tk.Button(btn_frame, text="← Left",     command=move_left,     **BTN_STYLE).pack(side=tk.RIGHT, padx=3)
tk.Button(btn_frame, text="x Stop",     command=stop,          **BTN_STYLE).pack(side=tk.RIGHT, padx=3)

# ---- main content paned split ----
paned = tk.PanedWindow(root, orient=tk.HORIZONTAL, bg="#0a0a1a",
                       sashwidth=6, sashrelief=tk.FLAT,
                       sashcursor="sb_h_double_arrow")
paned.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=4)

# ---- LEFT panel ----
left_frame = tk.Frame(paned, bg="#1a1a2e")
paned.add(left_frame, minsize=200, width=580, stretch="always")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5.5, 7),
                                facecolor="#1a1a2e", tight_layout=True)

def style_axes():
    for ax, title, ylabel in [
        (ax1, "PING Sensor Data", "Distance (cm)"),
        (ax2, "IR Sensor Data",   "Raw ADC"),
    ]:
        ax.set_facecolor("#0d0d1a")
        ax.set_title(title, color="#e0e0e0", fontsize=9, fontfamily="Courier New")
        ax.set_xlabel("Angle", color="#888", fontsize=8)
        ax.set_ylabel(ylabel,  color="#888", fontsize=8)
        ax.tick_params(colors="#666")
        for spine in ax.spines.values():
            spine.set_edgecolor("#333")

style_axes()
graph_canvas = FigureCanvasTkAgg(fig, master=left_frame)
graph_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)


# ============================================================
# SETTINGS PANEL — two-row toggle grid under the graphs
#
#  Row 1:  PING  [rays] [dots]  |  IR   [rays] [dots]
#  Row 2:  ADC   [rays] [dots]  |  MAP  [path]
# ============================================================

# BooleanVars — one per independent layer
show_ping_rays = tk.BooleanVar(value=True)
show_ping_dots = tk.BooleanVar(value=True)
show_ir_rays   = tk.BooleanVar(value=True)
show_ir_dots   = tk.BooleanVar(value=True)
show_adc_rays  = tk.BooleanVar(value=True)   # line from robot → dot
show_adc_dots  = tk.BooleanVar(value=True)   # projected detection dot
show_path      = tk.BooleanVar(value=True)

SWITCH_ON_BG = "#083951"
SWITCH_OFF_BG = "#2a2a3e"
SWITCH_ON_FG  = "#00ff88"
SWITCH_OFF_FG = "#555566"
SWITCH_FONT   = ("Courier New", 8, "bold")


class ToggleSwitch(tk.Frame):
    """Compact pill-style ON/OFF toggle switch."""
    def __init__(self, parent, label, variable, on_color=SWITCH_ON_BG,
                 callback=None, label_width=5, **kwargs):
        super().__init__(parent, bg="#0d0d1a", **kwargs)
        self._var      = variable
        self._on_color = on_color
        self._callback = callback

        lbl = tk.Label(self, text=label, bg="#0d0d1a", fg="#aaaacc",
                       font=SWITCH_FONT, width=label_width, anchor="w")
        lbl.pack(side=tk.LEFT, padx=(3, 3))
        lbl.bind("<Button-1>", self._toggle)

        self._cw, self._ch = 40, 18
        self._canvas = tk.Canvas(self, width=self._cw, height=self._ch,
                                 bg="#0d0d1a", highlightthickness=0, cursor="hand2")
        self._canvas.pack(side=tk.LEFT)
        self._canvas.bind("<Button-1>", self._toggle)

        self._state_lbl = tk.Label(self, width=3, bg="#0d0d1a",
                                   font=("Courier New", 7, "bold"), anchor="w")
        self._state_lbl.pack(side=tk.LEFT, padx=(3, 0))
        self._draw()

    def _toggle(self, _=None):
        self._var.set(not self._var.get())
        self._draw()
        if self._callback:
            self._callback()

    def _draw(self):
        c, on = self._canvas, self._var.get()
        cw, ch = self._cw, self._ch
        r = ch // 2 - 1
        c.delete("all")
        bg = self._on_color if on else SWITCH_OFF_BG
        c.create_oval(1,       1, ch-1,    ch-1, fill=bg, outline="")
        c.create_oval(cw-ch+1, 1, cw-1,    ch-1, fill=bg, outline="")
        c.create_rectangle(ch//2, 1, cw-ch//2, ch-1, fill=bg, outline="")
        kx = cw - r - 3 if on else r + 3
        c.create_oval(kx-r, 2, kx+r, ch-2, fill="#ffffff", outline="")
        fg = SWITCH_ON_FG if on else SWITCH_OFF_FG
        self._state_lbl.config(text="ON" if on else "OFF", fg=fg)


def _redraw(_=None):
    draw_map()


SEP_STYLE = dict(bg="#333344", width=1)

settings_outer = tk.Frame(left_frame, bg="#0d0d1a")
settings_outer.pack(fill=tk.X, padx=2, pady=(3, 2))

tk.Label(settings_outer, text="MAP LAYERS", bg="#0d0d1a", fg="#e94560",
         font=("Courier New", 8, "bold")).pack(anchor="w", padx=8, pady=(4, 2))

# --- row 1: PING and IR ---
row1 = tk.Frame(settings_outer, bg="#0d0d1a")
row1.pack(fill=tk.X, padx=6, pady=1)

tk.Label(row1, text="PING", bg="#0d0d1a", fg="#00aaff",
         font=SWITCH_FONT, width=5).pack(side=tk.LEFT, padx=(0, 2))
ToggleSwitch(row1, "rays", show_ping_rays, callback=_redraw).pack(side=tk.LEFT, padx=1)
ToggleSwitch(row1, "dots", show_ping_dots, callback=_redraw).pack(side=tk.LEFT, padx=1)

tk.Frame(row1, height=16, **SEP_STYLE).pack(side=tk.LEFT, padx=10, fill=tk.Y)

tk.Label(row1, text="IR", bg="#0d0d1a", fg="#ff6600",
         font=SWITCH_FONT, width=4).pack(side=tk.LEFT, padx=(0, 2))
ToggleSwitch(row1, "rays", show_ir_rays, callback=_redraw).pack(side=tk.LEFT, padx=1)
ToggleSwitch(row1, "dots", show_ir_dots, callback=_redraw).pack(side=tk.LEFT, padx=1)

# --- row 2: ADC and PATH ---
row2 = tk.Frame(settings_outer, bg="#0d0d1a")
row2.pack(fill=tk.X, padx=6, pady=(1, 5))

tk.Label(row2, text="ADC", bg="#0d0d1a", fg="#cc44ff",
         font=SWITCH_FONT, width=5).pack(side=tk.LEFT, padx=(0, 2))
ToggleSwitch(row2, "rays", show_adc_rays, callback=_redraw).pack(side=tk.LEFT, padx=1)
ToggleSwitch(row2, "dots", show_adc_dots, callback=_redraw).pack(side=tk.LEFT, padx=1)

tk.Frame(row2, height=16, **SEP_STYLE).pack(side=tk.LEFT, padx=10, fill=tk.Y)

tk.Label(row2, text="MAP", bg="#0d0d1a", fg="#aaaacc",
         font=SWITCH_FONT, width=4).pack(side=tk.LEFT, padx=(0, 2))
ToggleSwitch(row2, "path", show_path, callback=_redraw).pack(side=tk.LEFT, padx=1)


# ---- RIGHT panel: vertical pane — map on top, terminal on bottom ----
right_frame = tk.Frame(paned, bg="#0d0d1a")
paned.add(right_frame, minsize=200, stretch="always")

right_paned = tk.PanedWindow(right_frame, orient=tk.VERTICAL, bg="#0a0a1a",
                              sashwidth=6, sashrelief=tk.FLAT,
                              sashcursor="sb_v_double_arrow")
right_paned.pack(fill=tk.BOTH, expand=True)

# --- map area (top, expands) ---
map_frame = tk.Frame(right_paned, bg="#0d0d1a")
right_paned.add(map_frame, stretch="always")

tk.Label(map_frame, text="FIELD MAP", bg="#0d0d1a", fg="#e94560",
         font=("Courier New", 10, "bold")).pack(side=tk.TOP, pady=(6, 2))

map_fig = plt.figure(figsize=(7, 7), facecolor="#0d0d1a")
map_ax  = map_fig.add_subplot(111)

# --- terminal area (bottom, resizable upward) ---
term_frame = tk.Frame(right_paned, bg="#0d0d1a")
right_paned.add(term_frame, height=160, stretch="never")

tk.Label(term_frame, text="TERMINAL", bg="#0d0d1a", fg="#e94560",
         font=("Courier New", 8, "bold")).pack(side=tk.TOP, anchor="w",
                                               padx=6, pady=(4, 1))
terminal = ScrolledText(term_frame, bg="#0d0d1a", fg="#00ff88",
                        insertbackground="#00ff88",
                        font=("Courier New", 9), relief=tk.FLAT)
terminal.pack(fill=tk.BOTH, expand=True, padx=4, pady=(0, 4))

def style_map_ax():
    """Style the map axes with fixed field bounds and a boundary rectangle."""
    oob = not (0 <= live_x <= FIELD_W and -FIELD_H <= live_y <= 0)
    border_color = "#ff2222" if oob else "#334455"

    map_ax.set_facecolor("#0d0d1a")
    map_ax.set_aspect('equal')
    map_ax.set_xlim(-10, FIELD_W + 10)
    map_ax.set_ylim(-FIELD_H - 10, 10)
    map_ax.tick_params(colors="#444")
    map_ax.set_xlabel("X (cm)", color="#555", fontsize=8)
    map_ax.set_ylabel("Y (cm)", color="#555", fontsize=8)
    map_ax.set_title("Robot Path & Scan Positions",
                     color="#aaa", fontsize=9, fontfamily="Courier New")
    for spine in map_ax.spines.values():
        spine.set_edgecolor(border_color)
        spine.set_linewidth(2 if oob else 1)
    map_ax.grid(True, color="#1e1e2e", linewidth=0.5)

    # Draw the field boundary rectangle
    from matplotlib.patches import Rectangle
    field_rect = Rectangle((0, -FIELD_H), FIELD_W, FIELD_H,
                            linewidth=2, edgecolor=border_color,
                            facecolor="none", linestyle="--", zorder=0, alpha=0.7)
    map_ax.add_patch(field_rect)

    # Draw OOB warning text at the bottom of the map
    if oob:
        map_ax.text(FIELD_W / 2, -FIELD_H - 6,
                    "⚠  ROBOT OUT OF BOUNDS  ⚠",
                    color="#ffdd00", fontsize=9, fontfamily="Courier New",
                    fontweight="bold", ha="center", va="top",
                    zorder=20)

style_map_ax()
map_canvas = FigureCanvasTkAgg(map_fig, master=map_frame)
map_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=4, pady=4)


# ============================================================
# MAP DRAWING
# ============================================================
PING_RAY_COLOR = "#00aaff"
IR_RAY_COLOR   = "#ff6600"
ADC_COLOR      = "#cc44ff"
PATH_COLOR     = "#00ff88"
SCAN_DOT_COLOR = "#e94560"
MAX_PING_RAY   = 200    # cm — cap ray length for display
IR_SCALE       = 0.05   # ir_raw  → display length in cm
ADC_DOT_SIZE   = 18     # fixed scatter marker area (s param) for all ADC dots


def draw_map():
    map_ax.clear()
    style_map_ax()

    # ── live robot body ──────────────────────────────────────────────────────
    # Draw a to-scale circle (33 cm diameter = ROBOT_WIDTH) representing the
    # physical robot footprint, plus a heading arrow from centre to front edge.
    from matplotlib.patches import Circle as MplCircle

    live_rad = math.radians(live_heading)

    # Body circle — cyan when normal, yellow outline when IMU is active
    body_edge = "#ffdd00" if _imu_heading_valid else "#00ffff"
    robot_circle = MplCircle(
        (live_x, live_y), ROBOT_RADIUS,
        linewidth=1.8, edgecolor=body_edge,
        facecolor="#003333" if _imu_heading_valid else "#001a1a",
        alpha=0.85, zorder=10
    )
    map_ax.add_patch(robot_circle)

    # Heading arrow — tip at the front edge of the robot body
    arrow_tip_x = live_x + ROBOT_RADIUS * math.cos(live_rad)
    arrow_tip_y = live_y + ROBOT_RADIUS * math.sin(live_rad)
    map_ax.annotate("",
        xy=(arrow_tip_x, arrow_tip_y),
        xytext=(live_x, live_y),
        arrowprops=dict(arrowstyle="->", color=body_edge, lw=2),
        zorder=11)

    # IMU status badge in top-left of the map
    imu_label = "IMU ✓" if _imu_heading_valid else "IMU —"
    imu_color = "#00ff88" if _imu_heading_valid else "#555566"
    map_ax.text(5, -5, imu_label, color=imu_color,
                fontsize=8, fontfamily="Courier New", fontweight="bold",
                va="top", zorder=20)

    # bump indicator
    if bump_left or bump_right:
        c = "#ff0000" if (bump_left and bump_right) else "#ffaa00"
        map_ax.scatter(live_x, live_y, color=c, s=200, zorder=12, alpha=0.5)
        if bump_left:
            map_ax.text(live_x - 14, live_y, "L", color="#ff4444",
                        fontsize=10, zorder=13)
        if bump_right:
            map_ax.text(live_x + 10, live_y, "R", color="#ff4444",
                        fontsize=10, zorder=13)

    # ADC events
    want_adc_rays = show_adc_rays.get()
    want_adc_dots = show_adc_dots.get()
    if (want_adc_rays or want_adc_dots) and col_events:
        for ev in col_events:
            if want_adc_rays:
                map_ax.plot([ev.x, ev.dot_x], [ev.y, ev.dot_y],
                            color=ADC_COLOR, linewidth=0.7, alpha=0.35, zorder=6)
            if want_adc_dots:
                map_ax.scatter(ev.dot_x, ev.dot_y,
                               color=ADC_COLOR, s=ADC_DOT_SIZE, alpha=0.65, zorder=7,
                               edgecolors="#ff88ff", linewidths=0.5)
                map_ax.text(ev.dot_x + 3, ev.dot_y + 3,
                            f"C{ev.sensor_id}", color=ADC_COLOR,
                            fontsize=6, fontfamily="Courier New", zorder=8)

    if not scans:
        _draw_legend()
        map_canvas.draw()
        return

    xs = [s.x for s in scans]
    ys = [s.y for s in scans]

    if show_path.get():
        map_ax.plot(xs, ys, color=PATH_COLOR, linewidth=1.5,
                    linestyle="--", alpha=0.7, zorder=1)

    want_ping_rays = show_ping_rays.get()
    want_ping_dots = show_ping_dots.get()
    want_ir_rays   = show_ir_rays.get()
    want_ir_dots   = show_ir_dots.get()

    for scan in scans:
        ping_ex, ping_ey = [], []
        ir_ex,   ir_ey   = [], []

        for angle_deg, ping_cm, ir_raw in scan.data:
            world_angle = math.radians(scan.heading + (angle_deg - 90))

            ray_len = min(ping_cm, MAX_PING_RAY)  # ping_cm is already centimeters
            ex = scan.x + ray_len * math.cos(world_angle)
            ey = scan.y + ray_len * math.sin(world_angle)
            ping_ex.append(ex); ping_ey.append(ey)

            ir_len = ir_raw * IR_SCALE * MOVEMENT_SCALE
            ix = scan.x + ir_len * math.cos(world_angle)
            iy = scan.y + ir_len * math.sin(world_angle)
            ir_ex.append(ix); ir_ey.append(iy)

            if want_ping_rays:
                map_ax.plot([scan.x, ex], [scan.y, ey],
                            color=PING_RAY_COLOR, linewidth=0.4, alpha=0.3, zorder=2)
            if want_ir_rays:
                map_ax.plot([scan.x, ix], [scan.y, iy],
                            color=IR_RAY_COLOR, linewidth=0.3, alpha=0.2, zorder=2)

        if want_ping_dots:
            map_ax.scatter(ping_ex, ping_ey, color=PING_RAY_COLOR,
                           s=4, alpha=0.6, zorder=3, linewidths=0)
        if want_ir_dots:
            map_ax.scatter(ir_ex, ir_ey, color=IR_RAY_COLOR,
                           s=3, alpha=0.5, zorder=3, linewidths=0)

        # heading arrow
        arrow_len = 15  # cm
        hdg_rad   = math.radians(scan.heading)
        map_ax.annotate("",
            xy=(scan.x + arrow_len * math.cos(hdg_rad),
                scan.y + arrow_len * math.sin(hdg_rad)),
            xytext=(scan.x, scan.y),
            arrowprops=dict(arrowstyle="->", color="#ffdd00", lw=1.2),
            zorder=4)

        map_ax.scatter(scan.x, scan.y, color=SCAN_DOT_COLOR,
                       s=60, zorder=5, edgecolors="#fff", linewidths=0.5)
        map_ax.text(scan.x + 5, scan.y + 5, f"#{scan.scan_number}",
                    color="#e0e0e0", fontsize=7, fontfamily="Courier New", zorder=6)

    _draw_legend()
    map_canvas.draw()


def _draw_legend():
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    elems = []
    if show_path.get():
        elems.append(Line2D([0],[0], color=PATH_COLOR, lw=1.5,
                            linestyle="--", label="Path"))
    if show_ping_rays.get() or show_ping_dots.get():
        elems.append(Line2D([0],[0], color=PING_RAY_COLOR, lw=1, label="PING"))
    if show_ir_rays.get() or show_ir_dots.get():
        elems.append(Line2D([0],[0], color=IR_RAY_COLOR, lw=1, label="IR"))
    if show_adc_rays.get() or show_adc_dots.get():
        elems.append(Patch(facecolor=ADC_COLOR, edgecolor="#ff88ff",
                           alpha=0.6, label="ADC"))
    if elems:
        map_ax.legend(handles=elems, loc="upper right",
                      facecolor="#1a1a2e", edgecolor="#333",
                      labelcolor="#aaa", fontsize=7)


# ============================================================
# LIVE DATA UPDATE LOOP
# ============================================================
def commit_active_scan():
    if _active_angles:
        scan_graph_data.append({
            "angles":    list(_active_angles),
            "ping_vals": list(_active_ping_vals),
            "ir_vals":   list(_active_ir_vals),
        })
        _active_angles.clear()
        _active_ping_vals.clear()
        _active_ir_vals.clear()


def redraw_graphs():
    ax1.clear(); ax2.clear()
    style_axes()
    for i, sd in enumerate(scan_graph_data):
        color = get_scan_color(i)
        ax1.plot(sd["angles"], sd["ping_vals"], color=color, linewidth=1,
                 marker='o', markersize=2, alpha=0.85, label=f"Scan {i+1}")
        ax2.plot(sd["angles"], sd["ir_vals"],   color=color, linewidth=1,
                 marker='o', markersize=2, alpha=0.85, label=f"Scan {i+1}")
    if _active_angles:
        ax1.plot(_active_angles, _active_ping_vals, color="#ffffff",
                 linewidth=1, marker='o', markersize=2, alpha=0.6, label="Active")
        ax2.plot(_active_angles, _active_ir_vals,   color="#ffffff",
                 linewidth=1, marker='o', markersize=2, alpha=0.6, label="Active")
    total = len(scan_graph_data) + (1 if _active_angles else 0)
    if total > 0:
        ax1.legend(loc="upper right", facecolor="#1a1a2e", edgecolor="#333",
                   labelcolor="#aaa", fontsize=7)
        ax2.legend(loc="upper right", facecolor="#1a1a2e", edgecolor="#333",
                   labelcolor="#aaa", fontsize=7)
    graph_canvas.draw()


def update_data():
    global recv_buffer, _in_scan

    if sock:
        try:
            while True:
                data = sock.recv(1024)
                if not data:
                    break
                recv_buffer += data.decode(errors='ignore')
        except BlockingIOError:
            pass

        while '\n' in recv_buffer:
            line, recv_buffer = recv_buffer.split('\n', 1)
            line = line.strip()
            if not line:
                continue

            terminal.insert(tk.END, line + "\n")
            terminal.see(tk.END)

            # IMU: — BNO055 fused heading
            # Format: IMU:<degrees>   e.g. "IMU:273.5"
            # Degrees are BNO055 Euler heading: 0 = North, clockwise, 0–360.
            # apply_imu_heading() converts to GUI math convention automatically.
            if line.startswith("IMU:"):
                raw_hdg = line[4:].strip()
                try:
                    imu_deg = float(raw_hdg)
                    apply_imu_heading(imu_deg)
                    terminal.insert(tk.END,
                        f"  [IMU heading={imu_deg:.1f}° → gui={live_heading:.1f}°]\n")
                    terminal.see(tk.END)
                except ValueError:
                    print(f"[WARNING] Bad IMU packet: {line}")

            # MOV:
            if line.startswith("MOV:"):
                raw_cmd = line[4:].strip()
                clean_cmd = extract_movement_from_packet(raw_cmd)

                if clean_cmd is None:
                    terminal_log(f"  [ignored bad MOV packet: {raw_cmd!r}]\n")
                else:
                    try:
                        apply_single_movement(clean_cmd)
                        if clean_cmd != raw_cmd:
                            terminal_log(f"  [movement applied: {clean_cmd} from noisy packet {raw_cmd!r}]\n")
                        else:
                            terminal_log(f"  [movement applied: {clean_cmd}]\n")
                    except ValueError as e:
                        terminal_log(f"  [ignored bad MOV packet: {raw_cmd!r}; {e}]\n")

            # COL: — ADC sensor reading
            # Format: COL:<sensor_id>,<adc_value>  e.g. "COL:2,1540"
            # sensor_id must match a key in ADC_SENSOR_ANGLES (0–5)
            elif line.startswith("COL:"):
                parts = line[4:].split(',')
                if len(parts) == 2:
                    try:
                        sensor_id = int(parts[0].strip())
                        adc_value = int(parts[1].strip())
                        ingest_col_event(sensor_id, adc_value)
                        terminal.insert(tk.END,
                            f"  [ADC sensor={sensor_id} value={adc_value}]\n")
                        terminal.see(tk.END)
                    except ValueError:
                        print(f"[WARNING] Bad COL data: {line}")
                else:
                    print(f"[WARNING] Malformed COL line: {line}")

            # SCAN_START
            elif line == "Starting Scan...":
                _in_scan = True
                _active_angles.clear()
                _active_ping_vals.clear()
                _active_ir_vals.clear()
                terminal.insert(tk.END, "  [scan started]\n")
                terminal.see(tk.END)

            # SCAN_END
            elif line == "SCAN_END":
                _in_scan = False
                if _active_angles:
                    sensor_data = list(zip(_active_angles,
                                           _active_ping_vals,
                                           _active_ir_vals))
                    scan_graph_data.append({
                        "angles":    list(_active_angles),
                        "ping_vals": list(_active_ping_vals),
                        "ir_vals":   list(_active_ir_vals),
                    })
                    ingest_scan(sensor_data)
                    _active_angles.clear()
                    _active_ping_vals.clear()
                    _active_ir_vals.clear()
                terminal.insert(tk.END, "  [scan ended]\n")
                terminal.see(tk.END)

            # BUMP:
            elif line.startswith("BUMP:"):
                data = line[5:].strip()
                if len(data) >= 2:
                    try:
                        global bump_left, bump_right
                        bump_left  = int(data[0])
                        bump_right = int(data[1])
                        terminal.insert(tk.END,
                            f"  [bump L:{bump_left} R:{bump_right}]\n")
                        terminal.see(tk.END)
                        draw_map()
                    except ValueError:
                        print(f"[WARNING] Bad BUMP data: {line}")

            # angle,ping,ir — sensor reading inside a scan
            else:
                # Ignore echoed command bytes if firmware still echoes received chars.
                if line in {'w', 'a', 's', 'd', 'x', 'm', 'h', 'n', 'l', 'o', 'p'}:
                    continue

                parts = line.split(',')
                if len(parts) == 3:
                    try:
                        angle = float(parts[0])
                        ping  = float(parts[1])
                        ir    = float(parts[2])
                        _active_angles.append(angle)
                        _active_ping_vals.append(ping)
                        _active_ir_vals.append(ir)
                    except ValueError:
                        pass

    redraw_graphs()
    root.after(100, update_data)


# ============================================================
# CLOSE HANDLER
# ============================================================
def on_close():
    try:
        if sock: sock.close()
    except Exception:
        pass
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# ============================================================
# DEMO  — uncomment load_demo_scans() to test without robot
# ============================================================
def load_demo_scans():
    """Fake scan + ADC data so the UI is immediately testable."""
    import random

    def fake_data():
        return [(a, random.uniform(30, 150), random.randint(200, 2000))
                for a in range(0, 181, 5)]

    # Robot starts at (ROBOT_RADIUS, -ROBOT_RADIUS) heading -90 (down).
    # "f" moves in -Y direction, so positions stay inside the field.
    demo_scans = [
        ([], fake_data()),
        (["80f", "90r", "60f"], fake_data()),
        (["45l", "70f"], fake_data()),
    ]

    for movements, data in demo_scans:
        for cmd in movements:
            apply_single_movement(cmd)
        ingest_scan(data)
        scan_graph_data.append({
            "angles":    [d[0] for d in data],
            "ping_vals": [d[1] for d in data],
            "ir_vals":   [d[2] for d in data],
        })
        for sid in random.sample(range(1, 7), k=random.randint(2, 4)):
            ingest_col_event(sid, random.randint(400, 3800))

#load_demo_scans()

# ============================================================
# KEYBOARD BINDINGS
# ============================================================
def on_key_press(event):
    key = event.keysym.lower()
    if   key == 'w':             move_forward()
    elif key == 's':             move_backward()
    elif key == 'a':             move_left()
    elif key == 'd':             move_right()
    elif key == 'p':             turn_right_90()
    elif key == 'o':             turn_left_90()
    elif key in ('x', 'space'): stop()
    elif key == 'm':             start_scan()
    elif key == 'h':             stop_scan()

root.bind("<KeyPress>", on_key_press)
root.focus_set()

# ============================================================
# START
# ============================================================
update_data()
root.mainloop()