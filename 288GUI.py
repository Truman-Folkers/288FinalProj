import socket
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math

# ============================================================
# CONSTANTS  — edit these as needed
# ============================================================
HOST = "192.168.1.1"
PORT = 288
MOVEMENT_SCALE = 1.0   # 1 unit on map = 1 cm; adjust once real scale is known

# ============================================================
# SOCKET SETUP  — comment back in when connected to robot
# ============================================================
# sock = None
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
sock.setblocking(False)

recv_buffer = ""

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
        self.data        = data          # list of (angle, ping_cm, ir_raw)
        self.x           = x
        self.y           = y
        self.heading     = heading       # degrees, 0=right, 90=up

    def __repr__(self):
        return (f"Scan(#{self.scan_number}, pos=({self.x:.1f},{self.y:.1f}), "
                f"hdg={self.heading:.1f}°, cmds={self.movements}, "
                f"readings={len(self.data)})")


# ============================================================
# DEAD RECKONING
# ============================================================
def parse_movement(cmd):
    """
    Parse a single movement command string into (value, direction).

    Examples
    --------
    "90r"  -> (90.0, 'r')
    "150f" -> (150.0, 'f')
    "45l"  -> (45.0, 'l')
    "200b" -> (200.0, 'b')
    """
    cmd = cmd.strip().lower()
    direction = cmd[-1]               # last character is the direction
    value = float(cmd[:-1])           # everything before is the number
    return value, direction


def apply_movements(start_x, start_y, start_heading, movements):
    """
    Apply a list of movement commands to a starting pose and return the
    resulting (x, y, heading).

    Parameters
    ----------
    start_x, start_y : float   Starting position in cm.
    start_heading    : float   Starting heading in degrees (0=right, 90=up).
    movements        : list[str]

    Returns
    -------
    (x, y, heading) : tuple[float, float, float]
    """
    x       = start_x
    y       = start_y
    heading = start_heading

    for cmd in movements:
        if not cmd:
            continue
        try:
            value, direction = parse_movement(cmd)
        except (ValueError, IndexError):
            print(f"[WARNING] Could not parse movement command: '{cmd}'")
            continue

        if direction == 'r':
            heading -= value          # clockwise turn decreases heading
        elif direction == 'l':
            heading += value          # counter-clockwise turn increases heading
        elif direction == 'f':
            rad = math.radians(heading)
            x  += value * MOVEMENT_SCALE * math.cos(rad)
            y  += value * MOVEMENT_SCALE * math.sin(rad)
        elif direction == 'b':
            rad = math.radians(heading)
            x  -= value * MOVEMENT_SCALE * math.cos(rad)
            y  -= value * MOVEMENT_SCALE * math.sin(rad)

        heading %= 360               # keep heading in [0, 360)

    return x, y, heading


# ============================================================
# SCAN STORAGE & INGESTION
# ============================================================
scans = []   # list of Scan objects in order received

def ingest_scan(data):
    """
    Create a new Scan from the sensor data using the CURRENT live pose
    as the scan's position.  Movements are now applied live as they arrive
    (see apply_single_movement) so we just snapshot here.

    Parameters
    ----------
    data : list[tuple]
        Sensor readings as (angle_deg, ping_cm, ir_raw).
    """
    scan_number = len(scans) + 1

    scan = Scan(scan_number, [], data, live_x, live_y, live_heading)
    scans.append(scan)
    print(f"[INFO] Ingested {scan}")
    draw_map()
    return scan


# ============================================================
# DATA STORAGE FOR GRAPHS
# Per-scan storage: each entry is (angles, ping_vals, ir_vals) for one scan
# ============================================================
scan_graph_data = []   # list of dicts: {angles, ping_vals, ir_vals}

# Palette — one color per scan, cycles if more scans than colors
SCAN_COLORS = [
    "#00aaff",  # blue
    "#ff6600",  # orange
    "#00ff88",  # green
    "#e94560",  # red
    "#ffdd00",  # yellow
    "#cc44ff",  # purple
    "#ff44aa",  # pink
    "#44ffee",  # cyan
]

def get_scan_color(scan_index):
    return SCAN_COLORS[scan_index % len(SCAN_COLORS)]

# Active (current incoming) scan buffer
_active_angles    = []
_active_ping_vals = []
_active_ir_vals   = []


# ============================================================
# LIVE POSE
# Updated immediately as each MOV: arrives, so the robot's position
# on the map moves in real time.  When SCAN_END is received, this
# pose is snapshotted as the position of the new Scan.
# ============================================================
live_x       = 0.0
live_y       = 0.0
live_heading = 90.0       # degrees: 0=right, 90=up
_in_scan     = False      # True between SCAN_START and SCAN_END


def apply_single_movement(cmd):
    """
    Apply ONE movement command (e.g. "20f", "5r") to the live pose
    and trigger a redraw so the robot icon moves on the map immediately.
    """
    global live_x, live_y, live_heading
    try:
        value, direction = parse_movement(cmd)
    except (ValueError, IndexError):
        print(f"[WARNING] Could not parse movement command: '{cmd}'")
        return

    if direction == 'r':
        live_heading -= value
    elif direction == 'l':
        live_heading += value
    elif direction == 'f':
        rad = math.radians(live_heading)
        live_x += value * MOVEMENT_SCALE * math.cos(rad)
        live_y += value * MOVEMENT_SCALE * math.sin(rad)
    elif direction == 'b':
        rad = math.radians(live_heading)
        live_x -= value * MOVEMENT_SCALE * math.cos(rad)
        live_y -= value * MOVEMENT_SCALE * math.sin(rad)

    live_heading %= 360
    draw_map()


# ============================================================
# GUI SETUP
# ============================================================
root = tk.Tk()
root.title("Sensor GUI")
root.configure(bg="#1a1a2e")
root.geometry("1400x800")

# ---- top button bar ----
btn_frame = tk.Frame(root, bg="#1a1a2e")
btn_frame.pack(side=tk.TOP, fill=tk.X, padx=8, pady=4)

BTN_STYLE = dict(bg="#16213e", fg="#e0e0e0", activebackground="#0f3460",
                 activeforeground="#e94560", relief=tk.FLAT,
                 font=("Courier New", 9, "bold"), padx=8, pady=3)

def start_scan():
    if sock:
        sock.sendall(b'm')
    terminal.insert(tk.END, "Sent: m\n")
    terminal.see(tk.END)

def stop_scan():
    if sock:
        sock.sendall(b'h')
    terminal.insert(tk.END, "Sent: h\n")
    terminal.see(tk.END)

def clear_graph():
    global live_x, live_y, live_heading
    scan_graph_data.clear()
    _active_angles.clear()
    _active_ping_vals.clear()
    _active_ir_vals.clear()
    scans.clear()
    # reset live pose back to origin
    live_x, live_y, live_heading = 0.0, 0.0, 90.0
    ax1.clear(); ax2.clear()
    style_axes()
    graph_canvas.draw()
    draw_map()

def move_forward():
    if sock: sock.sendall(b'w')
    terminal.insert(tk.END, "Sent: w\n")
    terminal.see(tk.END)

def move_backward():
    if sock: sock.sendall(b's')
    terminal.insert(tk.END, "Sent: s\n")
    terminal.see(tk.END)

def move_right():
    if sock: sock.sendall(b'd')
    terminal.insert(tk.END, "Sent: d\n")
    terminal.see(tk.END)

def move_left():
    if sock: sock.sendall(b'a')
    terminal.insert(tk.END, "Sent: a\n")
    terminal.see(tk.END)

def stop():
    if sock: sock.sendall(b'x')
    terminal.insert(tk.END, "Sent: x\n")
    terminal.see(tk.END)

# ---- ray toggle state ----
show_rays = True

def toggle_rays():
    global show_rays
    show_rays = not show_rays
    ray_btn.config(
        text="Rays: ON " if show_rays else "Rays: OFF",
        fg="#00ff88"     if show_rays else "#e94560"
    )
    draw_map()

tk.Button(btn_frame, text="Start Scan",  command=start_scan,  **BTN_STYLE).pack(side=tk.LEFT, padx=3)
tk.Button(btn_frame, text="Stop Scan",   command=stop_scan,   **BTN_STYLE).pack(side=tk.LEFT, padx=3)
tk.Button(btn_frame, text="Clear Graph", command=clear_graph, **BTN_STYLE).pack(side=tk.LEFT, padx=3)
ray_btn = tk.Button(btn_frame, text="Rays: ON", command=toggle_rays,
                    **{**BTN_STYLE, "fg": "#00ff88"})
ray_btn.pack(side=tk.LEFT, padx=3)
tk.Button(btn_frame, text="↑ Forward",  command=move_forward,  **BTN_STYLE).pack(side=tk.RIGHT, padx=3)
tk.Button(btn_frame, text="↓ Backward", command=move_backward, **BTN_STYLE).pack(side=tk.RIGHT, padx=3)
tk.Button(btn_frame, text="→ Right",    command=move_right,    **BTN_STYLE).pack(side=tk.RIGHT, padx=3)
tk.Button(btn_frame, text="← Left",     command=move_left,     **BTN_STYLE).pack(side=tk.RIGHT, padx=3)
tk.Button(btn_frame, text="x Stop",     command=stop,     **BTN_STYLE).pack(side=tk.RIGHT, padx=3)

# ---- main content area: PanedWindow for resizable split ----
paned = tk.PanedWindow(root, orient=tk.HORIZONTAL, bg="#0a0a1a",
                       sashwidth=6, sashrelief=tk.FLAT,
                       sashcursor="sb_h_double_arrow")
paned.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=4)

# ---- LEFT: graphs (2/5 width) ----
left_frame = tk.Frame(paned, bg="#1a1a2e")
paned.add(left_frame, minsize=200, width=560, stretch="always")

# terminal inside left frame
terminal = ScrolledText(left_frame, height=10, bg="#0d0d1a", fg="#00ff88",
                        insertbackground="#00ff88",
                        font=("Courier New", 8), relief=tk.FLAT)
terminal.pack(fill=tk.X, padx=2, pady=(0, 4))

# matplotlib graphs
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5.5, 8),
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

# ---- RIGHT: map (3/5 width) ----
right_frame = tk.Frame(paned, bg="#0d0d1a", relief=tk.FLAT)
paned.add(right_frame, minsize=200, stretch="always")

map_label = tk.Label(right_frame, text="FIELD MAP",
                     bg="#0d0d1a", fg="#e94560",
                     font=("Courier New", 10, "bold"))
map_label.pack(side=tk.TOP, pady=(6, 2))

map_fig = plt.figure(figsize=(7, 7), facecolor="#0d0d1a")
map_ax  = map_fig.add_subplot(111)

def style_map_ax():
    map_ax.set_facecolor("#0d0d1a")
    map_ax.set_aspect('equal')
    map_ax.tick_params(colors="#444")
    map_ax.set_xlabel("X (cm)", color="#555", fontsize=8)
    map_ax.set_ylabel("Y (cm)", color="#555", fontsize=8)
    map_ax.set_title("Robot Path & Scan Positions",
                     color="#aaa", fontsize=9, fontfamily="Courier New")
    for spine in map_ax.spines.values():
        spine.set_edgecolor("#222")
    map_ax.grid(True, color="#1e1e2e", linewidth=0.5)

style_map_ax()

map_canvas = FigureCanvasTkAgg(map_fig, master=right_frame)
map_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=4, pady=4)


# ============================================================
# MAP DRAWING
# ============================================================
PING_RAY_COLOR = "#00aaff"
IR_RAY_COLOR   = "#ff6600"
PATH_COLOR     = "#00ff88"
SCAN_DOT_COLOR = "#e94560"
MAX_PING_RAY   = 200    # cm — cap ray length for display
IR_SCALE       = 0.05   # scale factor for ir_raw → display length


def draw_map():
    """Redraw the entire map from the scans list."""
    map_ax.clear()
    style_map_ax()

    # ---- always draw the LIVE robot marker (cyan triangle + heading arrow) ----
    live_rad = math.radians(live_heading)
    live_arrow_len = 20 * MOVEMENT_SCALE
    map_ax.annotate("",
        xy=(live_x + live_arrow_len * math.cos(live_rad),
            live_y + live_arrow_len * math.sin(live_rad)),
        xytext=(live_x, live_y),
        arrowprops=dict(arrowstyle="->", color="#00ffff", lw=2),
        zorder=10)
    map_ax.scatter(live_x, live_y, color="#00ffff",
                   s=80, zorder=11, edgecolors="#fff",
                   linewidths=0.8, marker='^')

    if not scans:
        map_canvas.draw()
        return

    xs = [s.x for s in scans]
    ys = [s.y for s in scans]

    # ---- draw robot path ----
    map_ax.plot(xs, ys, color=PATH_COLOR, linewidth=1.5,
                linestyle="--", alpha=0.7, zorder=1)

    for scan in scans:
        # ---- draw sensor rays and/or endpoint dots ----
        ping_ex, ping_ey = [], []
        ir_ex,   ir_ey   = [], []

        for angle_deg, ping_cm, ir_raw in scan.data:
            world_angle = math.radians(scan.heading + (angle_deg - 90))

            # PING endpoint
            ray_len = min(ping_cm, MAX_PING_RAY) * MOVEMENT_SCALE
            ex = scan.x + ray_len * math.cos(world_angle)
            ey = scan.y + ray_len * math.sin(world_angle)
            ping_ex.append(ex); ping_ey.append(ey)

            # IR endpoint
            ir_len = ir_raw * IR_SCALE * MOVEMENT_SCALE
            ix = scan.x + ir_len * math.cos(world_angle)
            iy = scan.y + ir_len * math.sin(world_angle)
            ir_ex.append(ix); ir_ey.append(iy)

            # draw rays only if toggle is on
            if show_rays:
                map_ax.plot([scan.x, ex], [scan.y, ey],
                            color=PING_RAY_COLOR, linewidth=0.4, alpha=0.3, zorder=2)
                map_ax.plot([scan.x, ix], [scan.y, iy],
                            color=IR_RAY_COLOR, linewidth=0.3, alpha=0.2, zorder=2)

        # always draw endpoint dots
        map_ax.scatter(ping_ex, ping_ey, color=PING_RAY_COLOR,
                       s=4, alpha=0.6, zorder=3, linewidths=0)
        map_ax.scatter(ir_ex, ir_ey, color=IR_RAY_COLOR,
                       s=3, alpha=0.5, zorder=3, linewidths=0)

        # ---- heading arrow ----
        arrow_len = 15 * MOVEMENT_SCALE
        hdg_rad = math.radians(scan.heading)
        map_ax.annotate("",
            xy=(scan.x + arrow_len * math.cos(hdg_rad),
                scan.y + arrow_len * math.sin(hdg_rad)),
            xytext=(scan.x, scan.y),
            arrowprops=dict(arrowstyle="->", color="#ffdd00", lw=1.2),
            zorder=4)

        # ---- scan position dot + label ----
        map_ax.scatter(scan.x, scan.y, color=SCAN_DOT_COLOR,
                       s=60, zorder=5, edgecolors="#fff", linewidths=0.5)
        map_ax.text(scan.x + 5, scan.y + 5, f"#{scan.scan_number}",
                    color="#e0e0e0", fontsize=7, fontfamily="Courier New", zorder=6)

    # ---- legend ----
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color=PATH_COLOR,     lw=1.5, linestyle="--", label="Path"),
        Line2D([0], [0], color=PING_RAY_COLOR, lw=1,   label="PING"),
        Line2D([0], [0], color=IR_RAY_COLOR,   lw=1,   label="IR"),
    ]
    map_ax.legend(handles=legend_elements, loc="upper right",
                  facecolor="#1a1a2e", edgecolor="#333",
                  labelcolor="#aaa", fontsize=7)

    map_canvas.draw()


# ============================================================
# LIVE DATA UPDATE LOOP  (socket → graphs)
# ============================================================
def commit_active_scan():
    """Called when a scan completes — moves active buffer into scan_graph_data."""
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
    """Redraw both graphs, one colored line per completed scan + active scan."""
    ax1.clear(); ax2.clear()
    style_axes()

    # Draw each completed scan in its own color
    for i, sd in enumerate(scan_graph_data):
        color = get_scan_color(i)
        ax1.plot(sd["angles"], sd["ping_vals"], color=color, linewidth=1,
                 marker='o', markersize=2, alpha=0.85, label=f"Scan {i+1}")
        ax2.plot(sd["angles"], sd["ir_vals"],   color=color, linewidth=1,
                 marker='o', markersize=2, alpha=0.85, label=f"Scan {i+1}")

    # Draw the currently incoming scan in white (in-progress)
    if _active_angles:
        ax1.plot(_active_angles, _active_ping_vals, color="#ffffff",
                 linewidth=1, marker='o', markersize=2, alpha=0.6, label="Active")
        ax2.plot(_active_angles, _active_ir_vals,   color="#ffffff",
                 linewidth=1, marker='o', markersize=2, alpha=0.6, label="Active")

    # Show legend if there's anything to show
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

            # --------------------------------------------------
            # MOV: — robot finished a chunk of movement, apply
            # immediately to the live pose so the map updates now.
            # --------------------------------------------------
            if line.startswith("MOV:"):
                cmd = line[4:].strip()   # e.g. "20f", "5r"
                apply_single_movement(cmd)
                terminal.insert(tk.END, f"  [movement applied: {cmd}]\n")
                terminal.see(tk.END)

            # --------------------------------------------------
            # SCAN_START — robot is beginning a scan
            # --------------------------------------------------
            elif line == "\r\nStarting Scan...\r\n":
                _in_scan = True
                _active_angles.clear()
                _active_ping_vals.clear()
                _active_ir_vals.clear()
                terminal.insert(tk.END, "  [scan started]\n")
                terminal.see(tk.END)

            # --------------------------------------------------
            # SCAN_END — scan complete, snapshot live pose into a Scan
            # --------------------------------------------------
            elif line == "SCAN_END":
                _in_scan = False
                if _active_angles:
                    sensor_data = list(zip(
                        _active_angles,
                        _active_ping_vals,
                        _active_ir_vals
                    ))
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

            # --------------------------------------------------
            # angle,ping,ir — sensor reading inside a scan
            # --------------------------------------------------
            else:
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
        if sock:
            sock.close()
    except Exception:
        pass
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# ============================================================
# DEMO  — remove this block once real data is coming in
# ============================================================
def load_demo_scans():
    """
    Feeds fake scans so you can see everything working immediately.
    Delete or comment out this function and its call when using real data.
    """
    import random
    def fake_data():
        return [(a, random.uniform(30, 150), random.randint(200, 2000))
                for a in range(0, 181, 5)]

    demo_scans = [
        ([], fake_data()),
        (["200f", "90r", "100f"], fake_data()),
        (["45l", "150f"], fake_data()),
    ]

    for movements, data in demo_scans:
        for cmd in movements:
            apply_single_movement(cmd)
        ingest_scan(data)
        # also populate graph data so graphs show per-scan colors
        scan_graph_data.append({
            "angles":    [d[0] for d in data],
            "ping_vals": [d[1] for d in data],
            "ir_vals":   [d[2] for d in data],
        })

# load_demo_scans()

# ============================================================
# START
# ============================================================
update_data()
root.mainloop()