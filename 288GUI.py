import socket
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ---- SOCKET SETUP ----
HOST = "192.168.1.1"
PORT = 288

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
sock.setblocking(False)

recv_buffer = ""

# ---- DATA STORAGE ----
angles = []
ping_vals = []
ir_vals = []

# ---- GUI SETUP ----
root = tk.Tk()
root.title("Sensor GUI")

# ---- BUTTON FUNCTIONS ----
def start_scan():
    sock.sendall(b'm')
    terminal.insert(tk.END, "Sent: m\n")
    terminal.see(tk.END)

def stop_scan():
    sock.sendall(b's')
    terminal.insert(tk.END, "Sent: s\n")
    terminal.see(tk.END)

def clear_graph():
    angles.clear()
    ping_vals.clear()
    ir_vals.clear()
    ax1.clear()
    ax2.clear()
    canvas.draw()

def move_forward():
    pass

def move_backward():
    pass

def move_right():
    pass

def move_left():
    pass


# ---- BUTTONS ----
btn_frame = tk.Frame(root)
btn_frame.pack()

tk.Button(btn_frame, text="Start Scan", command=start_scan).pack(side=tk.LEFT, padx=5)
tk.Button(btn_frame, text="Stop Scan", command=stop_scan).pack(side=tk.LEFT, padx=5)
tk.Button(btn_frame, text="Clear Graph", command=clear_graph).pack(side=tk.LEFT, padx=5)

tk.Button(btn_frame, text="Move Forward", command=move_forward).pack(side=tk.RIGHT, padx=5)
tk.Button(btn_frame, text="Move Backward", command=move_backward).pack(side=tk.RIGHT, padx=5)
tk.Button(btn_frame, text="Move Right", command=move_right).pack(side=tk.RIGHT, padx=5)
tk.Button(btn_frame, text="Move Left", command=move_left).pack(side=tk.RIGHT, padx=5)


# ---- TERMINAL OUTPUT ----
terminal = ScrolledText(root, height=8)
terminal.pack(fill=tk.BOTH, padx=10, pady=5)

# ---- MATPLOTLIB FIGURE ----
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 5))

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# ---- UPDATE LOOP ----
def update_data():
    global recv_buffer

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

        if line:
            terminal.insert(tk.END, line + "\n")
            terminal.see(tk.END)

            parts = line.split(',')
            if len(parts) == 3:
                try:
                    angle = float(parts[0])
                    ping = float(parts[1])
                    ir = float(parts[2])

                    angles.append(angle)
                    ping_vals.append(ping)
                    ir_vals.append(ir)
                except ValueError:
                    pass

    # redraw graphs
    ax1.clear()
    ax2.clear()

    ax1.plot(angles, ping_vals, marker='o')
    ax1.set_title("PING Sensor Data")
    ax1.set_xlabel("Angle")
    ax1.set_ylabel("Distance (cm)")

    ax2.plot(angles, ir_vals, marker='o')
    ax2.set_title("IR Sensor Data")
    ax2.set_xlabel("Angle")
    ax2.set_ylabel("Raw ADC")

    canvas.draw()

    root.after(100, update_data)

def on_close():
    try:
        sock.close()
    except:
        pass
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)

# start update loop
update_data()

root.mainloop()