"""
Plot realtime MPU6050 (IMU) + simulasi 3D orientasi dari STM32 Blue Pill
yang mengirim data lewat MAVLink v2 (USB CDC / virtual COM port).

Pesan yang dibaca:
    SCALED_IMU  -> acc [mG], gyro [mrad/s], suhu [cdegC]
    ATTITUDE    -> roll, pitch [rad]
    STATUSTEXT  -> pesan status / error dari STM32 (ditampilkan di panel 3D & konsol)
    HEARTBEAT   -> deteksi koneksi

Pemakaian:
    pip install pymavlink pyserial numpy matplotlib
    python plot_mavlink.py --list-ports           # lihat daftar port serial
    python plot_mavlink.py COM5                   # Windows
    python plot_mavlink.py /dev/ttyACM0           # Linux
    python plot_mavlink.py COM5 --window 20
    python plot_mavlink.py COM5 --flip-roll --flip-pitch
    python plot_mavlink.py udpin:127.0.0.1:14550  # string koneksi pymavlink lain juga bisa

Catatan:
    - Port COM hanya bisa dibuka satu program. TUTUP PuTTY (dan Mission Planner /
      QGroundControl) sebelum menjalankan script ini.
    - Baudrate tidak berpengaruh pada USB CDC (default 115200).
    - Sumbu waktu memakai time_boot_ms dari STM32 (akurat, tidak tergantung PC).
    - MPU6050 tanpa magnetometer tidak punya yaw stabil, jadi yaw dianggap 0.
"""
import argparse
import math
import os
import time
from collections import deque

os.environ.setdefault("MAVLINK20", "1")     # harus sebelum import pymavlink

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pymavlink import mavutil

NO_DATA_TIMEOUT = 3.0       # detik tanpa pesan -> dianggap tidak terhubung
MAX_MSGS_PER_FRAME = 300


# ---------------------------------------------------------------------------
# Geometri 3D
# ---------------------------------------------------------------------------
def rotation_matrix(roll_deg, pitch_deg):
    """Matriks rotasi dari roll (sumbu X) dan pitch (sumbu Y). Yaw = 0."""
    r, p = np.radians(roll_deg), np.radians(pitch_deg)
    rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    return ry @ rx


# Kotak mewakili papan sensor: panjang(X) 1.6, lebar(Y) 1.0, tebal(Z) 0.2
HALF = np.array([0.8, 0.5, 0.1])
VERTS = np.array([[sx, sy, sz]
                  for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)],
                 dtype=float) * HALF
FACES = [
    [0, 1, 3, 2],  # bawah (z-)
    [4, 5, 7, 6],  # atas  (z+)
    [0, 1, 5, 4],  # depan (y-)
    [2, 3, 7, 6],  # belakang (y+)
    [0, 2, 6, 4],  # kiri  (x-)
    [1, 3, 7, 5],  # kanan (x+)
]
FACE_COLORS = ["#7f8c8d", "#3498db", "#95a5a6", "#95a5a6", "#bdc3c7", "#e67e22"]
AXIS_LEN = 1.3
AXIS_COLORS = ("red", "green", "blue")


def box_faces(R):
    v = VERTS @ R.T
    return [[v[i] for i in face] for face in FACES]


def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        raise SystemExit("pyserial belum terpasang: pip install pyserial")
    ports = list(lp.comports())
    if not ports:
        print("Tidak ada port serial terdeteksi.")
    for p in ports:
        print(f"{p.device:12s} {p.description}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Realtime plot IMU + 3D dari MAVLink (STM32)")
    ap.add_argument("port", nargs="?", help="port serial (COM5, /dev/ttyACM0) atau string pymavlink")
    ap.add_argument("--list-ports", action="store_true", help="tampilkan daftar port lalu keluar")
    ap.add_argument("--baud", type=int, default=115200, help="baudrate (diabaikan USB CDC)")
    ap.add_argument("--window", type=float, default=30.0, help="lebar jendela waktu (detik)")
    ap.add_argument("--interval", type=int, default=50, help="interval refresh plot (ms)")
    ap.add_argument("--flip-roll", action="store_true", help="balik arah roll model 3D")
    ap.add_argument("--flip-pitch", action="store_true", help="balik arah pitch model 3D")
    args = ap.parse_args()

    if args.list_ports:
        list_ports()
        return
    if not args.port:
        ap.error("port belum diisi. Pakai --list-ports untuk melihat port yang ada.")

    try:
        conn = mavutil.mavlink_connection(args.port, baud=args.baud)
    except Exception as e:                      # noqa: BLE001
        raise SystemExit(f"Gagal membuka {args.port}: {e}\n"
                         "Pastikan port benar dan program lain (PuTTY, dll.) sudah ditutup.")

    maxlen = 5000
    t_imu = deque(maxlen=maxlen)
    t_att = deque(maxlen=maxlen)
    imu = {k: deque(maxlen=maxlen) for k in ("ax", "ay", "az", "gx", "gy", "gz", "temp")}
    att = {k: deque(maxlen=maxlen) for k in ("roll", "pitch")}
    state = {"last_rx": None, "last_imu_ms": None, "last_att_ms": None,
             "status": "", "hb": False}

    # ---- Layout: grafik 2x2 di kiri, panel 3D di kanan ----
    fig = plt.figure(figsize=(14, 7))
    fig.suptitle("STM32 Blue Pill - MPU6050 via MAVLink")
    gs = GridSpec(2, 3, figure=fig, width_ratios=[1, 1, 1.1])

    ax_acc = fig.add_subplot(gs[0, 0])
    ax_gyr = fig.add_subplot(gs[0, 1], sharex=ax_acc)
    ax_ang = fig.add_subplot(gs[1, 0], sharex=ax_acc)
    ax_tmp = fig.add_subplot(gs[1, 1], sharex=ax_acc)
    ax3d = fig.add_subplot(gs[:, 2], projection="3d")

    # (axes, judul, satuan, key, label, sumber waktu, sumber data)
    plots = [
        (ax_acc, "Accelerometer", "g", [("ax", "X"), ("ay", "Y"), ("az", "Z")], t_imu, imu),
        (ax_gyr, "Gyroscope", "deg/s", [("gx", "X"), ("gy", "Y"), ("gz", "Z")], t_imu, imu),
        (ax_ang, "Roll & Pitch", "derajat", [("roll", "Roll"), ("pitch", "Pitch")], t_att, att),
        (ax_tmp, "Temperatur MPU6050", "°C", [("temp", "MPU6050")], t_imu, imu),
    ]

    lines = []      # (Line2D, t_deque, data_deque)
    for ax, title, unit, series, tq, dq in plots:
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.grid(True, alpha=0.3)
        for key, label in series:
            (ln,) = ax.plot([], [], label=label)
            lines.append((ln, tq, dq[key]))
        if len(series) > 1:
            ax.legend(loc="upper left", fontsize=8)
    ax_ang.set_xlabel("Waktu (s)")
    ax_tmp.set_xlabel("Waktu (s)")

    # ---- Panel 3D ----
    ax3d.set_title("Orientasi 3D (roll & pitch)")
    ax3d.set_xlim(-1.5, 1.5)
    ax3d.set_ylim(-1.5, 1.5)
    ax3d.set_zlim(-1.5, 1.5)
    ax3d.set_box_aspect((1, 1, 1))
    ax3d.set_xlabel("X")
    ax3d.set_ylabel("Y")
    ax3d.set_zlabel("Z")
    ax3d.view_init(elev=22, azim=-60)

    poly = Poly3DCollection(box_faces(np.eye(3)), facecolors=FACE_COLORS,
                            edgecolors="black", linewidths=1.0, alpha=0.9)
    ax3d.add_collection3d(poly)

    axis_lines = []
    for i, color in enumerate(AXIS_COLORS):
        tip = np.zeros(3)
        tip[i] = AXIS_LEN
        (ln,) = ax3d.plot([0, tip[0]], [0, tip[1]], [0, tip[2]], color=color, linewidth=2)
        axis_lines.append(ln)
    angle_text = ax3d.text2D(0.02, 0.97, "", transform=ax3d.transAxes, fontsize=9, va="top")
    status_text = ax3d.text2D(0.02, 0.91, "", transform=ax3d.transAxes, fontsize=8,
                              va="top", color="firebrick", wrap=True)

    sign_r = -1.0 if args.flip_roll else 1.0
    sign_p = -1.0 if args.flip_pitch else 1.0

    def clear_all():
        for q in (t_imu, t_att, *imu.values(), *att.values()):
            q.clear()
        state["last_imu_ms"] = state["last_att_ms"] = None

    def handle(msg):
        typ = msg.get_type()
        if typ == "SCALED_IMU":
            ms = msg.time_boot_ms
            if state["last_imu_ms"] is not None and ms < state["last_imu_ms"]:
                clear_all()                 # STM32 reset -> grafik baru
            state["last_imu_ms"] = ms
            t_imu.append(ms / 1000.0)
            imu["ax"].append(msg.xacc / 1000.0)
            imu["ay"].append(msg.yacc / 1000.0)
            imu["az"].append(msg.zacc / 1000.0)
            imu["gx"].append(math.degrees(msg.xgyro / 1000.0))
            imu["gy"].append(math.degrees(msg.ygyro / 1000.0))
            imu["gz"].append(math.degrees(msg.zgyro / 1000.0))
            imu["temp"].append(getattr(msg, "temperature", 0) / 100.0)
        elif typ == "ATTITUDE":
            ms = msg.time_boot_ms
            if state["last_att_ms"] is not None and ms < state["last_att_ms"]:
                clear_all()
            state["last_att_ms"] = ms
            t_att.append(ms / 1000.0)
            att["roll"].append(math.degrees(msg.roll))
            att["pitch"].append(math.degrees(msg.pitch))
        elif typ == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            text = text.rstrip("\x00")
            print(f"[STM32] {text}")
            state["status"] = text
        elif typ == "HEARTBEAT":
            state["hb"] = True

    def update(_frame):
        for _ in range(MAX_MSGS_PER_FRAME):
            msg = conn.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() == "BAD_DATA":
                continue
            state["last_rx"] = time.monotonic()
            handle(msg)

        # --- grafik 2D ---
        t_ref = max([q[-1] for q in (t_imu, t_att) if q], default=None)
        if t_ref is not None:
            for ln, tq, dq in lines:
                n = min(len(tq), len(dq))
                ln.set_data(list(tq)[:n], list(dq)[:n])
            ax_acc.set_xlim(max(0.0, t_ref - args.window) if t_ref > args.window else 0.0,
                            max(args.window, t_ref))
            for ax, *_ in plots:
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)

        # --- panel 3D + status ---
        connected = (state["last_rx"] is not None and
                     time.monotonic() - state["last_rx"] < NO_DATA_TIMEOUT)

        if att["roll"] and connected:
            roll, pitch = att["roll"][-1], att["pitch"][-1]
            R = rotation_matrix(sign_r * roll, sign_p * pitch)
            poly.set_verts(box_faces(R))
            for i, ln in enumerate(axis_lines):
                tip = R[:, i] * AXIS_LEN
                ln.set_data_3d([0, tip[0]], [0, tip[1]], [0, tip[2]])
            angle_text.set_text(f"Roll: {roll:.1f}°  Pitch: {pitch:.1f}°")
        elif not connected:
            angle_text.set_text("Tidak ada data MAVLink (cek port / STM32)"
                                if state["last_rx"] is not None
                                else "Menunggu data MAVLink...")
        else:
            angle_text.set_text("Terhubung, menunggu data IMU...")

        status_text.set_text(f"STM32: {state['status']}" if state["status"] else "")
        return [ln for ln, *_ in lines]

    _anim = FuncAnimation(fig, update, interval=args.interval,
                          blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()