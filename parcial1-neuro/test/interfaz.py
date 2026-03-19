#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
🧠 Neurorobot Monitor v18 — Panel Scrollable + IMU + PyBullet + WebSocket
Versión final con todas las correcciones aplicadas
"""
# ==============================================================================
# 1. CONFIGURACIÓN DE ENTORNO
# ==============================================================================
import os
import sys
os.environ['LIBGL_ALWAYS_INDIRECT'] = '1'
os.environ['MESA_GL_VERSION_OVERRIDE'] = '3.3'
os.environ['PYBULLET_USE_GLX'] = '0'
import matplotlib
matplotlib.use('TkAgg')
# ==============================================================================
# 2. IMPORTACIONES
# ==============================================================================
import customtkinter as ctk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import time
import random
import threading
import math
import glob
from datetime import datetime
# PyBullet
try:
    import pybullet as p
    import pybullet_data
    from PIL import Image, ImageTk
    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False
    print("⚠️ PyBullet no disponible")
# WebSocket
try:
    import websocket
    WS_AVAILABLE = True
except ImportError:
    WS_AVAILABLE = False
    print("⚠️ websocket-client no instalado")
# ==============================================================================
# 3. CONFIGURACIÓN DE TEMA Y COLORES
# ==============================================================================
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")
COLOR_BG = "#080c10"
COLOR_PANEL = "#0d1218"
COLOR_ACCENT = "#00d4ff"
COLOR_GREEN = "#00ff88"
COLOR_ORANGE = "#ff8c00"
COLOR_RED = "#ff3344"
COLOR_DIM = "#3a4a5a"
COLOR_TEXT = "#c8d8e8"
# ==============================================================================
# 4. CLASE PRINCIPAL
# ==============================================================================
class NeuralMonitorGUI(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("🧠 Neurorobot Monitor v18 — Panel Scrollable")
        self.geometry("1500x900")
        self.minsize(1300, 800)
        # === ESTADO ===
        self.running = True
        self.mode = "SIMULATION"
        self.sim_running = False
        self.ws_connected = False
        self.ws = None
        self.ws_thread = None
        # === DATOS GRÁFICAS ===
        self.max_points = 300
        self.times = []
        self.neural_activity = []
        self.motor_left = []
        self.motor_right = []
        self.us1_data = []          # ← CRÍTICO: Inicializar
        self.us2_data = []          # ← CRÍTICO: Inicializar
        # === DATOS IMU ===
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        # === PYBULLET ===
        self.pybullet_connected = False
        self.robot_id = None
        self.arena_id = None
        self.view_matrix = None
        self.proj_matrix = None
        self.current_photo = None
        self.sim_view_label = None
        # === SIMULACIÓN NEURONAL ===
        self.A, self.B = 100.0, 120.0
        self.z = np.zeros(8)
        self.z[4] = 0.1
        self.prev_vel_z = 0.0
        self.impact_time = None
        self.pulse_active = False
        self.sim_time = 0.0
        self.update_counter = 0
        self.render_counter = 0
        # === WEBSOCKET ===
        self.ws_url = "ws://192.168.4.1:81/"
        self.ws_retry = 0
        # === UI REFS ===
        self.metric_labels = {}
        self.imu_labels = {}
        self.us_labels = {}
        self.extra_metrics = {}
        self.state_label = None
        self.connection_status = None
        self.log_text = None
        self.setup_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.add_log("🚀 Monitor v18 listo — Panel scrollable + IMU + PyBullet")
    # ==============================================================================
    # 5. INTERFAZ DE USUARIO
    # ==============================================================================
    def setup_ui(self):
        # ── BARRA SUPERIOR ─────────────────────────────────────
        top_bar = ctk.CTkFrame(self, height=50, fg_color=COLOR_PANEL)
        top_bar.pack(fill="x", padx=0, pady=0)
        ctk.CTkLabel(top_bar, text="🧠 NEUROROBOT MONITOR",
                    font=("IBM Plex Mono", 18, "bold"),
                    text_color=COLOR_ACCENT).pack(side="left", padx=20)
        self.mode_label = ctk.CTkLabel(top_bar, text="Modo: SIMULACIÓN",
                                      font=("IBM Plex Mono", 12))
        self.mode_label.pack(side="left", padx=20)
        self.connection_status = ctk.CTkLabel(top_bar, text="● Desconectado",
                                             text_color=COLOR_DIM,
                                             font=("IBM Plex Mono", 12))
        self.connection_status.pack(side="right", padx=20)
        # ── CONTENEDOR PRINCIPAL ──────────────────────────────
        main_container = ctk.CTkFrame(self, fg_color="transparent")
        main_container.pack(fill="both", expand=True, padx=10, pady=10)
        # === COLUMNA A: VISTA 3D + SENSORES + IMU (SCROLLABLE) ===
        col_a = ctk.CTkFrame(main_container, fg_color=COLOR_PANEL, width=450)
        col_a.pack(side="left", fill="both", expand=False, padx=(0, 5))
        col_a.pack_propagate(False)
        ctk.CTkLabel(col_a, text="🎥 Vista 3D / Sensores",
                    font=("IBM Plex Mono", 13, "bold"),
                    text_color=COLOR_TEXT).pack(pady=8)
        # ← PANEL SCROLLABLE
        scrollable_frame = ctk.CTkScrollableFrame(
            col_a,
            width=420,
            fg_color="transparent",
            scrollbar_button_color=COLOR_DIM,
            scrollbar_button_hover_color=COLOR_ACCENT
        )
        scrollable_frame.pack(fill="both", expand=True, padx=5, pady=5)
        # === 1. VISTA PYBULLET ===
        view_container = ctk.CTkFrame(scrollable_frame, fg_color=COLOR_BG)
        view_container.pack(fill="x", padx=5, pady=10)
        if PYBULLET_AVAILABLE:
            self.sim_view_label = ctk.CTkLabel(view_container, text="▶️ Iniciar para ver vista 3D",
                                              fg_color="#050810", text_color=COLOR_DIM,
                                              corner_radius=5, font=("IBM Plex Mono", 10))
            self.sim_view_label.pack(fill="both", expand=True, padx=10, pady=10)
        else:
            ctk.CTkLabel(view_container, text="⚠️ PyBullet no disponible",
                        fg_color="#050810", text_color=COLOR_ORANGE,
                        justify="center").pack(fill="both", expand=True, padx=10, pady=10)
        # === 2. ULTRASONIDOS ===
        us_container = ctk.CTkFrame(scrollable_frame, fg_color=COLOR_BG)
        us_container.pack(fill="x", padx=5, pady=5)
        ctk.CTkLabel(us_container, text="📡 Ultrasonidos",
                    font=("IBM Plex Mono", 11, "bold"),
                    text_color=COLOR_TEXT).pack(pady=8)
        us_grid = ctk.CTkFrame(us_container, fg_color="transparent")
        us_grid.pack(fill="x", padx=10, pady=10)
        for i, (label, side) in enumerate([("US1 IZQ:", "IZQUIERDO"), ("US2 DER:", "DERECHO")]):
            card = ctk.CTkFrame(us_grid, fg_color=COLOR_PANEL, height=50)
            card.grid(row=0, column=i, padx=5, pady=0, sticky="ew")
            us_grid.grid_columnconfigure(i, weight=1)
            ctk.CTkLabel(card, text=label,
                        font=("IBM Plex Mono", 9),
                        text_color=COLOR_DIM).pack(padx=8, pady=(5, 0))
            self.us_labels[f"US{i+1}"] = ctk.CTkLabel(card, text="-- cm",
                                                     font=("IBM Plex Mono", 16, "bold"),
                                                     text_color=COLOR_ACCENT)
            self.us_labels[f"US{i+1}"].pack(padx=8, pady=(0, 5))
        # === 3. IMU ===
        imu_container = ctk.CTkFrame(scrollable_frame, fg_color=COLOR_BG)
        imu_container.pack(fill="x", padx=5, pady=5)
        ctk.CTkLabel(imu_container, text="🧭 IMU (Inercial)",
                    font=("IBM Plex Mono", 11, "bold"),
                    text_color=COLOR_TEXT).pack(pady=8)
        imu_grid = ctk.CTkFrame(imu_container, fg_color="transparent")
        imu_grid.pack(fill="x", padx=10, pady=10)
        for i, axis in enumerate(["ROLL", "PITCH", "YAW"]):
            card = ctk.CTkFrame(imu_grid, fg_color=COLOR_PANEL, height=70)
            card.grid(row=0, column=i, padx=3, pady=0, sticky="ew")
            imu_grid.grid_columnconfigure(i, weight=1)
            icon = "📐" if axis == "ROLL" else "📏" if axis == "PITCH" else "🧭"
            ctk.CTkLabel(card, text=f"{icon} {axis}",
                        font=("IBM Plex Mono", 9),
                        text_color=COLOR_DIM).pack(pady=(8, 0))
            self.imu_labels[axis.lower()] = ctk.CTkLabel(card, text="0.0°",
                                                        font=("IBM Plex Mono", 18, "bold"),
                                                        text_color=COLOR_ACCENT)
            self.imu_labels[axis.lower()].pack(pady=(0, 8))
        # === 4. MÉTRICAS EXTRA ===
        metrics_container = ctk.CTkFrame(scrollable_frame, fg_color=COLOR_BG)
        metrics_container.pack(fill="x", padx=5, pady=5)
        ctk.CTkLabel(metrics_container, text="📊 Métricas Extra",
                    font=("IBM Plex Mono", 11, "bold"),
                    text_color=COLOR_TEXT).pack(pady=8)
        metrics_grid = ctk.CTkFrame(metrics_container, fg_color="transparent")
        metrics_grid.pack(fill="x", padx=10, pady=10)
        self.extra_metrics = {}
        for i, metric in enumerate(["Posición X", "Posición Y", "Batería"]):
            row = i // 2
            col = i % 2
            card = ctk.CTkFrame(metrics_grid, fg_color=COLOR_PANEL, height=45, width=180)
            card.grid(row=row, column=col, padx=3, pady=2, sticky="ew")
            ctk.CTkLabel(card, text=metric,
                        font=("IBM Plex Mono", 9),
                        text_color=COLOR_DIM).pack(padx=8, pady=(5, 0))
            lbl = ctk.CTkLabel(card, text="--",
                              font=("IBM Plex Mono", 13, "bold"),
                              text_color=COLOR_ACCENT)
            lbl.pack(padx=8, pady=(0, 5))
            self.extra_metrics[metric] = lbl
        metrics_grid.grid_columnconfigure(0, weight=1)
        metrics_grid.grid_columnconfigure(1, weight=1)
        # === COLUMNA B: GRÁFICAS ===
        col_b = ctk.CTkFrame(main_container, fg_color="transparent")
        col_b.pack(side="left", fill="both", expand=True, padx=5)
        # Gráfica 1: Neuronal
        neural_frame = ctk.CTkFrame(col_b, fg_color=COLOR_PANEL)
        neural_frame.pack(fill="both", expand=True, pady=(0, 5))
        ctk.CTkLabel(neural_frame, text="🧠 Actividad Neuronal (Z0-Z6 + Mem)",
                    font=("IBM Plex Mono", 12, "bold"),
                    text_color=COLOR_TEXT).pack(pady=5)
        self.fig1, self.ax1 = plt.subplots(figsize=(6, 3.5), facecolor=COLOR_BG)
        self.fig1.patch.set_facecolor(COLOR_BG)
        self.ax1.set_facecolor(COLOR_BG)
        self.ax1.set_xlim(0, 30)
        self.ax1.set_ylim(0, 100)
        self.ax1.set_xlabel('Tiempo (s)', color=COLOR_DIM, fontsize=9)
        self.ax1.set_ylabel('Activación', color=COLOR_DIM, fontsize=9)
        self.ax1.tick_params(colors=COLOR_DIM, labelsize=8)
        self.ax1.grid(True, alpha=0.2, color=COLOR_DIM, linestyle='--')
        self.ax1.spines['top'].set_visible(False)
        self.ax1.spines['right'].set_visible(False)
        colors = ['#185FA5','#1D9E75','#0F6E56','#e03030','#c47a00','#8877ee','#dd5588','#4a9e20']
        labels = ['Z0','Z1','Z2','Z3','Z4','Z5','Z6','Mem']
        self.neural_lines = []
        for c, l in zip(colors, labels):
            line, = self.ax1.plot([], [], c, linewidth=1.5, label=l, alpha=0.9)
            self.neural_lines.append(line)
        self.ax1.legend(facecolor=COLOR_BG, edgecolor='none', labelcolor=COLOR_TEXT,
                       fontsize=7, loc='upper right', ncol=4)
        self.canvas1 = FigureCanvasTkAgg(self.fig1, master=neural_frame)
        self.canvas1.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)
        # Gráfica 2: Motores + US ← CORREGIDO: Crear líneas US aquí (una sola vez)
        motor_frame = ctk.CTkFrame(col_b, fg_color=COLOR_PANEL)
        motor_frame.pack(fill="both", expand=True, pady=(5, 0))
        ctk.CTkLabel(motor_frame, text="⚙️ Motores + Sensores",
                    font=("IBM Plex Mono", 12, "bold"),
                    text_color=COLOR_TEXT).pack(pady=5)
        self.fig2, self.ax2 = plt.subplots(figsize=(6, 3), facecolor=COLOR_BG)
        self.fig2.patch.set_facecolor(COLOR_BG)
        self.ax2.set_facecolor(COLOR_BG)
        self.ax2.set_xlim(0, 30)
        self.ax2.set_ylim(0, 100)
        self.ax2.set_xlabel('Tiempo (s)', color=COLOR_DIM, fontsize=9)
        self.ax2.set_ylabel('Valor', color=COLOR_DIM, fontsize=9)
        self.ax2.tick_params(colors=COLOR_DIM, labelsize=8)
        self.ax2.grid(True, alpha=0.2, color=COLOR_DIM, linestyle='--')
        self.ax2.spines['top'].set_visible(False)
        self.ax2.spines['right'].set_visible(False)
        self.motor_l_line, = self.ax2.plot([], [], '#4488ff', linewidth=2, label='Motor Izq (Z4)')
        self.motor_r_line, = self.ax2.plot([], [], '#ff4444', linewidth=2, label='Motor Der (Z5)')
        # ← CRÍTICO: Crear líneas US una sola vez aquí
        self.us1_line, = self.ax2.plot([], [], '#00d4ff', linewidth=1, label='US1', alpha=0.7)
        self.us2_line, = self.ax2.plot([], [], '#00aa88', linewidth=1, label='US2', alpha=0.7)
        self.ax2.legend(facecolor=COLOR_BG, edgecolor='none', labelcolor=COLOR_TEXT,
                       fontsize=7, loc='upper right')
        self.canvas2 = FigureCanvasTkAgg(self.fig2, master=motor_frame)
        self.canvas2.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)
        # === COLUMNA C: CONTROLES ===
        col_c = ctk.CTkFrame(main_container, width=320, fg_color=COLOR_PANEL)
        col_c.pack(side="right", fill="y", padx=(5, 0))
        col_c.pack_propagate(False)
        # Estado
        state_frame = ctk.CTkFrame(col_c, fg_color="transparent")
        state_frame.pack(fill="x", padx=15, pady=(15, 10))
        ctk.CTkLabel(state_frame, text="🔖 Estado",
                    font=("IBM Plex Mono", 14, "bold"),
                    text_color=COLOR_TEXT).pack(pady=3)
        self.state_label = ctk.CTkLabel(state_frame, text="IDLE",
                                       font=("IBM Plex Mono", 16, "bold"),
                                       text_color=COLOR_DIM)
        self.state_label.pack(pady=5)
        # Métricas
        metrics_frame = ctk.CTkFrame(col_c, fg_color="transparent")
        metrics_frame.pack(fill="x", padx=15, pady=10)
        ctk.CTkLabel(metrics_frame, text="📊 Métricas",
                    font=("IBM Plex Mono", 14, "bold"),
                    text_color=COLOR_TEXT).pack(pady=3)
        for metric in ["Activación Z0", "Aceleración Z", "Vel Prom"]:
            row = ctk.CTkFrame(metrics_frame, fg_color=COLOR_BG, height=32)
            row.pack(fill="x", pady=2)
            row.pack_propagate(False)
            ctk.CTkLabel(row, text=f"{metric}:", width=110, anchor="w",
                        font=("IBM Plex Mono", 10), text_color=COLOR_DIM).pack(side="left", padx=10)
            self.metric_labels[metric] = ctk.CTkLabel(row, text="--", width=70, anchor="e",
                                                     font=("IBM Plex Mono", 11, "bold"),
                                                     text_color=COLOR_ACCENT)
            self.metric_labels[metric].pack(side="right", padx=10)
        # Controles
        control_frame = ctk.CTkFrame(col_c, fg_color="transparent")
        control_frame.pack(fill="x", padx=15, pady=10)
        ctk.CTkLabel(control_frame, text="🎮 Controles",
                    font=("IBM Plex Mono", 14, "bold"),
                    text_color=COLOR_TEXT).pack(pady=3)
        ctk.CTkButton(control_frame, text="▶️ Iniciar", height=38, font=("IBM Plex Mono", 11),
                     command=self.start_robot).pack(fill="x", pady=4)
        ctk.CTkButton(control_frame, text="⏸️ Pausar", height=38, font=("IBM Plex Mono", 11),
                     command=self.pause_robot).pack(fill="x", pady=4)
        ctk.CTkButton(control_frame, text="🔄 Cambiar Modo", height=32, font=("IBM Plex Mono", 10),
                     command=self.switch_mode, fg_color=COLOR_DIM).pack(fill="x", pady=4)
        # WebSocket
        ws_frame = ctk.CTkFrame(col_c, fg_color=COLOR_BG)
        ws_frame.pack(fill="x", padx=15, pady=10)
        ctk.CTkLabel(ws_frame, text="🔌 WebSocket",
                    font=("IBM Plex Mono", 11, "bold"),
                    text_color=COLOR_TEXT).pack(pady=3)
        self.ws_entry = ctk.CTkEntry(ws_frame, placeholder_text="ws://192.168.4.1:81/",
                                    font=("IBM Plex Mono", 9), height=28)
        self.ws_entry.pack(fill="x", padx=8, pady=4)
        self.ws_entry.insert(0, self.ws_url)
        ctk.CTkButton(ws_frame, text="Conectar", height=28, font=("IBM Plex Mono", 9),
                     command=self.connect_hardware, width=80).pack(side="left", padx=8, pady=(0,8))
        ctk.CTkButton(ws_frame, text="Desconectar", height=28, font=("IBM Plex Mono", 9),
                     command=self.disconnect_hardware, width=90, fg_color=COLOR_RED).pack(side="right", padx=8, pady=(0,8))
        # Log
        log_frame = ctk.CTkFrame(col_c, fg_color="transparent")
        log_frame.pack(fill="both", expand=True, padx=15, pady=(10, 15))
        ctk.CTkLabel(log_frame, text="📝 Log",
                    font=("IBM Plex Mono", 12, "bold"),
                    text_color=COLOR_TEXT).pack(pady=2)
        self.log_text = ctk.CTkTextbox(log_frame, font=("IBM Plex Mono", 9),
                                      fg_color="#050810", text_color=COLOR_TEXT,
                                      scrollbar_button_color=COLOR_DIM)
        self.log_text.pack(fill="both", expand=True)
    # ==============================================================================
    # 6. FUNCIONES DE UI
    # ==============================================================================
    def add_log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")
    def update_state_badge(self, state):
        colors = {
            "CRUCERO": COLOR_GREEN,
            "GIRANDO": COLOR_ORANGE,
            "STOP": COLOR_RED,
            "RUGOSO-RECTO": COLOR_ORANGE,
            "IMU-DETECT": "#cc88ff",
            "IMPACTO": COLOR_ORANGE,
        }
        color = colors.get(state.split()[0], COLOR_DIM)
        self.state_label.configure(text=state, text_color=color)
    def update_metrics(self, z0=None, acc_z=None, avg_vel=None):
        if z0 is not None:
            self.metric_labels["Activación Z0"].configure(text=f"{z0:.1f}")
        if acc_z is not None:
            self.metric_labels["Aceleración Z"].configure(text=f"{acc_z:.2f}")
        if avg_vel is not None:
            self.metric_labels["Vel Prom"].configure(text=f"{avg_vel:.1f}%")
    def update_sensors(self, us1=None, us2=None, roll=None, pitch=None, yaw=None):
        """Actualiza sensores US e IMU ← CORREGIDO: Claves en minúsculas"""
        if us1 is not None:
            self.us_labels["US1"].configure(text=f"{us1:.1f} cm")
            color = COLOR_RED if us1 <= 5 else COLOR_ORANGE if us1 <= 20 else COLOR_ACCENT
            self.us_labels["US1"].configure(text_color=color)
        if us2 is not None:
            self.us_labels["US2"].configure(text=f"{us2:.1f} cm")
            color = COLOR_RED if us2 <= 5 else COLOR_ORANGE if us2 <= 20 else COLOR_ACCENT
            self.us_labels["US2"].configure(text_color=color)
        # ← CORREGIDO: Usar "roll", "pitch", "yaw" (minúsculas)
        if roll is not None:
            self.imu_labels["roll"].configure(text=f"{roll:.1f}°")
        if pitch is not None:
            self.imu_labels["pitch"].configure(text=f"{pitch:.1f}°")
        if yaw is not None:
            self.imu_labels["yaw"].configure(text=f"{yaw:.1f}°")
    def update_plots(self, new_data=None):
        """Actualiza gráficas ← CORREGIDO: No crear nuevas líneas"""
        if len(self.times) == 0: return
        window = 30
        start_idx = max(0, len(self.times) - int(window * 10))
        if start_idx >= len(self.times): return
        times_window = np.array(self.times[start_idx:]) / 1000.0
        if len(times_window) > 0:
            # Neuronal (Z0-Z6 + Mem)
            data_series = [self.neural_activity, self.motor_left, self.motor_right,
                          [x*0.3 for x in self.motor_left],
                          self.motor_left, [x*0.8 for x in self.motor_right],
                          [x*0.5 for x in self.neural_activity],
                          [x*0.6 for x in self.motor_left]]
            for i, (line, series) in enumerate(zip(self.neural_lines, data_series)):
                if len(series) > start_idx:
                    line.set_data(times_window, series[start_idx:])
            self.ax1.set_xlim(times_window[0], times_window[0] + window)
            self.canvas1.draw_idle()
            # ← CORREGIDO: Solo actualizar datos, NO crear nuevas líneas
            if len(self.motor_left) > start_idx:
                self.motor_l_line.set_data(times_window, self.motor_left[start_idx:])
                self.motor_r_line.set_data(times_window, self.motor_right[start_idx:])
                # US en escala 0-100
                us1_scaled = [min(100, x*2) for x in self.us1_data[start_idx:]] if self.us1_data else []
                us2_scaled = [min(100, x*2) for x in self.us2_data[start_idx:]] if self.us2_data else []
                if us1_scaled:
                    self.us1_line.set_data(times_window, us1_scaled)
                if us2_scaled:
                    self.us2_line.set_data(times_window, us2_scaled)
                self.ax2.set_xlim(times_window[0], times_window[0] + window)
                self.canvas2.draw_idle()
    # ==============================================================================
    # 7. WEBSOCKET CLIENT
    # ==============================================================================
    def connect_hardware(self):
        if not WS_AVAILABLE:
            self.add_log("❌ websocket-client no instalado: pip install websocket-client")
            return
        if self.mode == "HARDWARE" and self.ws_connected:
            self.add_log("⚠️ Ya conectado al hardware")
            return
        if self.sim_running:
            self.stop_simulation()
        self.mode = "HARDWARE"
        self.mode_label.configure(text="Modo: HARDWARE")
        self.ws_url = self.ws_entry.get()
        self.add_log(f"🔌 Conectando a {self.ws_url}...")
        self.connection_status.configure(text="● Conectando...", text_color=COLOR_ORANGE)
        self.ws_thread = threading.Thread(target=self._ws_connect_loop, daemon=True)
        self.ws_thread.start()
    def _ws_connect_loop(self):
        while self.running and self.mode == "HARDWARE":
            try:
                self.ws = websocket.WebSocketApp(
                    self.ws_url,
                    on_open=self._ws_on_open,
                    on_message=self._ws_on_message,
                    on_error=self._ws_on_error,
                    on_close=self._ws_on_close
                )
                self.ws.run_forever(ping_interval=10, ping_timeout=5)
            except Exception as e:
                self.add_log(f"❌ Error WebSocket: {e}")
            if self.running and self.mode == "HARDWARE" and not self.ws_connected:
                self.ws_retry = min(self.ws_retry + 1, 5)
                self.add_log(f"🔄 Reintentando en {self.ws_retry}s...")
                time.sleep(self.ws_retry)
    def _ws_on_open(self, ws):
        self.ws_connected = True
        self.ws_retry = 0
        self.after(0, lambda: self.connection_status.configure(text="● CONECTADO", text_color=COLOR_GREEN))
        self.after(0, lambda: self.add_log("✅ WebSocket conectado"))
    def _ws_on_message(self, ws, message):
        try:
            data = self._parse_csv(message)
            self.after(0, lambda: self._process_telemetry(data))
        except Exception as e:
            self.add_log(f"⚠️ Error parsing: {e}")
    def _parse_csv(self, raw):
        out = {}
        if not raw or not isinstance(raw, str):
            return out
        for tok in raw.strip().split(','):
            if ':' in tok:
                key, val = tok.split(':', 1)
                out[key.strip()] = val.strip()
        return out
    def _process_telemetry(self, data):
        if 'ST' in data:
            self.update_state_badge(data['ST'])
        us1 = float(data['US1']) if 'US1' in data else None
        us2 = float(data['US2']) if 'US2' in data else None
        self.update_sensors(us1=us1, us2=us2)
        if us1 is not None: self.us1_data.append(us1)
        if us2 is not None: self.us2_data.append(us2)
        roll = float(data['R']) if 'R' in data else None
        pitch = float(data['P']) if 'P' in data else None
        yaw = float(data['Y']) if 'Y' in data else None
        self.update_sensors(roll=roll, pitch=pitch, yaw=yaw)
        z0 = float(data['Z0']) if 'Z0' in data else None
        if z0 is not None:
            self.times.append(time.time() * 1000)
            self.neural_activity.append(z0)
            self.motor_left.append(z0 * 0.8)
            self.motor_right.append(z0 * 0.7)
            if len(self.times) > self.max_points:
                for lst in [self.times, self.neural_activity, self.motor_left, self.motor_right, self.us1_data, self.us2_data]:
                    if lst: lst.pop(0)
            avg_vel = (self.motor_left[-1] + self.motor_right[-1]) / 2 if self.motor_left else 0
            self.update_metrics(z0=z0, avg_vel=avg_vel)
            self.update_plots()
    def _ws_on_error(self, ws, error):
        self.add_log(f"⚠️ WS Error: {error}")
    def _ws_on_close(self, ws, close_status_code, close_msg):
        self.ws_connected = False
        self.after(0, lambda: self.connection_status.configure(text="● DESCONECTADO", text_color=COLOR_DIM))
        if self.running and self.mode == "HARDWARE":
            self.add_log("🔌 WebSocket cerrado — reconectando...")
    def disconnect_hardware(self):
        self.mode = "SIMULATION"
        self.mode_label.configure(text="Modo: SIMULACIÓN")
        if self.ws:
            self.ws.close()
        self.ws_connected = False
        self.connection_status.configure(text="● Desconectado", text_color=COLOR_DIM)
        self.add_log("🔌 Desconectado del hardware")
    # ==============================================================================
    # 8. SIMULACIÓN PYBULLET
    # ==============================================================================
    def start_robot(self):
        if self.mode == "SIMULATION" and not self.sim_running:
            if PYBULLET_AVAILABLE:
                self.start_simulation()
            else:
                self.add_log("⚠️ PyBullet no disponible — usa modo HARDWARE")
        elif self.mode == "HARDWARE":
            self.connect_hardware()
    def pause_robot(self):
        if self.sim_running:
            self.stop_simulation()
            self.add_log("⏸️ Simulación pausada")
        elif self.ws_connected:
            self.disconnect_hardware()
    def switch_mode(self):
        if self.sim_running:
            self.stop_simulation()
        if self.ws_connected:
            self.disconnect_hardware()
        self.mode = "HARDWARE" if self.mode == "SIMULATION" else "SIMULATION"
        self.mode_label.configure(text=f"Modo: {self.mode}")
        self.add_log(f"🔄 Cambiado a modo {self.mode}")
    def start_simulation(self):
        if not PYBULLET_AVAILABLE:
            self.add_log("❌ PyBullet no disponible")
            return
        self.force_pybullet_cleanup()
        try:
            self.times = []
            self.neural_activity = []
            self.motor_left = []
            self.motor_right = []
            self.us1_data = []
            self.us2_data = []
            self.update_state_badge("NORMAL")
            self.connection_status.configure(text="● Simulando", text_color=COLOR_GREEN)
            p.connect(p.DIRECT)
            self.pybullet_connected = True
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            p.loadURDF("plane.urdf")
            espesor = 0.01
            coll = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.5, 1.5, espesor])
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[1.5, 1.5, espesor],
                                     rgbaColor=[0.8, 0.6, 0.3, 1])
            self.arena_id = p.createMultiBody(0, coll, vis, basePosition=[3, 0, espesor])
            p.changeDynamics(self.arena_id, -1, lateralFriction=2.0)
            try:
                self.robot_id = p.loadURDF("r2d2.urdf", [1, 0, 0.3],
                                          p.getQuaternionFromEuler([0, 0, 1.57]))
            except:
                coll = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.08, 0.05])
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.08, 0.05],
                                         rgbaColor=[0.2, 0.6, 0.9, 1])
                self.robot_id = p.createMultiBody(0.5, coll, vis, basePosition=[1, 0, 0.3])
                self.add_log("⚠️ Usando robot fallback (cubo)")
            self.view_matrix = p.computeViewMatrix([1.5, -3.0, 2.5], [3.0, 0.0, 0.5], [0, 0, 1])
            self.proj_matrix = p.computeProjectionMatrixFOV(60, 1.77, 0.1, 100.0)
            self.z = np.zeros(8)
            self.z[4] = 0.1
            self.prev_vel_z = 0.0
            self.impact_time = None
            self.pulse_active = False
            self.sim_time = 0.0
            self.update_counter = 0
            self.render_counter = 0
            self.sim_running = True
            if self.sim_view_label:
                self.sim_view_label.configure(text="", image="")
            self.add_log("✅ Simulación matemática + PyBullet iniciada")
            self.simulation_step()
        except Exception as e:
            self.add_log(f"❌ Error simulación: {str(e)}")
            self.stop_simulation()
    def simulation_step(self):
        if not self.sim_running or not self.pybullet_connected: return
        try:
            dt = 1.0 / 240.0
            base_noise = random.gauss(0, 0.5)
            current_vel_z = base_noise * 0.1
            acc_z = (current_vel_z - self.prev_vel_z) / dt
            self.prev_vel_z = current_vel_z
            if self.impact_time is None and random.random() < 0.005 and self.sim_time > 2.0:
                self.impact_time = self.sim_time
                self.pulse_active = True
                self.update_state_badge("⚠️ IMPACTO")
                self.add_log("💥 Impacto detectado (Simulado)")
            sx = 60.0 if (self.pulse_active and (self.sim_time - self.impact_time) <= 0.6) else 0.0
            if self.pulse_active and (self.sim_time - self.impact_time) > 0.6:
                self.pulse_active = False
                self.update_state_badge("CRUCERO")
            dt_sub = 0.1
            steps = max(1, int((dt * 1000) / dt_sub))
            for _ in range(steps):
                self.z[0] += (dt_sub/20.0) * (-self.z[0] + (self.A*(sx+3.0*self.z[2])**2) /
                            ((self.B+1.0*self.z[1])**2 + (sx+3.0*self.z[2])**2))
                self.z[1] += (dt_sub/12000.0) * (-self.z[1] + 0.7*self.z[0])
                self.z[2] += (dt_sub/20.0) * (-self.z[2] + (self.A*(3.0*self.z[0])**2) /
                            ((self.B+1.0*self.z[3])**2 + (3.0*self.z[0])**2))
                self.z[3] += (dt_sub/12000.0) * (-self.z[3] + 0.7*self.z[2])
                znrom = self.z[0] / self.A
                in_A = 160.0 + znrom * 120.0
                in_B = 160.0 + znrom * 80.0
                self.z[4] += (dt_sub/20.0) * (-self.z[4] + (self.A*(in_A-3.2*self.z[5])**2) /
                            ((self.B+self.z[6])**2 + (in_A-3.2*self.z[5])**2))
                self.z[6] += (dt_sub/600.0) * (-self.z[6] + 1.5*self.z[4])
                self.z[5] += (dt_sub/20.0) * (-self.z[5] + (self.A*(in_B-3.2*self.z[4])**2) /
                            ((self.B+self.z[7])**2 + (in_B-3.2*self.z[4])**2))
                self.z[7] += (dt_sub/600.0) * (-self.z[7] + 1.5*self.z[5])
            v_l = 10.0 if self.sim_time < 2.0 else 25.0 * (self.z[4] / self.A)
            v_r = 4.0 if self.sim_time < 2.0 else 25.0 * (self.z[5] / self.A)
            if self.robot_id:
                p.setJointMotorControl2(self.robot_id, 2, p.VELOCITY_CONTROL,
                                       targetVelocity=v_r, force=100)
                p.setJointMotorControl2(self.robot_id, 3, p.VELOCITY_CONTROL,
                                       targetVelocity=v_l, force=100)
                p.stepSimulation()
            if self.render_counter >= 10 and self.sim_view_label and PYBULLET_AVAILABLE:
                try:
                    _, _, rgba, _, _ = p.getCameraImage(640, 360,
                        self.view_matrix, self.proj_matrix,
                        shadow=True, renderer=p.ER_TINY_RENDERER)
                    img = Image.fromarray(np.array(rgba).reshape(360, 640, 4)[:, :, :3])
                    img = img.resize((480, 270), Image.LANCZOS)
                    photo = ImageTk.PhotoImage(image=img)
                    self.sim_view_label.configure(image=photo, text="")
                    self.current_photo = photo
                    self.render_counter = 0
                except: pass
            self.sim_time += dt
            self.update_counter += 1
            self.render_counter += 1
            if self.update_counter >= 30:
                self.times.append(self.sim_time * 1000)
                self.neural_activity.append(self.z[0])
                self.motor_left.append(self.z[4])
                self.motor_right.append(self.z[5])
                self.us1_data.append(15 + random.gauss(0, 3))
                self.us2_data.append(18 + random.gauss(0, 3))
                if len(self.times) > self.max_points:
                    for lst in [self.times, self.neural_activity, self.motor_left,
                               self.motor_right, self.us1_data, self.us2_data]:
                        lst.pop(0)
                avg_vel = (self.z[4] + self.z[5]) / 2
                self.update_metrics(z0=self.z[0], acc_z=acc_z, avg_vel=avg_vel)
                self.update_sensors(us1=self.us1_data[-1], us2=self.us2_data[-1],
                                  roll=random.gauss(0,1), pitch=random.gauss(0,1), yaw=random.gauss(0,2))
                self.update_plots()
                self.update_counter = 0
            self.after(15, self.simulation_step)
        except Exception as e:
            self.add_log(f"❌ Error loop: {str(e)}")
            self.stop_simulation()
    def stop_simulation(self):
        self.sim_running = False
        if self.pybullet_connected:
            try:
                if p.isConnected(): p.disconnect()
            except: pass
            self.pybullet_connected = False
            self.force_pybullet_cleanup()
        if self.sim_view_label:
            self.sim_view_label.configure(image="", text="▶️ Simulación detenida",
                                         fg_color="#050810")
            self.current_photo = None
        self.connection_status.configure(text="● Detenido", text_color=COLOR_DIM)
        self.add_log("⏹️ Simulación detenida")
    def force_pybullet_cleanup(self):
        try:
            if p.isConnected():
                p.disconnect()
                time.sleep(0.3)
        except: pass
        try:
            for f in glob.glob('/tmp/pybullet_*'):
                os.remove(f)
        except: pass
    # ==============================================================================
    # 9. CIERRE Y UTILIDADES
    # ==============================================================================
    def on_closing(self):
        self.running = False
        if self.sim_running:
            self.stop_simulation()
        if self.ws_connected and self.ws:
            self.ws.close()
        try:
            plt.close(self.fig1)
            plt.close(self.fig2)
        except: pass
        self.destroy()
        sys.exit(0)
    def save_data(self):
        try:
            import csv
            filename = f"neurorobot_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Time_ms', 'Z0', 'Z4', 'Z5', 'US1', 'US2'])
                for i in range(len(self.times)):
                    writer.writerow([
                        self.times[i],
                        self.neural_activity[i] if i < len(self.neural_activity) else 0,
                        self.motor_left[i] if i < len(self.motor_left) else 0,
                        self.motor_right[i] if i < len(self.motor_right) else 0,
                        self.us1_data[i] if i < len(self.us1_data) else 0,
                        self.us2_data[i] if i < len(self.us2_data) else 0,
                    ])
            self.add_log(f"💾 Datos guardados: {filename}")
        except Exception as e:
            self.add_log(f"❌ Error guardando: {e}")
# ==============================================================================
# 10. PUNTO DE ENTRADA
# ==============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("🧠 Neurorobot Monitor v18 — Panel Scrollable")
    print("=" * 60)
    print(f"PyBullet disponible: {PYBULLET_AVAILABLE}")
    print(f"WebSocket disponible: {WS_AVAILABLE}")
    print("=" * 60)
    app = NeuralMonitorGUI()
    app.mainloop()