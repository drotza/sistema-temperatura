# Activa el entorno virtual antes de ejecutar:
#   ..\.venv\Scripts\Activate.ps1      (estando en la carpeta Python)
#   python Python/control_gui.py COM 6

# Los CSV y PNG se guardan en ..\CSV_PNG
#
# Interfaz del control PID de temperatura (Parte 2). El PID corre en el Arduino
# (sketch PIArdController.ino); esta ventana grafica, cambia la referencia y las ganancias,
# y guarda todo en un CSV + PNG al cerrar.
# Todo va en unidades crudas: temperatura y referencia en cuentas del ADC (0..1023),
# PWM en cuentas (0..255). Con el termistor arriba del divisor, mas caliente = mas cuentas.
# Uso:
#   python control_gui.py COM6     -> con el Arduino
#   python control_gui.py FAKE     -> planta SIMULADA en tiempo real (modelo ajustado a datos reales)
#   python control_gui.py FAKE 10  -> lo mismo, 10x mas rapido
#   python control_gui.py REPLAY [archivo.csv]
#        -> reproduce en tiempo real una corrida guardada, desde que ya esta estable tras
#           apagarse, con el calentador siempre apagado (por defecto la del 23/10 9:17)
#   python control_gui.py VACIA    -> solo abre la interfaz, sin datos ni Arduino (no guarda nada)
#
# Durante la corrida:
#   Switch CALENTADOR (on.png / off.png): habilita / apaga. La salida real es (control AND habilitado);
#                     arranca APAGADO, asi que hay que habilitarlo para que caliente
#   Perilla "Temp Deseada" (knob.png): arrastrala o usa la rueda (se envia al dejar de moverla)
#   Lazo cerrado / Lazo abierto: compara el PID contra un PWM fijo
#   Kp, Ki, Kd + Aplicar: cambia las ganancias en caliente (sin salto en la salida)
#   Marcar perturbacion: deja una marca en la grafica
#                        (en FAKE ademas aplica una perturbacion de -60 PWM durante 60 s)
#   Para terminar: cerrar la ventana (apaga el calentador y guarda)
#
# CSV: t,PWM,Temp  ->  t [s] cada 0.5 s (tiempo del Arduino), PWM 0..255 (lo que sale
#      por D3), Temp en cuentas de ADC
#
# Requiere: pip install pyserial matplotlib tkdial
import sys, os, time, datetime, math, random, threading, queue
import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tkdial import ImageKnob

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
FAKE = PORT.upper() == "FAKE"
REPLAY = PORT.upper() == "REPLAY"
VACIA = PORT.upper() == "VACIA"
SPEED = float(sys.argv[2]) if FAKE and len(sys.argv) > 2 else 1.0   # velocidad del FAKE
REPLAY_CSV = sys.argv[2] if REPLAY and len(sys.argv) > 2 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "CSV_PNG", "pid_20261023_091721.csv")
HEADER = "t_s,ref_adc,adc,pwm,modo,hab"

REF0 = 180                         # referencia inicial [cuentas ADC] (~43 °C)
KP0, KI0, KD0 = 9.4, 0.157, 0.0    # PI por SIMC (tc=10 s) con G(s)=1.047e^(-5s)/(148s+1)
ADC_MIN, ADC_MAX = 50, 500         # rango de la perilla (~10 °C a ~84 °C con R2 = 1k)

BG, INK, BLUE = "#ffffff", "#1e1e1e", "#1971c2"
GREEN, RED, GRAY = "#2f9e44", "#e03131", "#868e96"

stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
OUT_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "CSV_PNG"))
os.makedirs(OUT_DIR, exist_ok=True)
base  = os.path.join(OUT_DIR, f"pid_{'FAKE_' if FAKE else 'REPLAY_' if REPLAY else ''}{stamp}")
fname, pngname = base + ".csv", base + ".png"

q = queue.Queue()                  # lineas que llegan (hilo lector -> interfaz)
stop = threading.Event()


# ---------------- Enlace con el Arduino ----------------
class SerialLink:
    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        # El Uno se reinicia al abrir el puerto: esperar a que diga "# Listo"
        print("Esperando al Arduino...")
        t_lim = time.time() + 8
        while time.time() < t_lim:
            l = self.ser.readline().decode(errors="ignore").strip()
            if l:
                print(l)
            if l.startswith("# Listo"):
                break
        else:
            print("AVISO: el Arduino no respondio '# Listo'. Revisa que PIArdController.ino este "
                  "cargado y que el Monitor Serial este cerrado. Se intentara de todos modos.")
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        while not stop.is_set():
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                break
            if line:
                q.put(line)

    def write(self, cmd):
        self.ser.write((cmd + "\n").encode())

    def close(self):
        try:
            self.ser.write(b"X\n")         # apaga el calentador
            time.sleep(0.1)
            self.ser.close()
        except Exception:
            pass


class PID:
    """La misma ley que PIArdController.ino (solo se usa en la simulacion)."""
    def __init__(self, Ts):
        self.Ts, self.kp, self.ki, self.kd = Ts, KP0, KI0, KD0
        self.I, self.dF, self.y_prev = 0.0, 0.0, None

    def step(self, ref, y, auto, u_man, hab):
        e = ref - y
        if self.y_prev is not None:
            self.dF += 0.3 * (-(y - self.y_prev) / self.Ts - self.dF)
        self.y_prev = y
        if not auto:
            return u_man
        v = self.kp * e + self.I + self.kd * self.dF
        u = min(max(v, 0.0), 255.0)
        if hab and not ((v >= 255 and e > 0) or (v <= 0 and e < 0)):
            self.I += self.ki * self.Ts * e
        self.I = min(max(self.I, -255.0), 255.0)
        return u

    def bumpless(self, u, ref, y):
        self.I = min(max(u - self.kp * (ref - y) - self.kd * self.dF, -255.0), 255.0)


class FakeLink:
    """Arduino simulado con un modelo ajustado a corridas reales (escalon de lazo abierto
       y pulso del 23/10 9:17), todo en cuentas de ADC y PWM:
         tau_c * dx1/dt = -x1 + K * pwm(t - theta)     (bloque del calentador)
         tau_s * dx2/dt = x1 - x2                      (retraso del termistor)
         adc = Ta + x2 + brinco*pwm + ruido, redondeado a 1/16 (promedio de 16 lecturas)
       'brinco' es el salto de lectura al encender (la corriente mueve la tierra comun).
       Arranca en reposo en Ta, el valor estable al que regreso esa corrida tras apagarse."""
    Ta, K, tau_c, tau_s, theta = 84.9, 1.214, 193.0, 2.5, 3.2
    BRINCO, RUIDO = 0.0129, 0.1

    def __init__(self, Ts=0.5, speed=1.0):
        self.Ts, self.speed = Ts, speed
        self.x1 = self.x2 = 0.0
        self.ref, self.u, self.u_man = REF0, 0.0, 0.0
        self.auto, self.hab, self.logging, self.k0, self.k = True, False, False, 0, 0
        self.hist, self.dist_until = [], -1.0
        self.pid = PID(Ts)
        self.y = self.Ta
        q.put("# Listo (simulado)")
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        n_delay = round(self.theta / self.Ts)
        sig = time.perf_counter()
        while not stop.is_set():
            self.k += 1
            self.u = self.pid.step(self.ref, self.y, self.auto, self.u_man, self.hab)
            out = round(self.u) if self.hab else 0          # control AND habilitado
            self.hist.append(out)
            u_del = self.hist[-n_delay - 1] if len(self.hist) > n_delay else 0
            dist = -60.0 if self.k * self.Ts < self.dist_until else 0.0   # ~soplarle
            self.x1 += self.Ts / self.tau_c * (-self.x1 + self.K * (u_del + dist))
            self.x2 += self.Ts / self.tau_s * (self.x1 - self.x2)
            y = self.Ta + self.x2 + self.BRINCO * out + random.gauss(0, self.RUIDO)
            self.y = round(y * 16) / 16
            if self.logging:
                t = (self.k - self.k0) * self.Ts
                q.put(f"{t:.1f},{self.ref:.0f},{self.y:.2f},{out},{int(self.auto)},{int(self.hab)}")
            sig += self.Ts / self.speed                     # tiempo real sin acumular desfase
            time.sleep(max(0.0, sig - time.perf_counter()))

    def write(self, cmd):
        c, arg = cmd[0], cmd[1:]
        if c == "L":
            self.logging, self.k0 = True, self.k
            q.put(HEADER)
        elif c == "R":
            self.ref = float(arg)
        elif c == "K":
            self.pid.kp, self.pid.ki, self.pid.kd = (float(x) for x in arg.split(","))
            self.pid.bumpless(self.u, self.ref, self.y)
        elif c == "A":
            if not self.auto:
                self.auto = True
                self.pid.bumpless(self.u, self.ref, self.y)
        elif c == "M":
            self.auto, self.u_man = False, float(arg)
        elif c == "E":
            self.hab = int(arg) != 0
        elif c == "D":
            self.dist_until = self.k * self.Ts + 60.0
        elif c == "X":
            self.logging, self.hab, self.auto, self.u_man = False, False, False, 0.0
        q.put(f"# (sim) {cmd}")

    def close(self):
        pass


class ReplayLink:
    """Reproduce en tiempo real una corrida guardada (CSV t,PWM,Temp), solo el tramo en
       que el calentador ya se apago y la temperatura volvio a estar estable (a +-2 cuentas
       del promedio del ultimo minuto). Las muestras se reproducen seguidas cada 0.5 s
       (si el CSV tiene huecos, se saltan) y al terminar se recorren de ida y vuelta para
       seguir sin brincos. El calentador siempre se reporta APAGADO: las ordenes se
       ignoran, salvo la referencia, que solo se muestra."""
    TOL = 2.0

    def __init__(self, path, Ts=0.5):
        self.Ts, self.ref = Ts, REF0
        with open(path) as f:
            filas = [r.strip().split(",") for r in f if r[:1].isdigit()]
        t = [float(r[0]) for r in filas]; u = [float(r[1]) for r in filas]
        y = [float(r[2]) for r in filas]
        k_off = max((i for i, v in enumerate(u) if v > 0), default=-1) + 1
        fin = [v for ti, v in zip(t, y) if ti > t[-1] - 60]
        fin = sum(fin) / len(fin)
        k0 = next(i for i in range(k_off, len(y)) if abs(y[i] - fin) < self.TOL)
        self.datos = y[k0:]
        print(f"# Reproduciendo {os.path.basename(path)} desde t={t[k0]:.1f} s "
              f"({len(self.datos) * Ts / 60:.1f} min, calentador apagado)")
        self.logging = threading.Event()
        q.put("# Listo (reproduccion)")
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        self.logging.wait()
        sig = time.perf_counter()
        ida_vuelta = self.datos + self.datos[-2:0:-1]
        k = 0
        while not stop.is_set():
            adc = ida_vuelta[k % len(ida_vuelta)]
            k += 1
            q.put(f"{k * self.Ts:.1f},{self.ref:.0f},{adc:.2f},0,1,0")
            sig += self.Ts
            time.sleep(max(0.0, sig - time.perf_counter()))

    def write(self, cmd):
        if cmd == "L":
            q.put(HEADER)
            self.logging.set()
        elif cmd[0] == "R":
            self.ref = float(cmd[1:])

    def close(self):
        pass


class NullLink:
    """Sin Arduino ni datos: la interfaz se abre vacia y las ordenes se ignoran."""
    def write(self, cmd):
        pass

    def close(self):
        pass


# ---------------- Widgets ----------------
def round_rect(cv, x0, y0, x1, y1, r, **kw):
    pts = [x0 + r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y1 - r, x1, y1,
           x1 - r, y1, x0 + r, y1, x0, y1, x0, y1 - r, x0, y0 + r, x0, y0]
    return cv.create_polygon(pts, smooth=True, **kw)


class Card(tk.Canvas):
    """Recuadro redondeado con un titulo y un valor grande."""
    def __init__(self, master, title, w=210, h=62):
        super().__init__(master, width=w, height=h, bg=BG, highlightthickness=0)
        round_rect(self, 3, 3, w - 3, h - 3, 16, fill=BG, outline=BLUE, width=2)
        self.create_text(w / 2, 16, text=title, fill=BLUE, font=("Segoe UI", 11))
        self.val = self.create_text(w / 2, 40, text="--", fill=INK, font=("Segoe UI", 16, "bold"))

    def set(self, text, color=INK):
        self.itemconfigure(self.val, text=text, fill=color)


class Knob(ttk.Frame):
    """ImageKnob de TkDial (github.com/Akascape/TkDial) con knob.png y una escala
       con los valores crudos del ADC alrededor (se dibuja segun vmin..vmax).
       Rueda del mouse: pasos de `step`. Ademas se puede arrastrar (ImageKnob solo
       trae la rueda). Recorre 270°: minimo abajo-izquierda, maximo abajo-derecha.
       Llama a command(valor) cuando dejas de moverla.
       knob_offset: hacia donde apunta el indicador en knob.png (90 = arriba, 180 = izq.)"""
    SWEEP = 270.0

    def __init__(self, master, vmin, vmax, value, command, step=5, size=230,
                 ring=80, knob_offset=90):
        super().__init__(master)
        self.vmin, self.vmax, self.step, self.command = vmin, vmax, step, command
        self.size = size
        self._job, self._listo = None, False
        aqui = os.path.dirname(os.path.abspath(__file__))
        self.knob = ImageKnob(self, image=os.path.join(aqui, "knob.png"),
                              scale_image=self._escala(size, ring), scale_width=ring,
                              start=vmin, end=vmax, radius=size, text="",
                              start_angle=225 - knob_offset, end_angle=-self.SWEEP,
                              scroll_steps=step, integer=True, bg=BG, command=self._cambio)
        self.knob.pack()
        self.knob.bind("<Button-1>", self._drag)
        self.knob.bind("<B1-Motion>", self._drag)
        self.set(value)
        self._listo = True

    def _escala(self, size, ring, k=3):
        """PNG (en memoria) con marcas cada 10 cuentas y numeros cada 50."""
        from io import BytesIO
        from PIL import Image, ImageDraw, ImageFont
        N, c = size * k, size * k / 2
        r_knob = (size - ring) * k / 2
        im = Image.new("RGBA", (N, N), (0, 0, 0, 0))
        d = ImageDraw.Draw(im)
        try:
            font = ImageFont.truetype("segoeuib.ttf", 10 * k)
        except OSError:
            font = ImageFont.load_default()
        for v in range(self.vmin, self.vmax + 1, 10):
            a = math.radians(225 - self.SWEEP * (v - self.vmin) / (self.vmax - self.vmin))
            mayor = v % 50 == 0
            r1, r2 = r_knob + 3 * k, r_knob + (12 if mayor else 8) * k
            d.line((c + r1 * math.cos(a), c - r1 * math.sin(a),
                    c + r2 * math.cos(a), c - r2 * math.sin(a)),
                   fill=INK if mayor else "#adb5bd", width=(2 if mayor else 1) * k)
            if mayor:
                rt = r_knob + 26 * k
                d.text((c + rt * math.cos(a), c - rt * math.sin(a)), str(v), font=font,
                       fill=INK, anchor="mm")
        buf = BytesIO()
        im.resize((size, size), Image.LANCZOS).save(buf, "PNG")
        buf.seek(0)
        return buf

    @property
    def value(self):
        return int(self.knob.get())

    def set(self, v):
        v = round(min(max(v, self.vmin), self.vmax) / self.step) * self.step
        if v != self.knob.get() or not self._listo:
            self.knob.set(v)

    def _drag(self, e):
        c = self.size / 2
        ang = math.degrees(math.atan2(c - e.y, e.x - c))
        d = (225.0 - ang) % 360.0       # 0 en el minimo, crece en sentido horario
        if d > self.SWEEP:              # zona muerta de abajo: pegar al extremo mas cercano
            d = 0.0 if d > (360 + self.SWEEP) / 2 else self.SWEEP
        self.set(self.vmin + d / self.SWEEP * (self.vmax - self.vmin))

    def _cambio(self):
        if not self._listo:
            return
        if self._job:                   # envia cuando dejas de moverla
            self.after_cancel(self._job)
        self._job = self.after(400, lambda: self.command(self.value))


# ---------------- Ventana principal ----------------
class App:
    def __init__(self, root, link):
        self.root, self.link = root, link
        self.t, self.T, self.U, self.R = [], [], [], []
        self.dirty, self.last_draw, self.closing = False, 0.0, False
        self.hab = False

        root.title(("[SIMULADO] " if FAKE else "[REPRODUCCION] " if REPLAY else "") + "Control PID de temperatura  -  " + os.path.basename(fname))
        root.configure(bg=BG)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        style = ttk.Style()
        style.configure("TFrame", background=BG)
        style.configure("TLabel", background=BG)
        style.configure("TRadiobutton", background=BG)

        # --- graficas: temperatura arriba (2/3), PWM abajo (1/3), mismo eje de tiempo ---
        left = ttk.Frame(root)
        left.grid(row=0, column=0, sticky="nsew", padx=(12, 6), pady=12)
        self.fig = Figure(figsize=(8.5, 5.0), dpi=100)
        self.fig.subplots_adjust(left=0.08, right=0.98, top=0.97, bottom=0.1, hspace=0.12)
        ax, axU = self.fig.subplots(2, 1, sharex=True, gridspec_kw={"height_ratios": [2, 1]})
        self.ax, self.axU = ax, axU
        self.lT, = ax.plot([], [], color=RED, lw=2, label="Temperatura")
        self.lR, = ax.plot([], [], color=RED, lw=1.2, ls="--", alpha=0.55, label="Referencia")
        self.lU, = axU.plot([], [], color=GREEN, lw=2, label="PWM", drawstyle="steps-post")
        ax.legend(loc="upper left", frameon=False)
        for a in (ax, axU):
            a.yaxis.grid(True, ls="--", color="#555", lw=0.8)
        ax.set_ylabel("Temperatura [ADC]", color=RED)
        axU.set_ylabel("PWM [0-255]", color=GREEN)
        axU.set_xlabel("Tiempo [s]")
        ax.set_ylim(50, 500)
        axU.set_ylim(-10, 265)
        axU.set_xlim(0, 60)
        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # --- controles debajo de la grafica ---
        bar = ttk.Frame(left)
        bar.pack(fill="x", pady=(8, 0))
        self.modo = tk.StringVar(value="A")
        ttk.Radiobutton(bar, text="Lazo cerrado (PID)", variable=self.modo, value="A",
                        command=self.cambia_modo).pack(side="left")
        ttk.Radiobutton(bar, text="Lazo abierto  PWM:", variable=self.modo, value="M",
                        command=self.cambia_modo).pack(side="left", padx=(10, 0))
        self.pwm_man = tk.DoubleVar(value=0.0)
        self.lbl_man = ttk.Label(bar, text="0", width=4)
        sc = ttk.Scale(bar, from_=0, to=255, variable=self.pwm_man, length=120,
                       command=lambda v: self.lbl_man.configure(text=f"{float(v):.0f}"))
        sc.pack(side="left", padx=4)
        sc.bind("<ButtonRelease-1>", lambda e: self.modo.get() == "M" and self.cambia_modo())
        self.lbl_man.pack(side="left")

        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=10)
        self.k_entries = []
        for name, val in (("Kp", KP0), ("Ki", KI0), ("Kd", KD0)):
            ttk.Label(bar, text=name).pack(side="left")
            e = tk.Entry(bar, width=6, justify="center")
            e.insert(0, f"{val:g}")
            e.pack(side="left", padx=(2, 6))
            e.bind("<Return>", lambda ev: self.aplica_k())
            self.k_entries.append(e)
        ttk.Button(bar, text="Aplicar", command=self.aplica_k).pack(side="left")
        ttk.Button(bar, text="Marcar perturbación", command=self.perturbacion).pack(side="right")

        # --- columna derecha: habilitar, recuadros y perilla ---
        right = ttk.Frame(root)
        right.grid(row=0, column=1, sticky="n", padx=(6, 12), pady=12)
        # switch on/off (como el ejemplo de GeeksforGeeks: Button con imagen + Label al lado)
        aqui = os.path.dirname(os.path.abspath(__file__))
        self.img_on = tk.PhotoImage(file=os.path.join(aqui, "on.png"))
        self.img_off = tk.PhotoImage(file=os.path.join(aqui, "off.png"))
        sw = ttk.Frame(right)
        sw.pack(fill="x", pady=(4, 8))
        self.btn_hab = tk.Button(sw, bd=0, relief="flat", cursor="hand2", bg=BG,
                                 activebackground=BG, highlightthickness=0,
                                 command=self.toggle_hab)
        self.btn_hab.pack(side="left")
        self.lbl_hab = tk.Label(sw, bg=BG, font=("Segoe UI", 11, "bold"), justify="left")
        self.lbl_hab.pack(side="left", padx=(8, 0))
        self.pinta_hab()
        self.c_temp = Card(right, "Temp Actual")
        self.c_pwm  = Card(right, "PWM Actual")
        self.c_err  = Card(right, "± Error")
        self.c_ref  = Card(right, "Temp Deseada")
        for c in (self.c_temp, self.c_pwm, self.c_err, self.c_ref):
            c.pack(pady=3)
        self.knob = Knob(right, ADC_MIN, ADC_MAX, REF0, self.envia_ref)
        self.knob.pack(pady=(6, 0))

        # --- CSV ---
        self.f = open(fname, "w")
        self.f.write("t,PWM,Temp\n")             # t [s], PWM [0-255], Temp [cuentas ADC]

        root.protocol("WM_DELETE_WINDOW", self.cerrar)
        root.after(50, self.poll)

    # ----- ordenes al Arduino -----
    def send(self, cmd):
        try:
            self.link.write(cmd)
        except Exception as ex:
            print(f"# Error al enviar {cmd}: {ex}")
            return
        print(f"# Enviado: {cmd}")

    def envia_ref(self, v):
        self.send(f"R{v:.0f}")

    def toggle_hab(self):
        self.hab = not self.hab
        self.send(f"E{int(self.hab)}")
        self.pinta_hab()
        self.marcar("habilitado" if self.hab else "apagado")

    def pinta_hab(self):
        if self.hab:
            self.btn_hab.configure(image=self.img_on)
            self.lbl_hab.configure(text="Calentador\nencendido", fg=GREEN)
        else:
            self.btn_hab.configure(image=self.img_off)
            self.lbl_hab.configure(text="Calentador\napagado", fg=GRAY)

    def cambia_modo(self):
        if self.modo.get() == "A":
            self.send("A")
            self.marcar("PID")
        else:
            self.send(f"M{self.pwm_man.get():.0f}")
            self.marcar(f"Lazo abierto {self.pwm_man.get():.0f}")

    def aplica_k(self):
        try:
            k = [float(e.get()) for e in self.k_entries]
        except ValueError:
            for e in self.k_entries:
                e.configure(bg="#ffe3e3")
            return
        for e in self.k_entries:
            e.configure(bg="white")
        self.send("K" + ",".join(f"{x:g}" for x in k))
        self.marcar(f"Kp={k[0]:g} Ki={k[1]:g} Kd={k[2]:g}")

    def perturbacion(self):
        if FAKE:
            self.send("D")
        self.marcar("perturbacion")

    def marcar(self, texto):
        """Linea vertical en las dos graficas, con texto en la de arriba."""
        if self.t:
            x = self.t[-1]
            for a in (self.ax, self.axU):
                a.axvline(x, color=GRAY, ls=":", lw=1)
            self.ax.text(x, 0.98, " " + texto, transform=self.ax.get_xaxis_transform(),
                         rotation=90, va="top", ha="right", fontsize=8, color=GRAY)
            self.dirty = True

    # ----- datos -----
    def poll(self):
        while True:
            try:
                line = q.get_nowait()
            except queue.Empty:
                break
            if line.startswith("#"):
                print(line); continue
            if line == HEADER:
                continue
            try:
                t, r, T, u, m, h = (float(x) for x in line.split(","))
            except ValueError:
                continue                      # basura del puerto
            self.f.write(f"{t:.1f},{u:.0f},{T:.2f}\n")
            self.t.append(t); self.T.append(T); self.U.append(u); self.R.append(r)
            self.c_temp.set(f"{T:.0f}", RED)
            self.c_pwm.set(f"{u:.0f}", GREEN)
            e = r - T
            self.c_err.set(f"{e:+.0f}", GREEN if abs(e) < 3 else INK)
            self.c_ref.set(f"{r:.0f}", BLUE)
            if bool(h) != self.hab:            # el Arduino manda: si se reinicio, refleja su estado
                self.hab = bool(h)
                self.pinta_hab()
            self.dirty = True
        if self.dirty and time.time() - self.last_draw > 0.25:
            self.f.flush()
            self.redraw()
        if not self.closing:
            self.root.after(50, self.poll)

    def redraw(self):
        self.lT.set_data(self.t, self.T)
        self.lU.set_data(self.t, self.U)
        self.lR.set_data(self.t, self.R)
        if self.t:
            self.axU.set_xlim(0, max(60.0, self.t[-1] * 1.03))
            vals = self.T + self.R                # ajusta a los datos, minimo 100 cuentas de alto
            lo, hi = min(vals) - 20, max(vals) + 20
            if hi - lo < 100:
                lo, hi = (lo + hi) / 2 - 50, (lo + hi) / 2 + 50
            self.ax.set_ylim(lo, hi)
        self.canvas.draw_idle()
        self.dirty, self.last_draw = False, time.time()

    def cerrar(self):
        if self.closing:
            return
        self.closing = True
        stop.set()
        self.link.close()
        self.f.close()
        if VACIA:                          # sin datos: no deja archivos vacios
            os.remove(fname)
        else:
            self.redraw()
            self.fig.savefig(pngname, dpi=150)
            print(f"CSV: {fname}\nGrafica: {pngname}")
        self.root.destroy()


# ---------------- Arranque ----------------
link = (FakeLink(speed=SPEED) if FAKE else ReplayLink(REPLAY_CSV) if REPLAY else
        NullLink() if VACIA else SerialLink(PORT))
root = tk.Tk()
app = App(root, link)
# Configuracion inicial: calentador apagado, ganancias, referencia, modo PID y registrar
for cmd in ("E0", f"K{KP0:g},{KI0:g},{KD0:g}", f"R{REF0:.0f}", "A", "L"):
    app.send(cmd)
print(f"Guardando en {fname}  (cierra la ventana para terminar)")
try:
    root.mainloop()
except KeyboardInterrupt:
    app.cerrar()
