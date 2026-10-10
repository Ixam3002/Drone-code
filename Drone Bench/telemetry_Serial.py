"""
telemetry_serial.py
---------------------
Moniteur de télémétrie Tkinter connecté au port série de l'Arduino.
Permet d'afficher les vitesses, l'état des moteurs, les graphiques d'angles (Roll/Pitch)
et de modifier/envoyer les gains PID à l'Arduino.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
import queue
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

class SerialTelemetryWindow:
    def __init__(self, port="COM3", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.is_connected = False
        
        # Données de télémétrie initiales
        self.drone_demarre = False
        self.vitesse = (0.0, 0.0, 0.0)
        self.norme_vitesse = 0.0
        self.vitesse_angulaire_deg = (0.0, 0.0)
        
        self.f_A = 0.0
        self.f_B = 0.0
        self.f_C = 0.0
        self.f_D = 0.0
        self.esc_equilibre = 1435
        
        self.esc_A = 1000
        self.esc_B = 1000
        self.esc_C = 1000
        self.esc_D = 1000
        
        self.hist_roll = []
        self.hist_pitch = []
        
        # Gains PID initiaux (synchronisés avec votre code Arduino)
        self.pid_gains = {
            "roll": {"kp": 120.0, "ki": 0.8, "kd": 30.0},
            "pitch": {"kp": 30.0, "ki": 1.0, "kd": 12.0},
            "alt": {"kp": 49.89, "ki": 0.0082, "kd": 1.346}
        }

        # Fenêtre Tkinter
        self.root = tk.Tk()
        self.root.title("Moniteur Télémétrie Série Arduino & Contrôle PID")
        self.root.geometry("600x950")
        self.root.configure(bg="#12161c")

        self.setup_ui()
        self.start_serial_thread()

    def setup_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure(".", background="#12161c", foreground="#ffffff")
        
        # Style des barres de progression moteurs
        style.configure(
            "Green.Vertical.TProgressbar",
            troughcolor="#1a212b",
            background="#00d2ff",
            thickness=20
        )

        # 1. En-tête Statut & Connexion Port Série
        top_frame = tk.Frame(self.root, bg="#12161c")
        top_frame.pack(fill="x", padx=10, pady=5)

        self.lbl_status = tk.Label(
            top_frame, text="MOTORS: OFFLINE", font=("Consolas", 14, "bold"),
            bg="#12161c", fg="#ff4d4d"
        )
        self.lbl_status.pack(side="left", padx=5)

        self.btn_connect = tk.Button(
            top_frame, text="Connexion", font=("Consolas", 9, "bold"),
            bg="#00ff66", fg="#12161c", command=self.toggle_connection
        )
        self.btn_connect.pack(side="right", padx=5)

        # 2. Vitesses Linéaires & Angulaires
        frame_speeds = tk.LabelFrame(self.root, text=" Vitesses ", fg="#00d2ff", bg="#12161c", font=("Consolas", 10, "bold"))
        frame_speeds.pack(fill="x", padx=10, pady=5)

        self.lbl_vitesse = tk.Label(frame_speeds, text="", font=("Consolas", 10), bg="#12161c", fg="#ffffff", justify="left")
        self.lbl_vitesse.pack(anchor="w", padx=5, pady=2)

        self.lbl_angulaire = tk.Label(frame_speeds, text="", font=("Consolas", 10), bg="#12161c", fg="#ffffff", justify="left")
        self.lbl_angulaire.pack(anchor="w", padx=5, pady=2)

        # 3. Moniteur Moteurs & ESC avec Barres Dynamiques
        frame_motors = tk.LabelFrame(self.root, text=" Commandes Moteurs ", fg="#00d2ff", bg="#12161c", font=("Consolas", 10, "bold"))
        frame_motors.pack(fill="x", padx=10, pady=5)

        self.lbl_motors_txt = tk.Label(frame_motors, text="", font=("Consolas", 9), bg="#12161c", fg="#e0e0e0", justify="left")
        self.lbl_motors_txt.pack(anchor="w", padx=5, pady=2)

        bars_container = tk.Frame(frame_motors, bg="#12161c")
        bars_container.pack(fill="x", pady=5)

        self.motor_bars = {}
        self.motor_labels = {}

        for i, m_name in enumerate(["A", "B", "C", "D"]):
            sub_frame = tk.Frame(bars_container, bg="#12161c")
            sub_frame.pack(side="left", expand=True)

            lbl_title = tk.Label(sub_frame, text=f"Moteur {m_name}", font=("Consolas", 9, "bold"), bg="#12161c", fg="#ffcc00")
            lbl_title.pack()

            bar = ttk.Progressbar(
                sub_frame, orient="vertical", length=100, mode="determinate",
                maximum=2000, style="Green.Vertical.TProgressbar"
            )
            bar.pack(pady=2)

            lbl_val = tk.Label(sub_frame, text="1000 ESC", font=("Consolas", 8), bg="#12161c", fg="#ffffff")
            lbl_val.pack()

            self.motor_bars[m_name] = bar
            self.motor_labels[m_name] = lbl_val

        # 4. Graphique Matplotlib (Roll / Pitch)
        frame_graph = tk.LabelFrame(self.root, text=" Historique Angles (°) ", fg="#00d2ff", bg="#12161c", font=("Consolas", 10, "bold"))
        frame_graph.pack(fill="x", padx=10, pady=5)

        self.fig = Figure(figsize=(5, 2.0), dpi=100, facecolor="#12161c")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#1a212b")
        self.ax.tick_params(colors='white', labelsize=8)
        self.ax.spines['bottom'].set_color('white')
        self.ax.spines['left'].set_color('white')

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame_graph)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # 5. Contrôle des Gains PID via Champs de Saisie
        frame_pid = tk.LabelFrame(self.root, text=" Gains PID ", fg="#00d2ff", bg="#12161c", font=("Consolas", 10, "bold"))
        frame_pid.pack(fill="x", padx=10, pady=5)

        self.pid_entries = {}
        
        tk.Label(frame_pid, text="", bg="#12161c").grid(row=0, column=0)
        for col_idx, p_name in enumerate(["KP", "KI", "KD"]):
            tk.Label(frame_pid, text=p_name, font=("Consolas", 9, "bold"), bg="#12161c", fg="#00d2ff").grid(row=0, column=col_idx*2+1, columnspan=2, padx=10)

        for row, (axis_name, axis_key) in enumerate([("Roll", "roll"), ("Pitch", "pitch"), ("Alt", "alt")]):
            tk.Label(frame_pid, text=f"{axis_name}:", font=("Consolas", 10, "bold"), bg="#12161c", fg="#ffcc00").grid(row=row+1, column=0, padx=5, pady=4)
            
            for col, param in enumerate(["kp", "ki", "kd"]):
                val_init = str(self.pid_gains[axis_key][param])
                entry = tk.Entry(frame_pid, width=7, font=("Consolas", 9), bg="#1a212b", fg="#ffffff", insertbackground="white", justify="center")
                entry.insert(0, val_init)
                entry.grid(row=row+1, column=col*2+1, padx=4, pady=4)
                
                self.pid_entries[(axis_key, param)] = entry

        btn_apply = tk.Button(
            frame_pid, text="Appliquer les PID", font=("Consolas", 9, "bold"),
            bg="#00ff66", fg="#12161c", activebackground="#00cc52", command=self.apply_pid_values
        )
        btn_apply.grid(row=4, column=0, columnspan=7, pady=8, padx=10, sticky="ew")

        # Boucle de rafraîchissement UI
        self.root.after(33, self.update_loop)

    def toggle_connection(self):
        """Ouvre ou ferme la liaison série avec l'Arduino."""
        if not self.is_connected:
            try:
                self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
                self.is_connected = True
                self.btn_connect.config(text="Déconnexion", bg="#ff4d4d", fg="#ffffff")
            except Exception as e:
                messagebox.showerror("Erreur Port Série", f"Impossible de s'ouvrir sur {self.port}:\n{e}")
        else:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.is_connected = False
            self.btn_connect.config(text="Connexion", bg="#00ff66", fg="#12161c")

    def start_serial_thread(self):
        """Lance un thread d'écoute en arrière-plan pour ne pas bloquer l'interface Tkinter."""
        def read_serial():
            while True:
                if self.is_connected and self.serial_conn and self.serial_conn.is_open:
                    try:
                        line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            self.parse_telemetry_line(line)
                    except Exception:
                        pass
                else:
                    threading.Event().wait(0.1)

        t = threading.Thread(target=read_serial, daemon=True)
        t.start()

    def parse_telemetry_line(self, line):
        """
        Décode les lignes reçues de l'Arduino.
        Format standard conseillé envoyé par l'Arduino via Serial.print :
        DATA,armed,vx,vy,vz,wx,wy,fA,fB,fC,fD,escA,escB,escC,escD,roll,pitch
        """
        try:
            parts = line.split(',')
            if parts[0] == "DATA" and len(parts) >= 17:
                self.drone_demarre = bool(int(parts[1]))
                self.vitesse = (float(parts[2]), float(parts[3]), float(parts[4]))
                self.norme_vitesse = (float(parts[2])**2 + float(parts[3])**2 + float(parts[4])**2)**0.5
                self.vitesse_angulaire_deg = (float(parts[5]), float(parts[6]))
                
                self.f_A, self.f_B, self.f_C, self.f_D = float(parts[7]), float(parts[8]), float(parts[9]), float(parts[10])
                self.esc_A, self.esc_B, self.esc_C, self.esc_D = float(parts[11]), float(parts[12]), float(parts[13]), float(parts[14])
                
                roll = float(parts[15])
                pitch = float(parts[16])
                
                self.hist_roll.append(roll)
                self.hist_pitch.append(pitch)
                if len(self.hist_roll) > 100:
                    self.hist_roll.pop(0)
                    self.hist_pitch.pop(0)
            elif len(parts) == 2:
                # Rétrocompatibilité avec votre code actuel Arduino (Serial.print(roll); Serial.println(pitch));
                try:
                    roll = float(parts[0])
                    pitch = float(parts[1])
                    self.hist_roll.append(roll)
                    self.hist_pitch.append(pitch)
                    if len(self.hist_roll) > 100:
                        self.hist_roll.pop(0)
                        self.hist_pitch.pop(0)
                except ValueError:
                    pass
        except Exception:
            pass

    def apply_pid_values(self):
        """Met à jour les gains et les transmet à l'Arduino en écrivant sur le port série."""
        for (axis, param), entry in self.pid_entries.items():
            try:
                val = float(entry.get().replace(",", "."))
                self.pid_gains[axis][param] = max(0.0, val)
            except ValueError:
                pass
        
        if self.is_connected and self.serial_conn and self.serial_conn.is_open:
            try:
                for axis in ["roll", "pitch", "alt"]:
                    kp = self.pid_gains[axis]["kp"]
                    ki = self.pid_gains[axis]["ki"]
                    kd = self.pid_gains[axis]["kd"]
                    cmd_str = f"PID:{axis}:{kp},{ki},{kd}\n"
                    self.serial_conn.write(cmd_str.encode('utf-8'))
                messagebox.showinfo("Succès", "Gains PID envoyés à l'Arduino avec succès !")
            except Exception as e:
                messagebox.showerror("Erreur", f"Erreur lors de l'envoi:\n{e}")
        else:
            messagebox.showwarning("Avertissement", "Port série non connecté !")

    def update_loop(self):
        # 1. Statut
        if self.is_connected:
            if self.drone_demarre:
                self.lbl_status.config(text="MOTORS: ONLINE", fg="#00ff66")
            else:
                self.lbl_status.config(text="MOTORS: ARMED / PAUSED", fg="#ff4d4d")
        else:
            self.lbl_status.config(text="MOTORS: OFFLINE", fg="#ff4d4d")

        # 2. Vitesses
        vx, vy, vz = self.vitesse
        self.lbl_vitesse.config(
            text=f"Vx: {vx:+.2f} m/s | Vy: {vy:+.2f} m/s | Vz: {vz:+.2f} m/s | ||V||: {self.norme_vitesse:.2f} m/s"
        )

        wx, wy = self.vitesse_angulaire_deg
        al_x = " ⚠️ [CRITIQUE]" if abs(wx) > 50.0 else ""
        al_y = " ⚠️ [CRITIQUE]" if abs(wy) > 50.0 else ""
        
        self.lbl_angulaire.config(
            text=f"Wx (Roll): {wx:+.1f}°/s{al_x}\nWy (Pitch): {wy:+.1f}°/s{al_y}",
            fg="#ff4d4d" if (al_x or al_y) else "#ffffff"
        )

        # 3. Moteurs
        self.lbl_motors_txt.config(
            text=(
                f"Forces (N) : A={self.f_A:.2f} | B={self.f_B:.2f} | C={self.f_C:.2f} | D={self.f_D:.2f}\n"
                f"Équilibre  : ~{int(self.esc_equilibre)} ESC"
            )
        )

        esc_vals = {
            "A": self.esc_A,
            "B": self.esc_B,
            "C": self.esc_C,
            "D": self.esc_D
        }

        for m_name, val in esc_vals.items():
            v_int = int(val)
            self.motor_bars[m_name]["value"] = v_int
            self.motor_labels[m_name].config(text=f"{v_int} ESC")

        # 4. Graphique
        self.ax.clear()
        self.ax.set_facecolor("#1a212b")
        self.ax.set_ylim(-60, 60)
        if len(self.hist_roll) > 1:
            self.ax.plot(self.hist_roll, label="Roll (°)", color="#00ff66", linewidth=1.5)
            self.ax.plot(self.hist_pitch, label="Pitch (°)", color="#00d2ff", linewidth=1.5)
            self.ax.legend(loc="upper right", facecolor="#12161c", edgecolor="none", labelcolor="white", fontsize=8)
        self.canvas.draw_idle()

        self.root.after(33, self.update_loop)

    def start(self):
        self.root.mainloop()

if __name__ == "__main__":
    # Pensez à remplacer "COM3" par votre port série Arduino réel (ex: "COM4" sur Windows ou "/dev/ttyUSB0" sur Linux)
    app = SerialTelemetryWindow(port="COM3", baudrate=115200)
    app.start()


# Serial.print("DATA,");
# Serial.print(ARMED ? 1 : 0); Serial.print(",");
# Serial.print(vitesse_x); Serial.print(",");
# // ... (vx, vy, vz, wx, wy, forces A-D, valeurs ESC A-D)
# Serial.print(mpu.roll); Serial.print(",");
# Serial.println(mpu.pitch);