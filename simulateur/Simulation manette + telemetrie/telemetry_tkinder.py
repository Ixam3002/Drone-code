"""
drone_telemetry_window.py
-------------------------
Fenêtre Tkinter de télémétrie avec barres de niveau pour les moteurs
et champs de saisie directs pour les gains PID.
"""

import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class TelemetryWindow:
    def __init__(self, simu):
        self.simu = simu
        self.root = tk.Tk()
        self.root.title("Moniteur de Télémétrie & Contrôle PID")
        self.root.geometry("600x900")
        self.root.configure(bg="#12161c")

        self.setup_ui()

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

        # 1. En-tête Statut
        self.lbl_status = tk.Label(
            self.root, text="MOTORS: OFFLINE", font=("Consolas", 14, "bold"),
            bg="#12161c", fg="#ff4d4d"
        )
        self.lbl_status.pack(pady=10)

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

        # Frame contenant les 4 barres verticales
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

            lbl_val = tk.Label(sub_frame, text="0 ESC", font=("Consolas", 8), bg="#12161c", fg="#ffffff")
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
        
        # En-tête des colonnes KP, KI, KD
        tk.Label(frame_pid, text="", bg="#12161c").grid(row=0, column=0)
        for col_idx, p_name in enumerate(["KP", "KI", "KD"]):
            tk.Label(frame_pid, text=p_name, font=("Consolas", 9, "bold"), bg="#12161c", fg="#00d2ff").grid(row=0, column=col_idx*2+1, columnspan=2, padx=10)

        for row, (axis_name, axis_key) in enumerate([("Roll", "roll"), ("Pitch", "pitch"), ("Alt", "alt")]):
            tk.Label(frame_pid, text=f"{axis_name}:", font=("Consolas", 10, "bold"), bg="#12161c", fg="#ffcc00").grid(row=row+1, column=0, padx=5, pady=4)
            
            for col, param in enumerate(["kp", "ki", "kd"]):
                val_init = str(self.simu.pid_gains[axis_key][param])
                entry = tk.Entry(frame_pid, width=7, font=("Consolas", 9), bg="#1a212b", fg="#ffffff", insertbackground="white", justify="center")
                entry.insert(0, val_init)
                entry.grid(row=row+1, column=col*2+1, padx=4, pady=4)
                
                self.pid_entries[(axis_key, param)] = entry

        # Bouton unique pour valider toutes les nouvelles valeurs entrées
        btn_apply = tk.Button(
            frame_pid, text="Appliquer les PID", font=("Consolas", 9, "bold"),
            bg="#00ff66", fg="#12161c", activebackground="#00cc52", command=self.apply_pid_values
        )
        btn_apply.grid(row=4, column=0, columnspan=7, pady=8, padx=10, sticky="ew")

        # Rafraîchissement périodique (30 fps)
        self.root.after(33, self.update_loop)

    def apply_pid_values(self):
        """Lit les champs texte et applique les gains au simulateur."""
        for (axis, param), entry in self.pid_entries.items():
            try:
                val = float(entry.get().replace(",", "."))
                self.simu.pid_gains[axis][param] = max(0.0, val)
            except ValueError:
                pass  # Ignore si la valeur n'est pas un nombre valide

    def update_loop(self):
        # 1. Statut
        if self.simu.drone_demarre:
            self.lbl_status.config(text="MOTORS: ONLINE", fg="#00ff66")
        else:
            self.lbl_status.config(text="MOTORS: ARMED / PAUSED", fg="#ff4d4d")

        # 2. Vitesses
        vx, vy, vz = self.simu.vitesse
        self.lbl_vitesse.config(
            text=f"Vx: {vx:+.2f} m/s | Vy: {vy:+.2f} m/s | Vz: {vz:+.2f} m/s | ||V||: {self.simu.norme_vitesse:.2f} m/s"
        )

        wx, wy = self.simu.vitesse_angulaire_deg[0], self.simu.vitesse_angulaire_deg[1]
        al_x = " ⚠️ [CRITIQUE]" if abs(wx) > self.simu.SEUIL_VIT_ANGULAIRE_MAX else ""
        al_y = " ⚠️ [CRITIQUE]" if abs(wy) > self.simu.SEUIL_VIT_ANGULAIRE_MAX else ""
        
        self.lbl_angulaire.config(
            text=f"Wx (Roll): {wx:+.1f}°/s{al_x}\nWy (Pitch): {wy:+.1f}°/s{al_y}",
            fg="#ff4d4d" if (al_x or al_y) else "#ffffff"
        )

        # 3. Moniteur Moteurs & Jauges
        self.lbl_motors_txt.config(
            text=(
                f"Forces (N) : A={self.simu.f_A:.2f} | B={self.simu.f_B:.2f} | C={self.simu.f_C:.2f} | D={self.simu.f_D:.2f}\n"
                f"Équilibre  : ~{int(self.simu.esc_equilibre)} ESC"
            )
        )

        esc_vals = {
            "A": self.simu.esc_A,
            "B": self.simu.esc_B,
            "C": self.simu.esc_C,
            "D": self.simu.esc_D
        }

        for m_name, val in esc_vals.items():
            v_int = int(val)
            self.motor_bars[m_name]["value"] = v_int
            self.motor_labels[m_name].config(text=f"{v_int} ESC")

        # 4. Graphiques Angles
        self.ax.clear()
        self.ax.set_facecolor("#1a212b")
        self.ax.set_ylim(-60, 60)
        if len(self.simu.hist_roll) > 1:
            self.ax.plot(self.simu.hist_roll, label="Roll (°)", color="#00ff66", linewidth=1.5)
            self.ax.plot(self.simu.hist_pitch, label="Pitch (°)", color="#00d2ff", linewidth=1.5)
            self.ax.legend(loc="upper right", facecolor="#12161c", edgecolor="none", labelcolor="white", fontsize=8)
        self.canvas.draw_idle()

        self.root.after(33, self.update_loop)

    def start(self):
        self.root.mainloop()