import sys
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, CheckButtons, Slider
import numpy as np
import pygame

import asservissement as asserv
import dynamique as dyn
import fonctions as fct
import manette_xbox_commande as mxc

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Aucune manette détectée ! Vérifie la connexion.")
    sys.exit()

manette = pygame.joystick.Joystick(0)
manette.init()

###############################
# Données terre & drone
###############################
g = 9.81
Masse = 0.3  # kg
Poids_global = np.array([0.0, 0.0, -Masse * g])

Longueur = 0.12  # m (Axe X local)
Largeur = 0.08   # m (Axe Y local)
Hauteur = 0.05   # m (Axe Z local)

L = Longueur / 2.0  # demi-longueur
l = Largeur / 2.0   # demi-largeur

Mat_Inertie = fct.matrice_inertie_parallelepipede(
    Longueur, Largeur, Hauteur, Masse
)

###############################
# Variables globales & état
###############################
pos = np.array([0.0, 0.0, 0.0])
pos_actuel = np.copy(pos)

angle = np.array([0.0, 0.0, 0.0])
angle_actuel = np.copy(angle)
angle_ancien = np.copy(angle)

delta_t = 0.05
t = 0.0

poussee_equilibre = (Masse * g) / 4.0
drone_demarre = False

trajectoire_x, trajectoire_y, trajectoire_z = [], [], []
historique_t = []
historique_pitch = []
historique_roll = []

###############################
# Configuration Affichage
###############################
plt.ion()

fig_3d = plt.figure(figsize=(9, 7))
ax_3d = fig_3d.add_subplot(projection="3d")

fig_graph = plt.figure(figsize=(9, 8))

ax_angles = fig_graph.add_axes([0.10, 0.50, 0.80, 0.44])
ax_angles.set_title("Évolution des Angles (Tangage & Roulis) en temps réel")
ax_angles.set_xlabel("Temps (s)")
ax_angles.set_ylabel("Angle (°)")
ax_angles.grid(True)

# Positions des Sliders Kp
ax_kp_pitch = fig_graph.add_axes([0.25, 0.38, 0.60, 0.025])
ax_kp_roll  = fig_graph.add_axes([0.25, 0.35, 0.60, 0.025])

# Positions des Sliders Ki
ax_ki_pitch = fig_graph.add_axes([0.25, 0.31, 0.60, 0.025]) 
ax_ki_roll  = fig_graph.add_axes([0.25, 0.28, 0.60, 0.025])

#  fig_graph.add_axes([0.25, 0.18, 0.60, 0.025])

#Position des Sliders Kd
ax_kd_pitch   = fig_graph.add_axes([0.25, 0.24, 0.60, 0.025])
ax_kd_roll   = fig_graph.add_axes([0.25, 0.21, 0.60, 0.025])

# Checkbox et Bouton Reset
ax_check = fig_graph.add_axes([0.15, 0.02, 0.30, 0.09])
ax_btn   = fig_graph.add_axes([0.55, 0.03, 0.30, 0.06])

# Sliders Kp
slider_kp_pitch = Slider(ax_kp_pitch, "Kp Pitch", 0.0, 0.5, valinit=0.02, valfmt="%.3f", color="skyblue")
slider_kp_roll  = Slider(ax_kp_roll,  "Kp Roll",  0.0, 0.5, valinit=0.02, valfmt="%.3f", color="lightgreen")

# Sliders Ki
slider_ki_pitch = Slider(ax_ki_pitch, "Ki Pitch", 0.0, 0.1, valinit=0.01, valfmt="%.3f", color="deepskyblue")
slider_ki_roll  = Slider(ax_ki_roll,  "Ki Roll",  0.0, 0.1, valinit=0.01, valfmt="%.3f", color="mediumseagreen")

#Sliders Kd
slider_kd_pitch   = Slider(ax_kd_pitch,   "Kd Pitch",   0.0, 0.1, valinit=0.005, valfmt="%.3f", color="coral")
slider_kd_roll   = Slider(ax_kd_roll,   "Kd Roll",   0.0, 0.1, valinit=0.005, valfmt="%.3f", color="indianred")

axes_labels = ["Roulis (Roll)", "Tangage (Pitch)", "Lacet (Yaw)"]
axes_status = [True, True, True]
check_axes = CheckButtons(ax_check, axes_labels, axes_status)

btn_reset = Button(ax_btn, "Redémarrer", color="lightgray", hovercolor="tomato")


def reinitialiser_simulation(event):
    global pos, pos_actuel, angle, angle_actuel, angle_ancien, t, drone_demarre
    global trajectoire_x, trajectoire_y, trajectoire_z, historique_t, historique_pitch, historique_roll

    pos = np.array([0.0, 0.0, 0.0])
    pos_actuel = np.copy(pos)
    angle = np.array([0.0, 0.0, 0.0])
    angle_actuel = np.copy(angle)
    angle_ancien = np.copy(angle)

    t = 0.0
    drone_demarre = False

    trajectoire_x.clear()
    trajectoire_y.clear()
    trajectoire_z.clear()
    historique_t.clear()
    historique_pitch.clear()
    historique_roll.clear()


btn_reset.on_clicked(reinitialiser_simulation)


try:
    while plt.fignum_exists(fig_3d.number) and plt.fignum_exists(fig_graph.number):
        pygame.event.pump()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        bouton_A = manette.get_button(0)
        if bouton_A:
            drone_demarre = True

        (
            stick_gauche_x,
            stick_gauche_y,
            stick_droite_x,
            stick_droite_y,
        ) = mxc.position_joystick_manette(manette)

        stick_droite_y = -stick_droite_y
        etats_axes = check_axes.get_status()  # [Roll, Pitch, Yaw]


        # if not(etats_axes[0]):
        #     angle = np.array([angle[0], 0, angle[2]])

        # if not(etats_axes[1]):
        #     angle = np.array([0, angle[1], angle[2]])




        # -------------------------------------------------------------
        # 1. CALCUL DES FORCES BRUTES DE LA MANETTE
        # -------------------------------------------------------------
        if drone_demarre:
            cmd_poussee = stick_droite_y * 0.5  # Variation de gaz
            cmd_roll_pilote = (stick_gauche_x * 0.05)
            cmd_pitch_pilote = (-stick_gauche_y * 0.05)

            if not etats_axes[0]:
                cmd_roll_pilote = 0

            if not etats_axes[1]:
                cmd_pitch_pilote = 0


            # Forces de base voulues par le pilote
            #cmd_yaw_pilote = (stick_droite_x * 0.05) if etats_axes[2] else 0.0

            # Injection sur les moteurs pour créer un couple sans tangage/roulis :
            f_A_manette = poussee_equilibre + cmd_poussee - cmd_roll_pilote - cmd_pitch_pilote
            f_B_manette = poussee_equilibre + cmd_poussee + cmd_roll_pilote - cmd_pitch_pilote
            f_C_manette = poussee_equilibre + cmd_poussee - cmd_roll_pilote + cmd_pitch_pilote
            f_D_manette = poussee_equilibre + cmd_poussee + cmd_roll_pilote + cmd_pitch_pilote
        else:
            f_A_manette, f_B_manette, f_C_manette, f_D_manette = 0.0, 0.0, 0.0, 0.0

        # -------------------------------------------------------------
        # 2. CALCUL DE LA CORRECTION D'ASSERVISSEMENT (Stabilisation à 0°)
        # -------------------------------------------------------------
        kp_roll  = slider_kp_roll.val
        kp_pitch = slider_kp_pitch.val

        ki_roll  = slider_ki_roll.val
        ki_pitch = slider_ki_pitch.val

        kd_roll  = slider_kd_roll.val
        kd_pitch = slider_kd_pitch.val

        # f_corr_A, f_corr_B, f_corr_C, f_corr_D = asserv.calcul_correction_PI(
        #     angle_actuel,
        #     angle_cible=[0.0, 0.0, 0.0],  # Cherche à stabiliser à plat
        #     Kp_roll=kp_roll, Kp_pitch=kp_pitch, Kp_yaw=kp_yaw,
        #     Ki_roll=ki_roll, Ki_pitch=ki_pitch, Ki_yaw=ki_yaw,
        #     delta_t=delta_t,
        #     reset_integrale=not drone_demarre
        # )


        f_corr_A, f_corr_B, f_corr_C, f_corr_D = asserv.calcul_correction_PID(
        angle_actuel,
        angle_cible=np.array([0.0, 0.0, 0.0]),
        Kp_roll=kp_roll, Kp_pitch=kp_pitch,  Kp_yaw=0.1,
        Ki_roll=ki_roll,  Ki_pitch=ki_pitch,  Ki_yaw=0.005,
        Kd_roll=kd_roll,  Kd_pitch=kd_pitch,  Kd_yaw=0.02,
        delta_t=delta_t,
        reset_pid=False,
    )


        # -------------------------------------------------------------
        # 3. FORCE FINALE = MANETTE + ASSERVISSEMENT
        # -------------------------------------------------------------
        if drone_demarre:
            f_A = max(0.0, f_A_manette + f_corr_A)
            f_B = max(0.0, f_B_manette + f_corr_B)
            f_C = max(0.0, f_C_manette + f_corr_C)
            f_D = max(0.0, f_D_manette + f_corr_D)
        else:
            f_A, f_B, f_C, f_D = 0.0, 0.0, 0.0, 0.0

        # f_A = f_A_manette
        # f_B = f_B_manette
        # f_C = f_C_manette
        # f_D = f_D_manette


        poussee_totale = f_A + f_B + f_C + f_D

        R = fct.get_rotation_matrix(angle[0], angle[1], angle[2])

        Force_A_g = np.dot(R, np.array([0, 0, f_A]))
        Force_B_g = np.dot(R, np.array([0, 0, f_B]))
        Force_C_g = np.dot(R, np.array([0, 0, f_C]))
        Force_D_g = np.dot(R, np.array([0, 0, f_D]))

        Forces = np.array([Force_A_g, Force_B_g, Force_C_g, Force_D_g, Poids_global])
        Moments = fct.moments_moteur(
            np.array([0, 0, f_A]),
            np.array([0, 0, f_B]),
            np.array([0, 0, f_C]),
            np.array([0, 0, f_D]),
            L,
            l,
        )

        # Réinitialisation des axes désactivés
        # for idx in range(3):
        #     if not etats_axes[idx]:
        #         Moments[idx] = 0.0
        #         angle[idx] = 0.0
        #         angle_actuel[idx] = 0.0
        #         angle_ancien[idx] = 0.0

        pos_ancienne = np.copy(pos_actuel)
        pos_actuel = np.copy(pos)

        angle_ancien = np.copy(angle_actuel)
        angle_actuel = np.copy(angle)

        #CALCUL DE LA POSITION ET DE L'ANGLE

        pos = dyn.position(
            pos_actuel, pos_ancienne, Forces, Masse, Delta_t=delta_t
        )
        angle = dyn.angle(
            angle_actuel, angle_ancien, Moments, Mat_Inertie, Delta_t=delta_t
        )

        # Contact Sol
        if pos[2] <= 0:
            pos[2] = 0.0
            if poussee_totale < (Masse * g):
                pos[0] = pos_actuel[0]
                pos[1] = pos_actuel[1]
                angle = np.copy(angle_actuel)

        trajectoire_x.append(pos[0])
        trajectoire_y.append(pos[1])
        trajectoire_z.append(pos[2])

        roulis_deg = np.degrees(angle[0])
        tangage_deg = np.degrees(angle[1])
        
        historique_t.append(t)
        historique_roll.append(roulis_deg)
        historique_pitch.append(tangage_deg)

        # Tracé 3D
        ax_3d.cla()
        ax_3d.plot_surface(
            np.array([[-5, 5], [-5, 5]]),
            np.array([[-5, -5], [5, 5]]),
            np.array([[0, 0], [0, 0]]),
            color="gray",
            alpha=0.2,
        )
        ax_3d.plot(
            trajectoire_x, trajectoire_y, trajectoire_z, "r--", linewidth=1.5, alpha=0.6, label="Trajectoire"
        )
        fct.tracer_pave_drone(ax_3d, pos, R, Longueur, Largeur, Hauteur, drone_demarre)

        scale_axes = 0.2
        u_x, u_y, u_z = R[:, 0] * scale_axes, R[:, 1] * scale_axes, R[:, 2] * scale_axes
        ax_3d.quiver(pos[0], pos[1], pos[2], u_x[0], u_x[1], u_x[2], color="red", linewidth=2)
        ax_3d.quiver(pos[0], pos[1], pos[2], u_y[0], u_y[1], u_y[2], color="green", linewidth=2)
        ax_3d.quiver(pos[0], pos[1], pos[2], u_z[0], u_z[1], u_z[2], color="blue", linewidth=2)

        ax_3d.set_xlim([pos[0] - 2, pos[0] + 2])
        ax_3d.set_ylim([pos[1] - 2, pos[1] + 2])
        ax_3d.set_zlim([max(0, pos[2] - 1), pos[2] + 3])
        ax_3d.set_xlabel("X (m)")
        ax_3d.set_ylabel("Y (m)")
        ax_3d.set_zlabel("Z (m)")

        pourcentage_poussee = (poussee_totale / (Masse * g)) * 100 if drone_demarre else 0.0
        etat_moteur = "DÉMARRÉS" if drone_demarre else "COUPÉS (Appuie sur A)"
        ax_3d.set_title(
            f"Simulateur Drone 3D | Moteurs: {etat_moteur}\n"
            f"Poussée: {pourcentage_poussee:.1f}% ({poussee_totale:.2f} N)\n"
            f"t = {round(t, 2)} s | Roll: {roulis_deg:.1f}° | Pitch: {tangage_deg:.1f}°"
        )

        # Graphique des angles
        ax_angles.cla()
        ax_angles.plot(historique_t, historique_roll, "g-", linewidth=2, label="Roulis / Roll (°)")
        ax_angles.plot(historique_t, historique_pitch, "b-", linewidth=2, label="Tangage / Pitch (°)")
        ax_angles.set_title("Évolution des Angles (Roulis & Tangage) en temps réel")
        ax_angles.set_xlabel("Temps (s)")
        ax_angles.set_ylabel("Angle (°)")
        ax_angles.grid(True)
        ax_angles.legend(loc="upper right")

        if t > 10:
            ax_angles.set_xlim([t - 10, t])

        t += delta_t
        plt.draw()
        plt.pause(0.001)

except KeyboardInterrupt:
    print("\nArrêt du simulateur.")
finally:
    pygame.quit()
    plt.close("all")