import asservissement as asserv
import dynamique as dyn
import fonctions as fct

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import differential_evolution

###############################
# Données terre & drone
###############################
g = 9.81
Masse = 0.650  # kg

Longueur = 0.14  # m
Largeur = 0.10  # m
Hauteur = 0.05  # m

# Branche des moteurs
L = 0.14
l = 0.125

# Coefficient de frottement de l'air (kg/m)
k_frot = 0.05 

###############################
# Conditions initiales & Seuils
###############################
pos_in = np.array([0.0, 0.0, 5.0])
angle_in = np.array([0.0, 10.0 * np.pi / 180, 0.0])

delta_t = 0.01
reussite_angle = np.radians(0.001)
reussite_vitesse = np.radians(0.001)
reussite_vit_lin = 0.01

###############################
# Fonction de simulation
###############################
def simuler_drone(x):
    kp_pitch, ki_pitch, kd_pitch, Longueur, Largeur, Hauteur, L, l = x

    Mat_Inertie = fct.matrice_inertie_parallelepipede(
        Longueur, Largeur, Hauteur, Masse
    )

    Poids_global = np.array([0.0, 0.0, -Masse * g])

    t = 0.0
    N = 0
    N_max = 1500

    pos = np.copy(pos_in)
    angle = np.copy(angle_in)
    pos_actuel = np.copy(pos)
    angle_actuel = np.copy(angle)

    historique_t = []
    historique_pitch = []
    historique_z = []
    historique_vit_x = []

    erreur_angle_cumulee = 0.0
    erreur_alt_cumulee = 0.0
    erreur_vit_x_cumulee = 0.0  # <-- NOUVEAU : Cumul erreur sur Vx
    echec_simulation = False

    while N < N_max:
        # 1. Compensation dynamique de la poussée d'équilibre
        cos_roll = np.cos(angle_actuel[0])
        cos_pitch = np.cos(angle_actuel[1])
        denom = max(0.1, cos_roll * cos_pitch)
        poussee_eq_dyn = (Masse * g) / (4.0 * denom)

        # 2. Correction PID
        f_corr_A, f_corr_B, f_corr_C, f_corr_D = asserv.calcul_correction_PID(
            angle_actuel,
            angle_cible=np.array([0.0, 0.0, 0.0]),
            Kp_roll=0,
            Kp_pitch=kp_pitch,
            Kp_yaw=0.1,
            Ki_roll=0,
            Ki_pitch=ki_pitch,
            Ki_yaw=0.005,
            Kd_roll=0,
            Kd_pitch=kd_pitch,
            Kd_yaw=0.02,
            delta_t=delta_t,
            reset_pid=(N == 0),
        )

        # 3. Application de la poussée
        f_A = max(0.0, poussee_eq_dyn + f_corr_A)
        f_B = max(0.0, poussee_eq_dyn + f_corr_B)
        f_C = max(0.0, poussee_eq_dyn + f_corr_C)
        f_D = max(0.0, poussee_eq_dyn + f_corr_D)

        # 4. Calcul de la vitesse et de la force de frottement de l'air
        vit = (pos - pos_actuel) / delta_t if N > 0 else np.array([0.0, 0.0, 0.0])
        vitesse_lin_norme = np.linalg.norm(vit)
        
        # Force de traînée quadratique (F_frot = -k * ||v|| * v)
        F_frot = -k_frot * vitesse_lin_norme * vit

        # 5. Dynamique
        R = fct.get_rotation_matrix(angle[0], angle[1], angle[2])
        Force_A_g = np.dot(R, np.array([0, 0, f_A]))
        Force_B_g = np.dot(R, np.array([0, 0, f_B]))
        Force_C_g = np.dot(R, np.array([0, 0, f_C]))
        Force_D_g = np.dot(R, np.array([0, 0, f_D]))

        # Ingestion de la force de frottement dans les forces appliquées
        Forces = np.array(
            [Force_A_g, Force_B_g, Force_C_g, Force_D_g, Poids_global, F_frot]
        )
        Moments = fct.moments_moteur(
            np.array([0, 0, f_A]),
            np.array([0, 0, f_B]),
            np.array([0, 0, f_C]),
            np.array([0, 0, f_D]),
            L,
            l,
        )

        pos_ancienne = np.copy(pos_actuel)
        pos_actuel = np.copy(pos)
        angle_ancien = np.copy(angle_actuel)
        angle_actuel = np.copy(angle)

        pos = dyn.position(
            pos_actuel, pos_ancienne, Forces, Masse, Delta_t=delta_t
        )
        angle = dyn.angle(
            angle_actuel, angle_ancien, Moments, Mat_Inertie, Delta_t=delta_t
        )

        # Sécurités : instabilité ou crash sol
        if np.isnan(angle).any() or np.isnan(pos).any() or abs(angle[1]) > (np.pi / 2):
            echec_simulation = True
            break

        if pos[2] <= 0.0:
            echec_simulation = True
            break

        # 6. Historique & Cumul d'erreurs (ITAE)
        historique_t.append(t)
        historique_pitch.append(np.degrees(angle[1]))
        historique_z.append(pos[2])
        historique_vit_x.append(vit[0])

        erreur_angle_cumulee += t * abs(angle[1]) * delta_t
        erreur_alt_cumulee += t * abs(pos[2] - pos_in[2]) * delta_t
        erreur_vit_x_cumulee += t * abs(vit[0]) * delta_t  # <-- NOUVEAU : Pénalité temps x |Vx|

        # 7. Condition d'arrêt de stabilisation
        vitesse_pitch = (angle[1] - angle_actuel[1]) / delta_t
        est_stabilise = (
            abs(angle[1]) <= reussite_angle
            and abs(vitesse_pitch) <= reussite_vitesse
            and vitesse_lin_norme <= reussite_vit_lin
            and abs(pos[2] - 5.0) <= 0.1
        )

        if est_stabilise:
            break

        t += delta_t
        N += 1

    return (
        t,
        erreur_angle_cumulee,
        erreur_alt_cumulee,
        erreur_vit_x_cumulee,  # <-- Retourné
        echec_simulation,
        historique_t,
        historique_pitch,
        historique_z,
        historique_vit_x,
    )

x = [4.37274459, 0.23640114, 0.07577104,
  0.13348314, 0.10681354, 0.09351382,
 0.1388413, 0.09488815]

# Affichage du résultat optimal
t_opt, _, _, _, echec_opt, hist_t, hist_pitch, hist_z, hist_vit_x = simuler_drone(x)

if not echec_opt:
    print(f"Temps de stabilisation optimal = {t_opt:.2f} s")

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 9), sharex=True)

    # 1. Tangage
    ax1.plot(hist_t, hist_pitch, "b-", linewidth=1.5, label="Tangage (°)")
    ax1.axhline(y=np.degrees(reussite_angle), color="r", linestyle="--", alpha=0.7)
    ax1.axhline(y=-np.degrees(reussite_angle), color="r", linestyle="--", alpha=0.7)
    ax1.set_ylabel("Angle (°)")
    ax1.set_title("Tangage, Altitude Z et Vitesse $V_x$")
    ax1.grid(True)
    ax1.legend()

    # 2. Altitude
    ax2.plot(hist_t, hist_z, "g-", linewidth=1.5, label="Altitude Z (m)")
    ax2.axhline(y=5.0, color="r", linestyle="--", alpha=0.7, label="Consigne (5 m)")
    ax2.set_ylabel("Altitude (m)")
    ax2.grid(True)
    ax2.legend()

    # 3. Vitesse Vx
    ax3.plot(hist_t, hist_vit_x, "m-", linewidth=1.5, label="Vitesse $V_x$ (m/s)")
    ax3.axhline(y=0.0, color="r", linestyle="--", alpha=0.7)
    ax3.set_xlabel("Temps (s)")
    ax3.set_ylabel("$V_x$ (m/s)")
    ax3.grid(True)
    ax3.legend()

    plt.tight_layout()
    plt.show()