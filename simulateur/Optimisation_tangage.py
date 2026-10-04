import asservissement as asserv
import dynamique as dyn
import fonctions as fct
import Lecture_data as ld

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import differential_evolution

chemin_fichier = "Donnees_drone.txt"
dic = ld.lire_configuration(chemin_fichier, par_section=True)

###############################
# Données terre & drone
###############################
g = 9.81

Masse = dic["DONNEE DRONE"]["Masse"]
Longueur = dic["DONNEE DRONE"]["Longueur"]
Largeur = dic["DONNEE DRONE"]["Largeur"]
Hauteur = dic["DONNEE DRONE"]["Hauteur"]
L = dic["DONNEE DRONE"]["Moteur_L"]
l = dic["DONNEE DRONE"]["Moteur_l"]
Mat_Inertie = np.array(dic["DONNEE DRONE"]["Matrice_Inertie"])

FORCE_MAX_MOTEUR = dic["DONNEE DRONE"]["Force_MAX"]

###############################
# Conditions initiales & Config
###############################
delta_t = dic["SIMULATION"]["delta_t"]
T_MAX = dic["SIMULATION"]["T_max"]  # Durée de simulation réduisant le temps de calcul
k_frot = dic["SIMULATION"]["k_frot"]

pos_in = np.array([0.0, 0.0, 5.0])
angle_in = np.array([0.0, 0.0, 0.0])
Stab_Alt = True

# Gains d'altitude modérés et stables (maintien Z ~ 5m)
kp_alt, ki_alt, kd_alt = 4.98980340e+01, 8.24779880e-03, 1.34603797e+00

###############################
# Profil de consigne dynamique (Tangage / Pitch)
###############################
def get_consigne_pitch(t):
    """
    Génère une consigne dynamique de Tangage (en radians) en fonction du temps t.
    """
    if t < 1.0:
        return np.radians(0.0)
    elif t < 3.5:
        return np.radians(15.0)   # Échelon +15° (Inclinaison droite)
    elif t < 6.0:
        return np.radians(-10.0)  # Échelon -10° (Inclinaison gauche)
    else:
        return np.radians(0.0)     # Retour au neutre

###############################
# Fonction de simulation

###############################
def simuler_drone(x):
    kp_pitch, ki_pitch, kd_pitch = x

    Poids_global = np.array([0.0, 0.0, -Masse * g])

    t = 0.0
    N = 0
    N_max = int(T_MAX / delta_t)

    pos_actuel = np.copy(pos_in)
    angle_actuel = np.copy(angle_in)
    pos_ancienne = np.copy(pos_actuel)
    angle_ancien = np.copy(angle_actuel)

    pos = np.copy(pos_actuel)
    angle = np.copy(angle_actuel)
    vit = np.array([0.0, 0.0, 0.0])

    historique_t = []
    historique_pitch = []
    historique_consigne = []
    historique_z = []
    historique_vit_x = []

    erreur_angle_cumulee = 0.0
    erreur_alt_cumulee = 0.0
    erreur_vit_x_cumulee = 0.0
    echec_simulation = False

    while N < N_max:
        # 1. Lecture de la consigne dynamique à l'instant t
        pitch_cible = get_consigne_pitch(t)

        # 2. Poussée de stationnaire d'équilibre compensée
        cos_roll = np.cos(angle_actuel[0])
        cos_pitch = np.cos(angle_actuel[1])


        Poids_Moteur = Masse * g / (4 * cos_roll * cos_pitch)
        esc_Moteur = fct.calcul_force2commande(Poids_Moteur)

        esc_m = esc_Moteur

        # 3. Correction PID pour le Tangage (index 1)
        esc_corr_A, esc_corr_B, esc_corr_C, esc_corr_D = asserv.calcul_correction_PID(
            angle_actuel,
            vit[2],
            altitude_corr=Stab_Alt,
            angle_cible=np.array([0.0, pitch_cible, 0.0]), # Index 1 = Pitch (Tangage)
            Kp_roll=0.0,      Ki_roll=0.0,      Kd_roll=0.0,
            Kp_pitch=kp_pitch, Ki_pitch=ki_pitch, Kd_pitch=kd_pitch,
            Kp_yaw=0.1,       Ki_yaw=0.005,     Kd_yaw=0.02,
            Kp_alt=kp_alt,    Ki_alt=ki_alt,    Kd_alt=kd_alt,
            delta_t=delta_t,
            reset_pid=(N == 0)
        )

        f_corr_A = fct.calcul_controleur2force(esc_m + esc_corr_A)
        f_corr_B = fct.calcul_controleur2force(esc_m + esc_corr_B)
        f_corr_C = fct.calcul_controleur2force(esc_m + esc_corr_C)
        f_corr_D = fct.calcul_controleur2force(esc_m + esc_corr_D)

        # 4. Saturation physique des moteurs
        f_A = np.clip(f_corr_A, 0.0, FORCE_MAX_MOTEUR)
        f_B = np.clip(f_corr_B, 0.0, FORCE_MAX_MOTEUR)
        f_C = np.clip(f_corr_C, 0.0, FORCE_MAX_MOTEUR)
        f_D = np.clip(f_corr_D, 0.0, FORCE_MAX_MOTEUR)

        # 5. Calcul des forces, frottements et moments
        vitesse_lin_norme = np.linalg.norm(vit)
        F_frot = -k_frot * vitesse_lin_norme * vit

        R = fct.get_rotation_matrix(angle_actuel[0], angle_actuel[1], angle_actuel[2])
        Force_A_g = np.dot(R, np.array([0, 0, f_A]))
        Force_B_g = np.dot(R, np.array([0, 0, f_B]))
        Force_C_g = np.dot(R, np.array([0, 0, f_C]))
        Force_D_g = np.dot(R, np.array([0, 0, f_D]))

        Forces = np.array([Force_A_g, Force_B_g, Force_C_g, Force_D_g, Poids_global, F_frot])
        Moments = fct.moments_moteur(
            np.array([0, 0, f_A]), np.array([0, 0, f_B]),
            np.array([0, 0, f_C]), np.array([0, 0, f_D]),
            L, l
        )

        # 6. Intégration dynamique (SANS écraser la position Z)
        pos = dyn.position(pos_actuel, pos_ancienne, Forces, Masse, Delta_t=delta_t)
        # pos = np.array([pos[0], pos[1], 5])
        angle = dyn.angle(angle_actuel, angle_ancien, Moments, Mat_Inertie, Delta_t=delta_t)

        vit = (pos - pos_actuel) / delta_t

        # Sécurités : instabilité (> 90°) ou crash sol
        if np.isnan(angle).any() or np.isnan(pos).any() or abs(angle[0]) > (np.pi / 2) or abs(angle[1]) > (np.pi / 2):
            echec_simulation = True
            break

        if pos[2] <= 0.0:
            echec_simulation = True
            break

        # Mise à jour des états
        pos_ancienne = np.copy(pos_actuel)
        pos_actuel = np.copy(pos)
        angle_ancien = np.copy(angle_actuel)
        angle_actuel = np.copy(angle)

        # 7. Historique & Calcul d'erreur ITAE
        historique_t.append(t)
        historique_pitch.append(np.degrees(angle[1]))
        historique_consigne.append(np.degrees(pitch_cible))
        historique_z.append(pos[2])
        historique_vit_x.append(vit[0])

        err_pitch_instatanee = abs(angle[1] - pitch_cible)
        erreur_angle_cumulee += t * err_pitch_instatanee * delta_t
        erreur_alt_cumulee += t * abs(pos[2] - pos_in[2]) * delta_t
        erreur_vit_x_cumulee += t * abs(vit[0]) * delta_t

        t += delta_t
        N += 1

    return (
        t,
        erreur_angle_cumulee,
        erreur_alt_cumulee,
        erreur_vit_x_cumulee,
        echec_simulation,
        historique_t,
        historique_pitch,
        historique_consigne,
        historique_z,
        historique_vit_x,
    )

###############################
# Fonction Coût & Optimisation
###############################
def fonction_cout(x):
    t_stab, err_angle, err_alt, err_vit_x, echec, _, _, _, _, _ = simuler_drone(x)

    if echec:
        return 1e6

    # Pénalise l'erreur d'angle et la dérive d'altitude
    cout_total =  err_angle

    return cout_total

bornes = [
    (0.01, 1500.0),  # Kp_pitch
    (0.01, 50.0),  # Ki_pitch
    (0.01, 100.0),  # Kd_pitch
]

res = differential_evolution(
    fonction_cout,
    bounds=bornes,
    popsize=12,
    mutation=(0.5, 1.0),
    recombination=0.7,
    maxiter=40,
    # workers=-1  # Multi-threading pour accélérer le calcul
)

print("Gains Tangage optimaux [Kp_pitch, Ki_pitch, Kd_pitch] :", res.x)

# Affichage des résultats
(
    _, _, _, _, echec_opt, hist_t, hist_pitch, hist_consigne, hist_z, hist_vit_x
) = simuler_drone(res.x)

if not echec_opt:
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 9), sharex=True)

    # 1. Tangage vs Consigne
    ax1.plot(hist_t, hist_pitch, "b-", linewidth=1.5, label="Tangage Réel (°)")
    ax1.plot(hist_t, hist_consigne, "r--", linewidth=1.5, label="Consigne Dynamique (°)")
    ax1.set_ylabel("Angle (°)")
    ax1.set_title("Suivi de Consigne Dynamique (Tangage)")
    ax1.grid(True)
    ax1.legend()

    # 2. Altitude Z
    ax2.plot(hist_t, hist_z, "g-", linewidth=1.5, label="Altitude Z (m)")
    ax2.set_ylabel("Altitude (m)")
    ax2.grid(True)
    ax2.legend()

    # 3. Vitesse Vx
    ax3.plot(hist_t, hist_vit_x, "m-", linewidth=1.5, label="Vitesse $V_x$ (m/s)")
    ax3.set_xlabel("Temps (s)")
    ax3.set_ylabel("$V_x$ (m/s)")
    ax3.grid(True)
    ax3.legend()

    plt.tight_layout()
    plt.show()