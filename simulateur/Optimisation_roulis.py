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
angle_in = np.array([0.0, 0.0, 0.0]) # Départ à plat

Stab_Alt = True
# Gains d'altitude modérés et stables pour maintenir Z ~ 5m
kp_alt, ki_alt, kd_alt = 6.51000205e+03, 4.45073717e+02, 1.00000000e-02

###############################
# Profil de consigne dynamique (Roulis / Roll)
###############################
def get_consigne_roll(t):
    """
    Génère une consigne dynamique de Roulis (en radians) en fonction du temps t.
    """
    if t < 1.0:
        return np.radians(0.0)
    elif t < 3.5:
        return np.radians(15.0)   # Échelon +15° (Inclinaison droite)
    elif t < 6.0:
        return np.radians(-10.0)  # Échelon -10° (Inclinaison gauche)
    else:
        return np.radians(0.0)    # Retour au neutre

###############################
# Fonction de simulation
###############################
def simuler_drone(x):
    kp_roll, ki_roll, kd_roll = x

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
    historique_roll = []
    historique_consigne = []
    historique_z = []
    historique_vit_y = []

    erreur_angle_cumulee = 0.0
    erreur_alt_cumulee = 0.0
    erreur_vit_y_cumulee = 0.0
    echec_simulation = False

    while N < N_max:
        # 1. Lecture de la consigne dynamique
        roll_cible = get_consigne_roll(t)

        # 2. Poussée de stationnaire d'équilibre compensée
        cos_roll = np.cos(angle_actuel[0])
        cos_pitch = np.cos(angle_actuel[1])
        
        Poids_Moteur = Masse * g / (4 * cos_roll * cos_pitch)
        esc_Moteur = fct.calcul_force2commande(Poids_Moteur)

        esc_m = esc_Moteur

        # 3. Correction PID pour le Roulis
        esc_corr_A, esc_corr_B, esc_corr_C, esc_corr_D = asserv.calcul_correction_PID(
            angle_actuel,
            vit[2],
            altitude_corr=Stab_Alt,
            angle_cible=np.array([roll_cible, 0.0, 0.0]), # Index 0 = Roll (Roulis)
            Kp_roll=kp_roll,  Ki_roll=ki_roll,  Kd_roll=kd_roll,
            Kp_pitch=0.0,     Ki_pitch=0.0,     Kd_pitch=0.0,
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

        # 5. Forces, frottements et moments
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

        # 6. Intégration temporelle de la dynamique
        pos = dyn.position(pos_actuel, pos_ancienne, Forces, Masse, Delta_t=delta_t)
        pos = np.array([pos[0], pos[1], 5])
        angle = dyn.angle(angle_actuel, angle_ancien, Moments, Mat_Inertie, Delta_t=delta_t)

        vit = (pos - pos_actuel) / delta_t

        # Sécurités : divergence d'angle (> 90°) ou crash au sol
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

        # 7. Historique & calcul des erreurs (ITAE)
        historique_t.append(t)
        historique_roll.append(np.degrees(angle[0]))
        historique_consigne.append(np.degrees(roll_cible))
        historique_z.append(pos[2])
        historique_vit_y.append(vit[1])

        err_roll_instantanée = abs(angle[0] - roll_cible)
        erreur_angle_cumulee += t * err_roll_instantanée * delta_t
        erreur_alt_cumulee += t * abs(pos[2] - pos_in[2]) * delta_t
        erreur_vit_y_cumulee += t * abs(vit[1]) * delta_t

        t += delta_t
        N += 1

    return (
        t,
        erreur_angle_cumulee,
        erreur_alt_cumulee,
        erreur_vit_y_cumulee,
        echec_simulation,
        historique_t,
        historique_roll,
        historique_consigne,
        historique_z,
        historique_vit_y,
    )

###############################
# Fonction Coût & Optimisation
###############################
def fonction_cout(x):
    t_stab, err_angle, err_alt, err_vit_y, echec, _, _, _, _, _ = simuler_drone(x)

    if echec:
        return 1e6

    # Pénalise l'erreur de suivi d'angle et la dérive d'altitude
    cout_total = err_angle

    return cout_total

# Bornes raisonnables pour le Roulis d'un quadricoptère de 500g
bornes = [
    (0.01, 2500.0),  # Kp_roll
    (0.01, 50.0),  # Ki_roll
    (0.01, 100.0),  # Kd_roll
]

res = differential_evolution(
    fonction_cout,
    bounds=bornes,
    popsize=12,
    mutation=(0.5, 1.0),
    recombination=0.7,
    maxiter=40,
    # workers=-1  # Utilise tous les cœurs CPU disponibles
)

print("Gains Roulis optimaux [Kp_roll, Ki_roll, Kd_roll] :", res.x)

# Validation et affichage des résultats
(
    _, _, _, _, echec_opt, hist_t, hist_roll, hist_consigne, hist_z, hist_vit_y
) = simuler_drone(res.x)

if not echec_opt:
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 9), sharex=True)

    # 1. Roulis vs Consigne
    ax1.plot(hist_t, hist_roll, "b-", linewidth=1.5, label="Roulis Réel (°)")
    ax1.plot(hist_t, hist_consigne, "r--", linewidth=1.5, label="Consigne (°)")
    ax1.set_ylabel("Angle (°)")
    ax1.set_title("Suivi de Consigne Dynamique (Roulis / Roll)")
    ax1.grid(True)
    ax1.legend()

    # 2. Altitude Z
    ax2.plot(hist_t, hist_z, "g-", linewidth=1.5, label="Altitude Z (m)")
    ax2.set_ylabel("Altitude (m)")
    ax2.grid(True)
    ax2.legend()

    # 3. Vitesse Vy (engendrée par le roulis)
    ax3.plot(hist_t, hist_vit_y, "m-", linewidth=1.5, label="Vitesse $V_y$ (m/s)")
    ax3.set_xlabel("Temps (s)")
    ax3.set_ylabel("$V_y$ (m/s)")
    ax3.grid(True)
    ax3.legend()

    plt.tight_layout()
    plt.show()