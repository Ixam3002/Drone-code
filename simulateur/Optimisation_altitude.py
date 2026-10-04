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


def get_consigne_altitude(t):
    """
    Retourne (vitesse_cible_z, asservissement_actif)
    """
    if t < 1:
        return 1.0, False  # Phase 1 : Vitesse imposée à 1 m/s
    elif t < 3.0:
        return 0.0, True   # Phase 2 : Asservissement actif pour v_z = 0 m/s

    elif t < 4:
        return 2.0, False
    else:
        return 0.0, True

def simuler_drone(x):
    kp_alt, ki_alt, kd_alt = x

    Poids_global = np.array([0.0, 0.0, -Masse * g])

    t = 0.0
    N = 0
    N_max = int(T_MAX / delta_t)

    pos_actuel = np.copy(pos_in)
    angle_actuel = np.copy(angle_in)
    pos_ancienne = np.copy(pos_actuel)
    angle_ancien = np.copy(angle_actuel)

    vit = np.array([0.0, 0.0, 0.0])

    historique_t = []
    historique_consigne = []
    historique_z = []
    historique_vit = []

    erreur_vit_cumulee = 0.0
    echec_simulation = False

    while N < N_max:
        vit_cible, Stab_Alt = get_consigne_altitude(t)

        if not Stab_Alt:
            # t < 2s : ascension forcée cinématique
            vit = np.array([0.0, 0.0, vit_cible])
            pos = pos_actuel + vit * delta_t
            angle = np.copy(angle_actuel)
        else:
            # t >= 2s : asservissement PID en dynamique

            ###############################
            # Force Stabilisation
            ###############################
            Poids_Moteur = Masse * g / (4 * np.cos(angle[0]) * np.cos(angle[1]))
            esc_Moteur = fct.calcul_force2commande(Poids_Moteur)

            esc_m = esc_Moteur

            esc_corr_A, esc_corr_B, esc_corr_C, esc_corr_D = asserv.calcul_correction_PID(
                angle_actuel,
                vit[2],
                altitude_corr=Stab_Alt,
                angle_cible=np.array([0.0, 0.0, 0.0]),
                Kp_roll=0, Kp_pitch=0, Kp_yaw=0.0, Kp_alt=kp_alt,
                Ki_roll=0, Ki_pitch=0, Ki_yaw=0.0, Ki_alt=ki_alt,
                Kd_roll=0, Kd_pitch=0, Kd_yaw=0.0, Kd_alt=kd_alt,
                delta_t=delta_t,
                reset_pid=(N == int(2.0 / delta_t))
            )

            f_corr_A = fct.calcul_controleur2force(esc_m + esc_corr_A)
            f_corr_B = fct.calcul_controleur2force(esc_m + esc_corr_B)
            f_corr_C = fct.calcul_controleur2force(esc_m + esc_corr_C)
            f_corr_D = fct.calcul_controleur2force(esc_m + esc_corr_D)

            f_A = np.clip(f_corr_A, 0.0, FORCE_MAX_MOTEUR)
            f_B = np.clip(f_corr_B, 0.0, FORCE_MAX_MOTEUR)
            f_C = np.clip(f_corr_C, 0.0, FORCE_MAX_MOTEUR)
            f_D = np.clip(f_corr_D, 0.0, FORCE_MAX_MOTEUR)

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

            pos = dyn.position(pos_actuel, pos_ancienne, Forces, Masse, Delta_t=delta_t)
            angle = dyn.angle(angle_actuel, angle_ancien, Moments, Mat_Inertie, Delta_t=delta_t)

            vit = (pos - pos_actuel) / delta_t

        # Sécurités (crash au sol ou basculement)
        if np.isnan(angle).any() or np.isnan(pos).any() or abs(angle[1]) > (np.pi / 2) or pos[2] <= 0.0:
            echec_simulation = True
            break

        # Mise à jour des états
        pos_ancienne = np.copy(pos_actuel)
        pos_actuel = np.copy(pos)
        angle_ancien = np.copy(angle_actuel)
        angle_actuel = np.copy(angle)

        # Enregistrement
        historique_t.append(t)
        historique_consigne.append(vit_cible)
        historique_z.append(pos[2])
        historique_vit.append(vit[2])

        # Calcul d'erreur ITAE sur la vitesse verticale
        err_instantanée = abs(vit[2] - vit_cible)
        erreur_vit_cumulee += t * err_instantanée * delta_t

        t += delta_t
        N += 1

    return t, erreur_vit_cumulee, echec_simulation, historique_t, historique_consigne, historique_z, historique_vit

###############################
# Fonction Coût & Optimisation
###############################
def fonction_cout(x):
    _, err_vit, echec, _, _, _, _ = simuler_drone(x)
    if echec:
        return 1e6
    return err_vit

bornes = [
    (0.01, 50),  # Kp_alt
    (0.00, 20),   # Ki_alt
    (0.01, 50),   # Kd_alt
]

# Lancement de l'optimisation
res = differential_evolution(
    fonction_cout,
    bounds=bornes,
    popsize=10,
    mutation=(0.5, 1.0),
    recombination=0.7,
    maxiter=40,
    #workers=-1  # Utilise tous les cœurs CPU disponibles
)

print("Gains optimaux [Kp, Ki, Kd] :", res.x)

# Simulation finale et affichage
_, _, _, hist_t, hist_consigne, hist_z, hist_vit = simuler_drone(res.x)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)

# Graphique Vitesse Z
ax1.plot(hist_t, hist_vit, "b-", linewidth=1.5, label="$V_z$ Réelle (m/s)")
ax1.plot(hist_t, hist_consigne, "r--", linewidth=1.5, label="Consigne $V_z$ (m/s)")
ax1.set_ylabel("Vitesse verticale (m/s)")
ax1.set_title("Réponse en vitesse et stabilisation à t = 2s")
ax1.grid(True)
ax1.legend()

# Graphique Altitude Z
ax2.plot(hist_t, hist_z, "g-", linewidth=1.5, label="Altitude $Z$ (m)")
ax2.set_xlabel("Temps (s)")
ax2.set_ylabel("Altitude (m)")
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()