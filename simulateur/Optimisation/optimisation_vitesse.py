import asservissement as asserv
import dynamique as dyn
import fonctions as fct
import matplotlib.pyplot as plt
import numpy as np
from scipy import optimize

###############################
# Données terre & drone
###############################
g = 9.81
Masse = 0.3  # kg
Poids_global = np.array([0.0, 0.0, -Masse * g])

Longueur = 0.12  # m (Axe X local)
Largeur = 0.08  # m (Axe Y local)
Hauteur = 0.05  # m (Axe Z local)

L = Longueur / 2.0  # demi-longueur
l = Largeur / 2.0  # demi-largeur

Mat_Inertie = fct.matrice_inertie_parallelepipede(
    Longueur, Largeur, Hauteur, Masse
)

###############################
# Conditions initiales & Seuils
###############################
pos_in = np.array([0.0, 0.0, 5.0])
angle_in = np.array([0.0, 0.0, 0.0])

delta_t = 0.05
poussee_equilibre = (Masse * g) / 4.0

perturbation = 0.01
f_A_pert = poussee_equilibre - perturbation
f_B_pert = poussee_equilibre - perturbation
f_C_pert = poussee_equilibre + perturbation
f_D_pert = poussee_equilibre + perturbation

# Seuils de stabilisation
reussite_angle = np.radians(0.5)  # 0.5°
reussite_vitesse = np.radians(0.1)  # 0.1°/s
reussite_vit_lin = 0.05  # Vitesse linéaire max tolérée (0.05 m/s)


###############################
# Fonction de simulation du drone
###############################
def simuler_drone(x):
    kp_pitch, ki_pitch, kd_pitch = x

    t = 0.0
    N = 0
    N_max = 1500  # 75s max

    pos = np.copy(pos_in)
    angle = np.copy(angle_in)
    pos_actuel = np.copy(pos)
    angle_actuel = np.copy(angle)

    historique_t = []
    historique_pitch = []
    historique_roll = []
    historique_vit_norm = []

    erreur_angle_cumulee = 0.0
    erreur_vit_cumulee = 0.0
    crash_sol = False

    while N < N_max:
        # 1. Calcul de la correction PID
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

        # 2. Application des poussées
        if t <= 0.5:
            f_A = max(0.0, f_A_pert + f_corr_A)
            f_B = max(0.0, f_B_pert + f_corr_B)
            f_C = max(0.0, f_C_pert + f_corr_C)
            f_D = max(0.0, f_D_pert + f_corr_D)
        else:
            f_A = max(0.0, poussee_equilibre + f_corr_A)
            f_B = max(0.0, poussee_equilibre + f_corr_B)
            f_C = max(0.0, poussee_equilibre + f_corr_C)
            f_D = max(0.0, poussee_equilibre + f_corr_D)

        # 3. Dynamique
        R = fct.get_rotation_matrix(angle[0], angle[1], angle[2])
        Force_A_g = np.dot(R, np.array([0, 0, f_A]))
        Force_B_g = np.dot(R, np.array([0, 0, f_B]))
        Force_C_g = np.dot(R, np.array([0, 0, f_C]))
        Force_D_g = np.dot(R, np.array([0, 0, f_D]))

        Forces = np.array(
            [Force_A_g, Force_B_g, Force_C_g, Force_D_g, Poids_global]
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

        # 4. Dérivée de la position : vitesse linéaire 3D
        vit = (pos - pos_actuel) / delta_t
        vitesse_lin_norme = np.linalg.norm(vit)

        # Contrainte Sol : crash si altitude <= 0m
        if pos[2] <= 0.0:
            crash_sol = True
            break

        # 5. Mesures et historisation
        roulis_deg = np.degrees(angle[0])
        tangage_deg = np.degrees(angle[1])

        historique_t.append(t)
        historique_roll.append(roulis_deg)
        historique_pitch.append(tangage_deg)
        historique_vit_norm.append(vitesse_lin_norme)

        # Accumulation de l'erreur
        erreur_angle_cumulee += t * abs(angle[1]) * delta_t
        erreur_vit_cumulee += t * vitesse_lin_norme * delta_t

        # 6. Condition d'arrêt complète
        vitesse_pitch = (angle[1] - angle_actuel[1]) / delta_t
        est_stabilise = (
            abs(angle[1]) <= reussite_angle
            and abs(vitesse_pitch) <= reussite_vitesse
            and vitesse_lin_norme <= reussite_vit_lin
        )

        if t > 0.5 and est_stabilise:
            break

        t += delta_t
        N += 1

    if N >= N_max:
        t = 100.0

    return (
        t,
        erreur_angle_cumulee,
        erreur_vit_cumulee,
        crash_sol,
        historique_t,
        historique_pitch,
        historique_vit_norm,
    )


###############################
# Fonction Coût pour SciPy
###############################
def fonction_cout(x):
    t_stab, err_angle, err_vit, crash, _, _, _ = simuler_drone(x)

    # Pénalité absolue si le drone touche le sol
    if crash:
        return 1e5

    # Coût : Temps + Erreur d'angle + Erreur de vitesse linéaire
    return t_stab + 2.0 * err_angle + 1.5 * err_vit


###############################
# Lancement de l'Optimisation
###############################
x0 = [0.02, 0.01, 0.005]

bornes = [
    (0.0, 5.0),  # Kp >= 0
    (0.0, 1.0),  # Ki >= 0
    (0.0, 2.0),  # Kd >= 0
]

res = optimize.minimize(
    fonction_cout,
    x0,
    method="L-BFGS-B",
    bounds=bornes,
)

kp_opt, ki_opt, kd_opt = res.x

print("--- RÉSULTATS OPTIMISATION ---")
print(f"Kp optimal = {kp_opt:.4f}")
print(f"Ki optimal = {ki_opt:.4f}")
print(f"Kd optimal = {kd_opt:.4f}")

# Simulation finale
t_opt, _, _, crash_opt, hist_t, hist_pitch, hist_vit = simuler_drone(res.x)

if crash_opt:
    print("Attention : Le réglage optimal fait crasher le drone au sol !")
else:
    print(f"Temps de stabilisation optimal = {t_opt:.2f} s")

###############################
# Graphiques de synthèse
###############################
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)

ax1.plot(hist_t, hist_pitch, "b-", linewidth=2, label="Tangage (°)")
ax1.axhline(
    y=np.degrees(reussite_angle), color="r", linestyle="--", alpha=0.6
)
ax1.axhline(
    y=-np.degrees(reussite_angle), color="r", linestyle="--", alpha=0.6
)
ax1.set_ylabel("Angle (°)")
ax1.set_title("Stabilisation de l'angle et de la vitesse linéaire")
ax1.grid(True)
ax1.legend()

ax2.plot(hist_t, hist_vit, "g-", linewidth=2, label="Vitesse linéaire (m/s)")
ax2.axhline(
    y=reussite_vit_lin,
    color="r",
    linestyle="--",
    alpha=0.6,
    label="Seuil vitesse",
)
ax2.set_xlabel("Temps (s)")
ax2.set_ylabel("Vitesse (m/s)")
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()