import matplotlib.pyplot as plt
import numpy as np

import dynamique as dyn
import fonctions as fct

###############################
# Données terre & drone
###############################
g = 9.81
Lmoteur = 0.2  # m
Masse = 0.3  # kg
Poids = np.array([0, 0, Masse * g])  # N
Longueur = 0.12  # m
Largeur = 0.08  # m

L = 0.261  # m
l = 0.141  # m

Mat_Inertie = fct.matrice_inertie_parallelepipede(Largeur, Longueur, 0.05, Masse)

###############################
# Données moteur
###############################
Power_motor = 100
RPM = 5000
Force = 3.1

Force_A = np.array([0, 0, Force/4])
Force_B = np.array([0, 0, Force/4 - 0.01])
Force_C = np.array([0, 0, Force/4])
Force_D = np.array([0, 0, Force/4 - 0.01])


###############################
# Initialisation
###############################
pos = np.array([0.0, 0.0, 0.0])
pos_actuel = np.copy(pos)

angle = np.array([0.0, 0.0, 0.0])
angle_actuel = np.copy(angle)

T = 10  # s
delta_t = 0.1  # s
N = int(T / delta_t)


###############################
# Calcul de la trajectoire
###############################

plt.ion()  # Active le mode interactif
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(projection="3d")

scale = 0.5  # Taille du repère local

# On garde une trace des positions pour afficher la trajectoire
trajectoire_x, trajectoire_y, trajectoire_z = [], [], []

for i in range(N):
    pos_ancienne = np.copy(pos_actuel)
    pos_actuel = np.copy(pos)

    angle_ancien = np.copy(angle_actuel)
    angle_actuel = np.copy(angle)

    R = fct.get_rotation_matrix(angle[0], angle[1], angle[2])

    Force_A = np.dot(R, Force_A)
    Force_B = np.dot(R, Force_B)
    Force_C = np.dot(R, Force_C)
    Force_D = np.dot(R, Force_D)

    Poids = np.dot(R, Poids)


    Forces = np.array([Force_A, Force_B, Force_C, Force_D, -Poids])
    Moments = fct.moments_moteur(Force_A, Force_B, Force_C, Force_D, L, l)

    # Calcul dynamique
    # pos = dyn.position(
    #     pos_actuel, pos_ancienne, Forces, Masse, Delta_t=delta_t
    # )

    pos = np.array([0, 0, 0])


    angle = dyn.angle(
        angle_actuel, angle_ancien, Moments, Mat_Inertie, Delta_t=delta_t
    )

    # Enregistrement pour le tracé
    trajectoire_x.append(pos[0])
    trajectoire_y.append(pos[1])
    trajectoire_z.append(pos[2])

    # --- Mise à jour de l'affichage ---
    ax.cla()  # Efface l'image précédente

    # 1. Trajectoire complète parcourue jusqu'ici
    # ax.plot(
    #     trajectoire_x,
    #     trajectoire_y,
    #     trajectoire_z,
    #     "r--",
    #     alpha=0.5,
    #     label="Trajectoire",
    # )

    # 2. Position actuelle du centre de masse G
    ax.scatter(
        pos[0], pos[1], pos[2], color="red", s=50, marker="o", label="Drone (G)"
    )

    # 3. Calcul du repère local orienté au point actuel
    
    u_x, u_y, u_z = R[:, 0] * scale, R[:, 1] * scale, R[:, 2] * scale

    # Axes du repère local
    ax.quiver(
        pos[0],
        pos[1],
        pos[2],
        u_x[0],
        u_x[1],
        u_x[2],
        color="r",
        linewidth=2,
        label="Axe X local",
    )
    ax.quiver(
        pos[0],
        pos[1],
        pos[2],
        u_y[0],
        u_y[1],
        u_y[2],
        color="g",
        linewidth=2,
        label="Axe Y local",
    )
    ax.quiver(
        pos[0],
        pos[1],
        pos[2],
        u_z[0],
        u_z[1],
        u_z[2],
        color="b",
        linewidth=2,
        label="Axe Z local",
    )

    # Définition des limites fixes des axes (Ajuste selon ton domaine de vol)
    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    ax.set_zlim([0, 10])

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(f"Simulation drone - t = {round(i * delta_t, 2)} s")

    plt.draw()
    plt.pause(0.01)  # Temps de pause pour rafraîchir l'écran

plt.ioff()
plt.show()