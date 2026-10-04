import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize


import asservissement as asserv
import dynamique as dyn
import fonctions as fct


def affichage(trajectoire_x, trajectoire_y, trajectoire_z, historique_roll, historique_pitch):
    # 2. Récupération de la position et de l'orientation finales
    pos_finale = np.array(
        [trajectoire_x[-1], trajectoire_y[-1], trajectoire_z[-1]]
    )

    roll_final = np.radians(historique_roll[-1])
    pitch_final = np.radians(historique_pitch[-1])
    yaw_final = 0.0 # Conservation du lacet initial

    # Calcul de la matrice de rotation finale
    R_final = fct.get_rotation_matrix(roll_final, pitch_final, yaw_final)

    # 3. Tracé 3D
    fig_3d = plt.figure(figsize=(9, 7))
    ax_3d = fig_3d.add_subplot(projection="3d")
    ax_3d.cla()

    # Repère du sol
    ax_3d.plot_surface(
        np.array([[-5, 5], [-5, 5]]),
        np.array([[-5, -5], [5, 5]]),
        np.array([[0, 0], [0, 0]]),
        color="gray",
        alpha=0.2,
    )

    # Courbe de trajectoire
    ax_3d.plot(
        trajectoire_x,
        trajectoire_y,
        trajectoire_z,
        "r--",
        linewidth=1.5,
        alpha=0.6,
        label="Trajectoire",
    )

    # Tracé du pavé du drone à la position et orientation finales
    fct.tracer_pave_drone(
        ax_3d, pos_finale, R_final, Longueur, Largeur, Hauteur, True
    )

    # Vecteurs du repère local (X: Rouge, Y: Vert, Z: Bleu)
    scale_axes = 0.2
    u_x, u_y, u_z = (
        R_final[:, 0] * scale_axes,
        R_final[:, 1] * scale_axes,
        R_final[:, 2] * scale_axes,
    )
    ax_3d.quiver(
        pos_finale[0],
        pos_finale[1],
        pos_finale[2],
        u_x[0],
        u_x[1],
        u_x[2],
        color="red",
        linewidth=2,
    )
    ax_3d.quiver(
        pos_finale[0],
        pos_finale[1],
        pos_finale[2],
        u_y[0],
        u_y[1],
        u_y[2],
        color="green",
        linewidth=2,
    )
    ax_3d.quiver(
        pos_finale[0],
        pos_finale[1],
        pos_finale[2],
        u_z[0],
        u_z[1],
        u_z[2],
        color="blue",
        linewidth=2,
    )

    # Recadrage de la caméra sur le drone
    ax_3d.set_xlim([pos_finale[0] - 2, pos_finale[0] + 2])
    ax_3d.set_ylim([pos_finale[1] - 2, pos_finale[1] + 2])
    ax_3d.set_zlim([max(0, pos_finale[2] - 1), pos_finale[2] + 3])

    ax_3d.set_xlabel("X (m)")
    ax_3d.set_ylabel("Y (m)")
    ax_3d.set_zlabel("Z (m)")
    # ax_3d.set_title(
    #     f"Drone à t = {round(t[-1], 2)} s | Roll: {historique_roll[-1]:.1f}° | Pitch: {historique_pitch[-1]:.1f}°"
    # )
    ax_3d.legend()

    plt.show()