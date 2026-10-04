import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math


def matrice_inertie_parallelepipede(Longueur, Largeur, Hauteur, m):
    """
    Longueur : suivant X (b)
    Largeur  : suivant Y (a)
    Hauteur  : suivant Z (c)
    """
    Ixx = m / 12 * (Largeur**2 + Hauteur**2)
    Iyy = m / 12 * (Longueur**2 + Hauteur**2)
    Izz = m / 12 * (Longueur**2 + Largeur**2)

    return np.array([
        [Ixx, 0, 0],
        [0, Iyy, 0],
        [0, 0, Izz]
    ])


def moments_moteur(Force_A, Force_B, Force_C, Force_D, L, l, k_torque=0.02):
    pos_A = np.array([ L, -l, 0])  # Avant-Droit
    pos_B = np.array([ L,  l, 0])  # Avant-Gauche
    pos_C = np.array([-L, -l, 0])  # Arrière-Droit
    pos_D = np.array([-L,  l, 0])  # Arrière-Gauche

    Mom_A = np.cross(pos_A, Force_A)
    Mom_B = np.cross(pos_B, Force_B)
    Mom_C = np.cross(pos_C, Force_C)
    Mom_D = np.cross(pos_D, Force_D)

    Mom_poussee = Mom_A + Mom_B + Mom_C + Mom_D

    f_A, f_B, f_C, f_D = Force_A[2], Force_B[2], Force_C[2], Force_D[2]
    
    # Signes corrigés pour que M_z ne soit pas nul lors d'une commande de lacet
    # A (Av-Dr) et C (Arr-Dr) vs B (Av-Ga) et D (Arr-Ga)
    M_z_torque = (-f_A + f_B - f_C + f_D) * k_torque

    return np.array([Mom_poussee[0], Mom_poussee[1], 0])


def get_rotation_matrix(tx, ty, tz):
    """
    tx: Roll (autour de X)
    ty: Pitch (autour de Y)
    tz: Yaw (autour de Z)
    """
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(tx), -np.sin(tx)],
        [0, np.sin(tx), np.cos(tx)]
    ])

    Ry = np.array([
        [np.cos(ty), 0, np.sin(ty)],
        [0, 1, 0],
        [-np.sin(ty), 0, np.cos(ty)]
    ])

    Rz = np.array([
        [np.cos(tz), -np.sin(tz), 0],
        [np.sin(tz), np.cos(tz), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx


def tracer_pave_drone(ax, pos, R, Longueur, Largeur, Hauteur, drone_demarre):
    """Trace le drone orienté (X+ rouge, Y+ vert)."""
    dx, dy, dz = Longueur / 2, Largeur / 2, Hauteur / 2

    sommets_locaux = np.array([
        [-dx, -dy, -dz],
        [ dx, -dy, -dz],
        [ dx,  dy, -dz],
        [-dx,  dy, -dz],
        [-dx, -dy,  dz],
        [ dx, -dy,  dz],
        [ dx,  dy,  dz],
        [-dx,  dy,  dz],
    ])

    sommets_globaux = np.dot(sommets_locaux, R.T) + pos

    faces = [
        [sommets_globaux[0], sommets_globaux[1], sommets_globaux[2], sommets_globaux[3]], # Bas
        [sommets_globaux[4], sommets_globaux[5], sommets_globaux[6], sommets_globaux[7]], # Haut
        [sommets_globaux[1], sommets_globaux[2], sommets_globaux[6], sommets_globaux[5]], # Avant (X+)
        [sommets_globaux[0], sommets_globaux[3], sommets_globaux[7], sommets_globaux[4]], # Arrière (X-)
        [sommets_globaux[2], sommets_globaux[3], sommets_globaux[7], sommets_globaux[6]], # Gauche (Y+)
        [sommets_globaux[0], sommets_globaux[1], sommets_globaux[5], sommets_globaux[4]], # Droit (Y-)
    ]

    couleur = "cyan" if drone_demarre else "gray"

    poly = Poly3DCollection(
        faces, facecolors=couleur, linewidths=1, edgecolors="darkblue", alpha=0.75
    )
    ax.add_collection3d(poly)



def calculer_force_moteur_continue(intensite, masse, roll_rad, pitch_rad, g = 9.81):
    # Poussée d'équilibre au neutre (0.5)
    denom = max(0.1, math.cos(roll_rad) * math.cos(pitch_rad))
    poussee_eq = (masse * g) / (4.0 * denom)
    
    # Bornes de la commande
    poussee_min = 0.0
    poussee_max = 0.0371 * 100.0 - 0.1195  # 3.5905 N
    
    if intensite <= 0.5:
        # Plage [0.0 , 0.5] -> [poussee_min , poussee_eq]
        return poussee_min + (poussee_eq - poussee_min) * (intensite / 0.5)
    else:
        # Plage [0.5 , 1.0] -> [poussee_eq , poussee_max]
        return poussee_eq + (poussee_max - poussee_eq) * ((intensite - 0.5) / 0.5)


def calcul_commande2controleur(power):
    return power*10

def calcul_force2commande(F):
    return 266.92 * F + 1036.5

def calcul_controleur2force(esc):
    return 0.0037*esc - 3.8313



if __name__ == "__main__":
    POWER = 0
    print(calcul_commande2controleur(POWER))