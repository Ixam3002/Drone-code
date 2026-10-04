import numpy as np


# def position(pos_actuel, pos_ancienne, Forces, masse, Delta_t=0.01):
#     """Calcul de la nouvelle position par intégration de Verlet."""
#     Sum_Forces = np.sum(Forces, axis=0)
#     pos = (Sum_Forces / masse) * (Delta_t**2) + 2 * pos_actuel - pos_ancienne
#     return pos


import numpy as np

def position(pos_actuel, pos_ancienne, Forces, masse, k=0.1, Delta_t=0.01, modele="quadratique"):
    """
    Calcul de la nouvelle position par intégration de Verlet en incluant les frottements de l'air.
    
    Parameters:
    -----------
    - k : float, coefficient de frottement de l'air.
    - modele : str, 'quadratique' (réaliste pour l'air : F = -k * ||v|| * v) 
               ou 'lineaire' (pour fluides très visqueux : F = -k * v).
    """
    # # 1. Estimation de la vitesse courante
    # vitesse = (pos_actuel - pos_ancienne) / Delta_t
    # norme_v = np.linalg.norm(vitesse)
    
    # # 2. Calcul du frottement de l'air
    # if modele == "quadratique":
    #     frot_air = -k * norme_v * vitesse
    # else:
    #     frot_air = -k * vitesse

    # 3. Somme des forces externes + frottement
    Sum_Forces = np.sum(Forces, axis=0) #+ frot_air
    
    # 4. Intégration de Verlet
    pos = 2 * pos_actuel - pos_ancienne + (Sum_Forces / masse) * (Delta_t**2)
    
    return pos


def angle(angle_actuel, angle_ancien, Moments, Inertie, Delta_t=0.01, amortissement=0):
    """Calcul du nouvel angle avec coefficient d'amortissement rotatif."""
    accel_angulaire = np.linalg.solve(Inertie, Moments)
    
    # Intégration de Verlet avec amortissement de la vitesse angulaire
    vitesse_angulaire = (angle_actuel - angle_ancien) * (1.0 - amortissement)
    angle_nouveau = angle_actuel + vitesse_angulaire + accel_angulaire * (Delta_t**2)
    
    return angle_nouveau