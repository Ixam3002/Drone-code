import numpy as np


def asservissement_proportionnel_angle(
    angle_actuel,
    angle_consigne,
    Forces,
    Kp_roll=0.2,
    Kp_pitch=0.2,
    Kp_yaw=0.1,
):
    # Erreur = Consigne - Mesure
    erreur_roll = angle_consigne[0] - angle_actuel[0]
    erreur_pitch = angle_consigne[1] - angle_actuel[1]
    erreur_yaw = angle_consigne[2] - angle_actuel[2]

    cmd_roll = Kp_roll * erreur_roll
    cmd_pitch = Kp_pitch * erreur_pitch
    cmd_yaw = Kp_yaw * erreur_yaw

    # Corrections inversées pour créer un rappel vers 0 :
    # Si pitch > 0 (le drone cabre), erreur_pitch < 0 -> il faut AUGMENTER la poussée à l'avant (A et B)
    f_A = Forces[0] - cmd_roll - cmd_pitch - cmd_yaw
    f_B = Forces[1] - cmd_roll - cmd_pitch + cmd_yaw
    f_C = Forces[2] + cmd_roll + cmd_pitch - cmd_yaw
    f_D = Forces[3] + cmd_roll + cmd_pitch + cmd_yaw

    return max(0.0, f_A), max(0.0, f_B), max(0.0, f_C), max(0.0, f_D)


erreur_integrale_roll = 0.0
erreur_integrale_pitch = 0.0
erreur_integrale_yaw = 0.0
erreur_integrale_alt = 0.0


def calcul_correction_PI(
    angle_actuel,
    angle_cible=np.array([0.0, 0.0, 0.0]),
    Kp_roll=0.2,
    Kp_pitch=0.2,
    Kp_yaw=0.1,
    Ki_roll=0.01,
    Ki_pitch=0.01,
    Ki_yaw=0.005,
    delta_t=0.05,
    reset_integrale=False,
):
    """Calcule uniquement les forces de correction pour stabiliser le drone à angle_cible (0° par défaut)."""
    global erreur_integrale_roll, erreur_integrale_pitch, erreur_integrale_yaw

    if reset_integrale:
        erreur_integrale_roll = 0.0
        erreur_integrale_pitch = 0.0
        erreur_integrale_yaw = 0.0

    # 1. Erreur par rapport à la consigne de stabilisation (0, 0, 0)
    erreur_roll = angle_cible[0] - angle_actuel[0]
    erreur_pitch = angle_cible[1] - angle_actuel[1]
    erreur_yaw = angle_cible[2] - angle_actuel[2]

    # 2. Intégration
    erreur_integrale_roll += erreur_roll * delta_t
    erreur_integrale_pitch += erreur_pitch * delta_t
    erreur_integrale_yaw += erreur_yaw * delta_t

    limite_integrale = 1.0
    erreur_integrale_roll = np.clip(erreur_integrale_roll, -limite_integrale, limite_integrale)
    erreur_integrale_pitch = np.clip(erreur_integrale_pitch, -limite_integrale, limite_integrale)
    erreur_integrale_yaw = np.clip(erreur_integrale_yaw, -limite_integrale, limite_integrale)

    # 3. Commandes de correction
    corr_roll = (Kp_roll * erreur_roll) + (Ki_roll * erreur_integrale_roll)
    corr_pitch = (Kp_pitch * erreur_pitch) + (Ki_pitch * erreur_integrale_pitch)
    corr_yaw = (Kp_yaw * erreur_yaw) + (Ki_yaw * erreur_integrale_yaw)

    # 4. Répartition des corrections sur les moteurs
    # A (Av-Dr) | B (Av-Ga) | C (Arr-Dr) | D (Arr-Ga)
    f_corr_A = + corr_roll - corr_pitch - corr_yaw
    f_corr_B = - corr_roll - corr_pitch + corr_yaw
    f_corr_C = + corr_roll + corr_pitch + corr_yaw
    f_corr_D = - corr_roll + corr_pitch - corr_yaw

    return f_corr_A, f_corr_B, f_corr_C, f_corr_D


import numpy as np

# Initialisation des variables globales d'état PID
erreur_integrale_roll = 0.0
erreur_integrale_pitch = 0.0
erreur_integrale_yaw = 0.0
erreur_integrale_alt = 0.0

erreur_prec_roll = 0.0
erreur_prec_pitch = 0.0
erreur_prec_yaw = 0.0
erreur_prec_alt = 0.0


def calcul_correction_PID(
    angle_actuel,
    Vitesse_Z,
    altitude_corr=True,
    angle_cible=np.array([0.0, 0.0, 0.0]),
    Kp_roll=0.2,
    Kp_pitch=0.2,
    Kp_yaw=0.1,
    Kp_alt=0.1,
    Ki_roll=0.01,
    Ki_pitch=0.01,
    Ki_yaw=0.005,
    Ki_alt=0.005,
    Kd_roll=0.05,
    Kd_pitch=0.05,
    Kd_yaw=0.02,
    Kd_alt=0.02,
    delta_t=0.05,
    reset_pid=False,
):
    """Calcule les forces de correction PID complètes pour stabiliser le drone."""
    global erreur_integrale_roll, erreur_integrale_pitch, erreur_integrale_yaw, erreur_integrale_alt
    global erreur_prec_roll, erreur_prec_pitch, erreur_prec_yaw, erreur_prec_alt

    # 1. Calcul des erreurs
    erreur_roll = angle_cible[0] - angle_actuel[0]
    erreur_pitch = angle_cible[1] - angle_actuel[1]
    erreur_yaw = angle_cible[2] - angle_actuel[2]
    erreur_alt = 0 - Vitesse_Z

    # Réinitialisation des états (intégrale et dérivée)
    if reset_pid:
        erreur_integrale_roll = 0.0
        erreur_integrale_pitch = 0.0
        erreur_integrale_yaw = 0.0
        erreur_integrale_alt = 0.0

        erreur_prec_roll = erreur_roll
        erreur_prec_pitch = erreur_pitch
        erreur_prec_yaw = erreur_yaw
        erreur_prec_alt = erreur_alt

    # 2. Terme Intégral (accumulation)
    erreur_integrale_roll += erreur_roll * delta_t
    erreur_integrale_pitch += erreur_pitch * delta_t
    erreur_integrale_yaw += erreur_yaw * delta_t
    erreur_integrale_alt += erreur_alt * delta_t

    limite_integrale = 1.0
    erreur_integrale_roll = np.clip(
        erreur_integrale_roll, -limite_integrale, limite_integrale
    )
    erreur_integrale_pitch = np.clip(
        erreur_integrale_pitch, -limite_integrale, limite_integrale
    )
    erreur_integrale_yaw = np.clip(
        erreur_integrale_yaw, -limite_integrale, limite_integrale
    )

    erreur_integrale_alt = np.clip(
            erreur_integrale_alt, -limite_integrale, limite_integrale
        )

    # 3. Terme Dérivatif (variation de l'erreur dans le temps)
    d_erreur_roll = (erreur_roll - erreur_prec_roll) / delta_t
    d_erreur_pitch = (erreur_pitch - erreur_prec_pitch) / delta_t
    d_erreur_yaw = (erreur_yaw - erreur_prec_yaw) / delta_t
    d_erreur_alt = (erreur_alt - erreur_prec_alt) / delta_t

    # Sauvegarde de l'erreur courante pour le prochain pas de temps
    erreur_prec_roll = erreur_roll
    erreur_prec_pitch = erreur_pitch
    erreur_prec_yaw = erreur_yaw
    erreur_prec_alt = erreur_alt

    # 4. Commandes de correction PID
    corr_roll = (
        (Kp_roll * erreur_roll)
        + (Ki_roll * erreur_integrale_roll)
        + (Kd_roll * d_erreur_roll)
    )
    corr_pitch = (
        (Kp_pitch * erreur_pitch)
        + (Ki_pitch * erreur_integrale_pitch)
        + (Kd_pitch * d_erreur_pitch)
    )
    corr_yaw = (
        (Kp_yaw * erreur_yaw)
        + (Ki_yaw * erreur_integrale_yaw)
        + (Kd_yaw * d_erreur_yaw)
    )

    corr_alt = (
            (Kp_alt * erreur_alt)
            + (Ki_alt * erreur_integrale_alt)
            + (Kd_alt * d_erreur_alt)
        )

    # 5. Répartition sur les 4 moteurs :
    # A (Av-Dr) | B (Av-Ga) | C (Arr-Dr) | D (Arr-Ga)

    if altitude_corr:
        f_corr_A = -corr_roll - corr_pitch - corr_yaw + corr_alt
        f_corr_B = +corr_roll - corr_pitch + corr_yaw + corr_alt
        f_corr_C = -corr_roll + corr_pitch + corr_yaw + corr_alt
        f_corr_D = +corr_roll + corr_pitch - corr_yaw + corr_alt

    else:
        f_corr_A = -corr_roll - corr_pitch - corr_yaw
        f_corr_B = +corr_roll - corr_pitch + corr_yaw
        f_corr_C = -corr_roll + corr_pitch + corr_yaw
        f_corr_D = +corr_roll + corr_pitch - corr_yaw

    return f_corr_A, f_corr_B, f_corr_C, f_corr_D