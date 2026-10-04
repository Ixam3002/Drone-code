import sys
import math
import numpy as np
import pygame

# Imports de tes modules existants
import asservissement as asserv
import dynamique as dyn
import fonctions as fct
import manette_xbox_commande as mxc
import Lecture_data as ld

pygame.init()
pygame.joystick.init()

# ---------------------------------------------------------
# CONSTANTES & CONFIGURATION FENÊTRE
# ---------------------------------------------------------
WIDTH, HEIGHT = 1280, 720
FPS = 60

SCREEN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Drone Flight Simulator — Pygame Edition")
CLOCK = pygame.time.Clock()

# Couleurs (Palette Dark UI / Sci-Fi)
BG_COLOR = (18, 22, 28)
PANEL_BG = (28, 35, 45)
GRID_COLOR = (40, 55, 70)
TEXT_COLOR = (220, 230, 240)
CYAN = (0, 225, 255)
GREEN = (46, 204, 113)
RED = (231, 76, 60)
ORANGE = (230, 126, 34)
WHITE = (255, 255, 255)

FONT_S = pygame.font.SysFont("Consolas", 12)
FONT_M = pygame.font.SysFont("Consolas", 16, bold=True)
FONT_L = pygame.font.SysFont("Consolas", 22, bold=True)

# SEUIL DE VITESSE ANGULAIRE CRITIQUE (°/s)
SEUIL_VIT_ANGULAIRE_MAX = 500.0

# ---------------------------------------------------------
# MANETTE XBOX
# ---------------------------------------------------------
manette = None
if pygame.joystick.get_count() > 0:
    manette = pygame.joystick.Joystick(0)
    manette.init()
    print(f"Manette connectée : {manette.get_name()}")
else:
    print("Attention : Aucune manette détectée. Utilise Z/Q/S/D/Espace pour le test clavier.")
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
#conditions initiales
###############################

pos = np.array([0.0, 0.0, 0.0])
pos_actuel = np.copy(pos)
angle = np.array([0.0, 0.0, 0.0])
angle_actuel = np.copy(angle)
angle_ancien = np.copy(angle)

delta_t = dic["SIMULATION"]["delta_t"]
k_frot = dic["SIMULATION"]["k_frot"]

t = 0.0
drone_demarre = False


kp_pitch, ki_pitch, kd_pitch = 1.49688032e+03, 1.00000000e-02, 4.68476324e+01
kp_roll, ki_roll, kd_roll = 2.5000000e+03, 1.0000000e-02, 9.9814911e+01
kp_alt, ki_alt, kd_alt = 4.98980340e+01, 8.24779880e-03, 1.34603797e+00

axes_active = [True, True, True]  # [Roll, Pitch, Yaw]
historique_roll = []
historique_pitch = []
historique_t = []

esc_com = 1000

ALTITUDE = 0
POWER = 0
Stab_Alt = True

Poids_global = np.array([0.0, 0.0, -Masse * g])

# ---------------------------------------------------------
# MOTEUR DE PROJECTION 3D -> 2D
# ---------------------------------------------------------
def project_3d_to_2d(point_3d, cam_pos, center_screen, fov=400):
    rel_x = point_3d[0] - cam_pos[0]
    rel_y = point_3d[1] - cam_pos[1]
    rel_z = point_3d[2] - cam_pos[2]

    x_proj = rel_x - rel_y * 0.7
    y_proj = -rel_z + (rel_x + rel_y) * 0.35

    screen_x = int(center_screen[0] + x_proj * fov)
    screen_y = int(center_screen[1] + y_proj * fov)
    return screen_x, screen_y


def reset_simu():
    global pos, pos_actuel, angle, angle_actuel, angle_ancien, t, drone_demarre
    global historique_roll, historique_pitch, historique_t
    pos = np.array([0.0, 0.0, 0.0])
    pos_actuel = np.copy(pos)
    angle = np.array([0.0, 0.0, 0.0])
    angle_actuel = np.copy(angle)
    angle_ancien = np.copy(angle)
    t = 0.0
    drone_demarre = False
    historique_roll.clear()
    historique_pitch.clear()
    historique_t.clear()

# ---------------------------------------------------------
# COMPOSANTS HUD (INTERFACES DU JEU)
# ---------------------------------------------------------
def draw_horizon_artificiel(surface, center, radius, roll, pitch):
    x_c, y_c = center
    pygame.draw.circle(surface, PANEL_BG, center, radius)
    pygame.draw.circle(surface, CYAN, center, radius, 2)

    pitch_offset = np.clip(pitch * 1.5, -radius + 5, radius - 5)
    angle_rad = math.radians(roll)

    dx = math.cos(angle_rad) * (radius - 10)
    dy = math.sin(angle_rad) * (radius - 10)

    p1 = (x_c - dx, y_c - dy + pitch_offset)
    p2 = (x_c + dx, y_c + dy + pitch_offset)

    pygame.draw.line(surface, GREEN, p1, p2, 2)

    pygame.draw.line(surface, RED, (x_c - 15, y_c), (x_c + 15, y_c), 2)
    pygame.draw.line(surface, RED, (x_c, y_c - 10), (x_c, y_c + 10), 2)


def draw_graph(surface, rect, hist_roll, hist_pitch):
    pygame.draw.rect(surface, PANEL_BG, rect, border_radius=8)
    pygame.draw.rect(surface, GRID_COLOR, rect, width=1, border_radius=8)

    title = FONT_M.render("ANGLE HISTOGRAM (°)", True, TEXT_COLOR)
    surface.blit(title, (rect.x + 10, rect.y + 10))

    if len(hist_roll) < 2:
        return

    pts_roll = []
    pts_pitch = []
    max_pts = min(len(hist_roll), 150)

    for i in range(max_pts):
        idx = -max_pts + i
        px = rect.x + 15 + (i / 150.0) * (rect.width - 30)
        
        py_r = rect.centery - (hist_roll[idx] / 45.0) * (rect.height / 2 - 20)
        py_p = rect.centery - (hist_pitch[idx] / 45.0) * (rect.height / 2 - 20)

        pts_roll.append((px, py_r))
        pts_pitch.append((px, py_p))

    pygame.draw.lines(surface, GREEN, False, pts_roll, 2)
    pygame.draw.lines(surface, CYAN, False, pts_pitch, 2)


# ---------------------------------------------------------
# BOUCLE PRINCIPALE
# ---------------------------------------------------------
running = True
btn_reset_rect = pygame.Rect(30, 640, 160, 45)

while running:
    # 1. GESTION DES ÉVÉNEMENTS & UI
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if btn_reset_rect.collidepoint(event.pos):
                reset_simu()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                drone_demarre = not drone_demarre
            elif event.key == pygame.K_r:
                reset_simu()

    # 2. COMMANDES MANETTE / CLAVIER
    stick_gx, stick_gy, stick_dx, stick_dy = 0.0, 0.0, 0.0, 0.0

    Poids_Moteur = Masse * g / (4 * np.cos(angle[0]) * np.cos(angle[1]))
    esc_Moteur = fct.calcul_force2commande(Poids_Moteur)
    esc_throttle = esc_Moteur

    if manette:
        if manette.get_button(0):  # Bouton A
            drone_demarre = True
        (stick_gx, stick_gy, stick_dx, stick_dy) = mxc.position_joystick_manette(manette)
        stick_dy = -stick_dy
    else:
        keys = pygame.key.get_pressed()
        if keys[pygame.K_z]: stick_dy = 0.8
        if keys[pygame.K_s]: stick_dy = -0.4
        if keys[pygame.K_q]: stick_gx = -0.5
        if keys[pygame.K_d]: stick_gx = 0.5

    # power = (stick_dy + 1.0) / 2.0 * 100 # en pourcentage

    POWER += stick_dy**3 * 10

    if abs(stick_dy) < 0.03:
        Stab_Alt = True
        POWER = 0
    else:
        Stab_Alt = False

    # 3. PHYSIQUE & CALCULS
    if drone_demarre:

        esc_com = fct.calcul_commande2controleur(POWER)

        esc_throttle = esc_com + esc_Moteur

        esc_throttle = max(1000, esc_throttle)
        esc_throttle = min(esc_throttle, 1800) #MAX power
        
        # f_m = fct.calcul_controleur2force(esc_com) #car on a fait des essais

        # f_m = fct.calculer_force_moteur_continue(intensite, Masse, angle_actuel[0], angle_actuel[1], g=9.81)

        # esc_com = 1000
        # cmd_alt = stick_dy
        # ALTITUDE +=  cmd_alt**3 * 0.02 #np.sign(cmd_alt)
        # ALTITUDE = max(0, ALTITUDE)
        # ALTITUDE = min(ALTITUDE, 15.0)

        cmd_angle_roll = stick_gx * 45 * np.pi / 180
        cmd_angle_pitch = -stick_gy * 45 * np.pi / 180

    else:
        cmd_angle_roll = 0.0
        cmd_angle_pitch = 0.0
        f_A_m, f_B_m, f_C_m, f_D_m = 0.0, 0.0, 0.0, 0.0

    vit = (pos - pos_actuel) / delta_t

    esc_corr_A, esc_corr_B, esc_corr_C, esc_corr_D = asserv.calcul_correction_PID(
        angle_actuel,
        vit[2],
        altitude_corr=Stab_Alt,
        angle_cible=np.array([cmd_angle_roll, cmd_angle_pitch, 0.0]),
        Kp_roll=kp_roll, Kp_pitch=kp_pitch, Kp_yaw=0.1, Kp_alt=kp_alt,
        Ki_roll=ki_roll, Ki_pitch=ki_pitch, Ki_yaw=0.005, Ki_alt=ki_alt,
        Kd_roll=kd_roll, Kd_pitch=kd_pitch, Kd_yaw=0.02, Kd_alt=kd_alt,
        delta_t=delta_t,
        reset_pid=not drone_demarre,
    )
    f_A = fct.calcul_controleur2force(esc_throttle + esc_corr_A)
    f_B = fct.calcul_controleur2force(esc_throttle + esc_corr_B)
    f_C = fct.calcul_controleur2force(esc_throttle + esc_corr_C)
    f_D = fct.calcul_controleur2force(esc_throttle + esc_corr_D)

    if drone_demarre:
        f_A = max(f_A, 0.0)
        f_B = max(f_B, 0.0)
        f_C = max(f_C, 0.0)
        f_D = max(f_D, 0.0)

        f_A = min(f_A, FORCE_MAX_MOTEUR)
        f_B = min(f_B, FORCE_MAX_MOTEUR)
        f_C = min(f_C, FORCE_MAX_MOTEUR)
        f_D = min(f_D, FORCE_MAX_MOTEUR)

        for nom_moteur, force_val in [("A", f_A), ("B", f_B), ("C", f_C), ("D", f_D)]:
            if force_val >= FORCE_MAX_MOTEUR:
                print(f"[ALERTE FORCE MAX] t={t:.2f}s | Moteur {nom_moteur} : {force_val:.3f} N >= {FORCE_MAX_MOTEUR} N")
    else:
        f_A = f_B = f_C = f_D = 0.0

    
    vitesse_lin_norme = np.linalg.norm(vit)
    
    # Force de traînée quadratique (F_frot = -k * ||v|| * v)
    F_frot = -k_frot * vitesse_lin_norme * vit

    poussee_totale = f_A + f_B + f_C + f_D
    R = fct.get_rotation_matrix(angle[0], angle[1], angle[2])

    Forces = np.array([
        np.dot(R, [0, 0, f_A]), np.dot(R, [0, 0, f_B]),
        np.dot(R, [0, 0, f_C]), np.dot(R, [0, 0, f_D]), Poids_global, F_frot
    ])
    Moments = fct.moments_moteur(
        np.array([0, 0, f_A]), np.array([0, 0, f_B]),
        np.array([0, 0, f_C]), np.array([0, 0, f_D]), L, l
    )

    pos_ancienne, pos_actuel = np.copy(pos_actuel), np.copy(pos)
    angle_ancien, angle_actuel = np.copy(angle_actuel), np.copy(angle)

    pos = dyn.position(pos_actuel, pos_ancienne, Forces, Masse, Delta_t=delta_t)
    angle = dyn.angle(angle_actuel, angle_ancien, Moments, Mat_Inertie, Delta_t=delta_t)

    if pos[2] <= 0:
        pos[2] = 0.0
        if poussee_totale < (Masse * g):
            pos[0], pos[1] = pos_actuel[0], pos_actuel[1]
            angle = np.copy(angle_actuel)

    # --- CALCUL DE LA VITESSE LINÉAIRE ---
    vitesse = (pos - pos_actuel) / delta_t
    norme_vitesse = np.linalg.norm(vitesse)

    # --- CALCUL DE LA VITESSE ANGULAIRE (°/s) ---
    vitesse_angulaire_rad = (angle - angle_actuel) / delta_t
    vitesse_angulaire_deg = np.degrees(vitesse_angulaire_rad)  # [w_roll, w_pitch, w_yaw] en °/s
    norme_vit_angulaire = np.linalg.norm(vitesse_angulaire_deg)

    # Vérification du dépassement de la limite (225°/s)
    alerte_vitesse_angulaire = np.any(np.abs(vitesse_angulaire_deg) > SEUIL_VIT_ANGULAIRE_MAX)

    if alerte_vitesse_angulaire:
        print("ALERTE Vit angulaire :", vitesse_angulaire_deg)

    roulis_deg, tangage_deg = np.degrees(angle[0]), np.degrees(angle[1])
    historique_roll.append(roulis_deg)
    historique_pitch.append(tangage_deg)
    t += delta_t

    # 4. RENDU GRAPHIQUE (PYGAME)
    SCREEN.fill(BG_COLOR)

    # --- VUE 3D (Centre de l'écran) ---
    cam_center = (640, 320)
    cam_target = pos

    grid_size = 50
    step = 2

    for x_g in range(-grid_size, grid_size + 1, step):
        p1 = project_3d_to_2d([x_g, -grid_size, 0], cam_target, cam_center)
        p2 = project_3d_to_2d([x_g, grid_size, 0], cam_target, cam_center)
        pygame.draw.line(SCREEN, GRID_COLOR, p1, p2, 1)

    for y_g in range(-grid_size, grid_size + 1, step):
        p1 = project_3d_to_2d([-grid_size, y_g, 0], cam_target, cam_center)
        p2 = project_3d_to_2d([grid_size, y_g, 0], cam_target, cam_center)
        pygame.draw.line(SCREEN, GRID_COLOR, p1, p2, 1)

    # Rendu du Drone 3D
    corners_local = [
        [-Longueur/2, -Largeur/2, -Hauteur /2], [Longueur/2, -Largeur/2, -Hauteur / 2],
        [Longueur/2, Largeur/2, -Hauteur / 2], [-Longueur/2, Largeur/2, -Hauteur / 2],
        [-Longueur/2, -Largeur/2, Hauteur / 2], [Longueur/2, -Largeur/2, Hauteur / 2],
        [Longueur/2, Largeur/2, Hauteur / 2], [-Longueur/2, Largeur/2, Hauteur / 2]
    ]

    corners_screen = []
    for c in corners_local:
        rotated = np.dot(R, c)
        world_pt = pos + rotated
        corners_screen.append(project_3d_to_2d(world_pt, cam_target, cam_center))

    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7)
    ]
    drone_color = RED if alerte_vitesse_angulaire else (CYAN if drone_demarre else ORANGE)
    for e in edges:
        pygame.draw.line(SCREEN, drone_color, corners_screen[e[0]], corners_screen[e[1]], 2)

    # --- PANNEAU DE BORD & HUD ---
    status_str = "MOTORS: ONLINE" if drone_demarre else "MOTORS: ARMED / PAUSED (Press A / Space)"
    status_color = GREEN if drone_demarre else RED
    SCREEN.blit(FONT_L.render(status_str, True, status_color), (30, 25))

    poussee_pct = (poussee_totale / (Masse * g)) * 100 if drone_demarre else 0.0
    telemetry_txt = f"ALT: {pos[2]:.2f}m | SPEED: {norme_vitesse:.2f}m/s | ROLL: {roulis_deg:+.1f}° | PITCH: {tangage_deg:+.1f}° | THRUST: {poussee_pct:.1f}%"
    SCREEN.blit(FONT_M.render(telemetry_txt, True, TEXT_COLOR), (30, 60))

    # --- BANDEAU D'ALERTE EN CAS DE DÉPASSEMENT DE VITESSE ANGULAIRE ---
    if alerte_vitesse_angulaire:
        alert_bg = pygame.Rect(30, 90, 780, 32)
        pygame.draw.rect(SCREEN, RED, alert_bg, border_radius=5)
        pygame.draw.rect(SCREEN, WHITE, alert_bg, width=2, border_radius=5)
        
        txt_alerte = FONT_M.render(
            f"⚠️ ALERTE : VITESSE ANGULAIRE CRITIQUE (> {SEUIL_VIT_ANGULAIRE_MAX}°/s) ! "
            f"[Roll: {vitesse_angulaire_deg[0]:.1f}°/s | Pitch: {vitesse_angulaire_deg[1]:.1f}°/s | Yaw: {vitesse_angulaire_deg[2]:.1f}°/s]",
            True, WHITE
        )
        SCREEN.blit(txt_alerte, (alert_bg.x + 10, alert_bg.y + 6))

    # Horizon Artificiel
    draw_horizon_artificiel(SCREEN, (100, 530), 65, roulis_deg, tangage_deg)
    SCREEN.blit(FONT_M.render("ATTITUDE", True, TEXT_COLOR), (65, 610))

    # Graphique en temps réel
    draw_graph(SCREEN, pygame.Rect(210, 460, 600, 220), historique_roll, historique_pitch)

    # ---------------------------------------------------------
    # PANNEAU DE DROITE : SYSTEM STATUS & FORCES MOTEURS
    # ---------------------------------------------------------
    panel_right = pygame.Rect(830, 20, 420, 680)
    pygame.draw.rect(SCREEN, PANEL_BG, panel_right, border_radius=10)
    pygame.draw.rect(SCREEN, GRID_COLOR, panel_right, width=1, border_radius=10)

    SCREEN.blit(FONT_L.render("SYSTEM STATUS", True, CYAN), (850, 40))
    
    color_vit_ang = RED if alerte_vitesse_angulaire else CYAN

    info_lines = [
        f"Sim Time: {t:.2f} s",
        f"Position X: {pos[0]:+.2f} m",
        f"Position Y: {pos[1]:+.2f} m",
        f"Position Z: {pos[2]:+.2f} m",
        "----------------------------------",
        f"Vitesse Vx: {vitesse[0]:+.2f} m/s",
        f"Vitesse Vy: {vitesse[1]:+.2f} m/s",
        f"Vitesse Vz: {vitesse[2]:+.2f} m/s",
        f"Vitesse ||V||: {norme_vitesse:.2f} m/s",
        "----------------------------------",
        f"Vit. Ang. Roll : {vitesse_angulaire_deg[0]:+.1f} °/s",
        f"Vit. Ang. Pitch: {vitesse_angulaire_deg[1]:+.1f} °/s",
        f"Vit. Ang. Yaw  : {vitesse_angulaire_deg[2]:+.1f} °/s",
        "----------------------------------",
        f"JOYSTICK INTENSITY : {POWER:.3f}",
        "----------------------------------",
        "MOTOR THRUSTS (N):",
        f"  Moteur A (f_A) : {f_A:.3f} N",
        f"  Moteur B (f_B) : {f_B:.3f} N",
        f"  Moteur C (f_C) : {f_C:.3f} N",
        f"  Moteur D (f_D) : {f_D:.3f} N",
        "----------------------------------",
        "PID CONTROLLER GAINS:",
        f"  Roll  : Kp={kp_roll:.4f} Ki={ki_roll:.4f} Kd={kd_roll:.4f}",
        f"  Pitch : Kp={kp_pitch:.4f} Ki={ki_pitch:.4f} Kd={kd_pitch:.4f}",
        "----------------------------------",
        "KEYBOARD CONTROLS:",
        "  [Space] : Start / Stop Motors",
        "  [Z/S]   : Pitch Up / Down",
        "  [Q/D]   : Roll Left / Right",
        "  [R]     : Reset Simulation",
    ]

    line_y_step = 20
    start_y_lines = 68

    for idx, line in enumerate(info_lines):
        if "Vit. Ang." in line:
            # Coloration spécifique en rouge si la vitesse angulaire dépasse 225°/s
            val_ang = abs(vitesse_angulaire_deg[idx - 10]) if 10 <= idx <= 12 else 0
            color = RED if val_ang > SEUIL_VIT_ANGULAIRE_MAX else CYAN
        elif "PID" in line or "CONTROLS" in line or "MOTOR" in line or "JOYSTICK" in line or "Vitesse" in line:
            color = CYAN
        else:
            color = TEXT_COLOR

        SCREEN.blit(FONT_S.render(line, True, color), (850, start_y_lines + idx * line_y_step))

    # --- BARRES VISUELLES DE POUSSÉE POUR CHAQUE MOTEUR ---
    moteurs_valeurs = [("A", f_A), ("B", f_B), ("C", f_C), ("D", f_D)]
    
    # Indice 17 dans info_lines correspond à '  Moteur A (f_A)' après l'ajout des 4 lignes de vitesse angulaire
    start_y_bars = start_y_lines + 17 * line_y_step + 2
    for idx, (nom, val) in enumerate(moteurs_valeurs):
        bar_x = 1070
        bar_y = start_y_bars + idx * line_y_step
        bar_w = 150
        bar_h = 12

        # Fond de la barre
        pygame.draw.rect(SCREEN, GRID_COLOR, (bar_x, bar_y, bar_w, bar_h), border_radius=3)
        
        # Remplissage de la barre
        fill_w = int(np.clip(val / FORCE_MAX_MOTEUR, 0.0, 1.0) * bar_w)
        if fill_w > 0:
            color_bar = RED if val >= FORCE_MAX_MOTEUR else (GREEN if drone_demarre else ORANGE)
            pygame.draw.rect(SCREEN, color_bar, (bar_x, bar_y, fill_w, bar_h), border_radius=3)

    # Bouton Reset Interactif
    pygame.draw.rect(SCREEN, RED, btn_reset_rect, border_radius=6)
    btn_txt = FONT_M.render("RESET (R)", True, WHITE)
    SCREEN.blit(btn_txt, (btn_reset_rect.x + 35, btn_reset_rect.y + 12))

    pygame.display.flip()
    CLOCK.tick(FPS)

pygame.quit()
sys.exit()