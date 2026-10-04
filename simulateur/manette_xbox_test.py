import sys
import pygame

# Initialisation de Pygame et du module Joystick
pygame.init()
pygame.joystick.init()

# Vérification de la présence d'une manette
if pygame.joystick.get_count() == 0:
    print("Aucune manette détectée ! Vérifie la connexion.")
    sys.exit()

# Sélection de la première manette détectée
manette = pygame.joystick.Joystick(0)
manette.init()

print(f"Manette connectée : {manette.get_name()}")
print(f"Nombre d'axes : {manette.get_numaxes()}")
print(f"Nombre de boutons : {manette.get_numbuttons()}")

# Boucle principale de lecture
try:
    while True:
        # Pygame a besoin de pomper les événements système pour mettre à jour l'état des entrées
        pygame.event.pump()

        # 1. Lecture des axes (Sticks analogiques et Gâchettes LT/RT)
        # Valeurs entre -1.0 et 1.0 (ou 0.0 et 1.0 selon la gâchette)
        stick_gauche_x = manette.get_axis(0)
        stick_gauche_y = manette.get_axis(1)
        gachette_lt = manette.get_axis(4)  # Selon la version du driver
        stick_droite_x = manette.get_axis(2)
        stick_droite_y = manette.get_axis(3)
        gachette_rt = manette.get_axis(5)

        # 2. Lecture des boutons (0 = relâché, 1 = appuyé)
        bouton_A = manette.get_button(0)
        bouton_B = manette.get_button(1)
        bouton_X = manette.get_button(2)
        bouton_Y = manette.get_button(3)


        # Affichage rapide si le bouton A est pressé ou le stick déplacé
        if (
            bouton_A
            or abs(stick_gauche_x) > 0.1
            or abs(stick_gauche_y) > 0.1
            or abs(stick_droite_x) > 0.1
            or abs(stick_droite_y) > 0.1
        ):
            print(
                f"Stick G: ({stick_gauche_x:.2f}, {stick_gauche_y:.2f}) | Bouton A: {bouton_A}"
                f" | Stick D: ({stick_droite_x:.2f}, {stick_droite_y:.2f})"
            )

        pygame.time.wait(1)  # Pause de 20 ms pour ne pas surcharger le CPU

except KeyboardInterrupt:
    print("\nArrêt du programme.")
    pygame.quit()