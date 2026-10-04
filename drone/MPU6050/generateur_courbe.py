
import serial
import matplotlib.pyplot as plt
from collections import deque

# ==================================================
# CONFIGURATION
# ==================================================

PORT = "COM5"          # <-- À MODIFIER
BAUDRATE = 115200

X_MAX = 1800
NB_VALEURS = 1800

# ==================================================
# CONNEXION À ARDUINO
# ==================================================

try:
    arduino = serial.Serial(PORT, BAUDRATE, timeout=0.05)
    print("Arduino connecté sur", PORT)
except serial.SerialException as e:
    print("Impossible de se connecter à", PORT)
    print(e)
    exit()

# Petit délai pour laisser Arduino redémarrer
arduino.reset_input_buffer()

# ==================================================
# DONNÉES
# ==================================================

valeurs = deque(maxlen=NB_VALEURS)
positions = deque(maxlen=NB_VALEURS)

compteur = 0

# ==================================================
# CRÉATION DU GRAPHIQUE
# ==================================================

plt.ion()

fig, ax = plt.subplots(figsize=(10, 5))

# IMPORTANT :
# on fixe l'axe AVANT de recevoir les données
ax.set_xlim(0, X_MAX)
ax.set_ylim(0, 1023)       # À adapter si nécessaire

ax.set_xlabel("Échantillon")
ax.set_ylabel("Valeur")
ax.set_title("Arduino - mesure en temps réel")

ax.grid(True)

ligne, = ax.plot(
    [],
    [],
    linewidth=1.5
)

# Affichage initial
fig.canvas.draw()
fig.canvas.flush_events()

# ==================================================
# BOUCLE
# ==================================================

try:

    while plt.fignum_exists(fig.number):

        # Lire toutes les données disponibles
       # while arduino.in_waiting:
        texte = arduino.readline()

        print(texte)
        if not texte:
            continue

        try:
            valeur = float(texte)
        except ValueError:
            continue

        # Ajouter la donnée
        valeurs.append(valeur)
        positions.append(compteur)

        compteur += 1

        # Mise à jour de la courbe
        ligne.set_data(
            list(positions),
            list(valeurs)
        )

        # X TOUJOURS FIXE
        ax.set_xlim(0, X_MAX)

        # Ajustement Y uniquement si on a des données
        if len(valeurs) > 1:

            minimum = min(valeurs)
            maximum = max(valeurs)

            if minimum == maximum:
                minimum -= 1
                maximum += 1

            marge = (maximum - minimum) * 0.10

            ax.set_ylim(
                minimum - marge,
                maximum + marge
            )

        # Rafraîchissement
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        plt.pause(0.01)

except KeyboardInterrupt:

    print("\nArrêt du programme.")

finally:

    arduino.close()
    plt.close("all")

    print("Port série fermé.")
