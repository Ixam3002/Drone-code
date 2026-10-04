import serial
import pygame
from pygame.locals import DOUBLEBUF, OPENGL, QUIT
from OpenGL.GL import *
from OpenGL.GLU import *
import math


# =========================
# Configuration
# =========================
PORT = "COM5"
BAUDRATE = 19200

roll = 0.0
pitch = 0.0
yaw = 0.0


# =========================
# Cube 3D
# =========================
def draw_cube():
    vertices = [
        (-193, -20, -100),
        (193, -20, -100),
        (193,  20, -100),
        (-193,  20, -100),

        (-193, -20, 100),
        (193, -20, 100),
        (193,  20, 100),
        (-193, 20, 100)
    ]

    faces = [
        (0, 1, 2, 3),  # arrière
        (4, 5, 6, 7),  # avant
        (0, 1, 5, 4),  # bas
        (2, 3, 7, 6),  # haut
        (1, 2, 6, 5),  # droite
        (0, 3, 7, 4)   # gauche
    ]

    glBegin(GL_QUADS)

    # Une couleur différente pour chaque face
    colors = [
        (0.0, 0.30, 0.60),
        (0.0, 0.40, 0.75),
        (0.0, 0.25, 0.50),
        (0.0, 0.35, 0.65),
        (0.0, 0.45, 0.80),
        (0.0, 0.20, 0.45)
    ]

    for i, face in enumerate(faces):
        glColor3f(*colors[i])

        for vertex in face:
            glVertex3fv(vertices[vertex])

    glEnd()


# =========================
# Texte
# =========================
def draw_text(text, x, y):
    font = pygame.font.SysFont("Arial", 25, True)
    surface = font.render(text, True, (0, 0, 0))

    text_data = pygame.image.tostring(surface, "RGBA", True)
    width, height = surface.get_size()

    glWindowPos2d(x, y)

    glDrawPixels(
        width,
        height,
        GL_RGBA,
        GL_UNSIGNED_BYTE,
        text_data
    )


# =========================
# Programme principal
# =========================
def main():

    global roll, pitch, yaw

    # Connexion Arduino
    try:
        serial_port = serial.Serial(PORT, BAUDRATE, timeout=0.01)
        print(f"Connexion à {PORT} à {BAUDRATE} bauds")
    except Exception as e:
        print("Impossible d'ouvrir le port série :")
        print(e)
        return

    # Initialisation Pygame
    pygame.init()

    screen = pygame.display.set_mode(
        (1280, 720),
        DOUBLEBUF | OPENGL
    )

    pygame.display.set_caption("Arduino MPU6050 - 3D Visualization")

    # Configuration OpenGL
    glEnable(GL_DEPTH_TEST)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()

    gluPerspective(
        45,
        1280 / 720,
        0.1,
        5000
    )

    glMatrixMode(GL_MODELVIEW)

    clock = pygame.time.Clock()

    running = True

    while running:

        # =========================
        # Gestion des événements
        # =========================
        for event in pygame.event.get():

            if event.type == QUIT:
                running = False

        # =========================
        # Lecture du port série
        # =========================
        try:
            while serial_port.in_waiting:

                line = serial_port.readline().decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

                if not line:
                    continue

                # Arduino envoie :
                # roll/pitch/yaw
                items = line.split("/")

                if len(items) >= 3:

                    try:
                        roll = float(items[0])
                        pitch = float(items[1])
                        yaw = float(items[2])

                    except ValueError:
                        pass

        except Exception:
            pass

        # =========================
        # Affichage
        # =========================
        glClear(
            GL_COLOR_BUFFER_BIT |
            GL_DEPTH_BUFFER_BIT
        )

        glLoadIdentity()

        # Position de la caméra
        glTranslatef(0, 0, -800)

        # =========================
        # Rotation du cube
        # =========================

        # Equivalent Processing :
        #
        # rotateX(radians(-pitch));
        # rotateZ(radians(roll));
        # rotateY(radians(yaw));

        glRotatef(-pitch, 1, 0, 0)
        glRotatef(roll, 0, 0, 1)
        glRotatef(yaw, 0, 1, 0)

        # Cube
        draw_cube()

        # =========================
        # Affichage des angles
        # =========================

        # On sauvegarde les matrices
        glPushMatrix()

        # Désactive la profondeur pour le texte
        glDisable(GL_DEPTH_TEST)

        # Retour en coordonnées écran
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()

        glOrtho(
            0,
            1280,
            0,
            720,
            -1,
            1
        )

        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        # Texte
        draw_text(
            f"Roll: {int(roll)}    Pitch: {int(pitch)}    Yaw: {int(yaw)}",
            30,
            670
        )

        glPopMatrix()

        glMatrixMode(GL_PROJECTION)
        glPopMatrix()

        glMatrixMode(GL_MODELVIEW)

        glEnable(GL_DEPTH_TEST)

        glPopMatrix()

        # Mise à jour écran
        pygame.display.flip()

        # ~60 FPS
        clock.tick(60)

    # Fermeture
    serial_port.close()
    pygame.quit()


if __name__ == "__main__":
    main()
