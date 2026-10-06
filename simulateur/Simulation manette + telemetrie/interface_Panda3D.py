"""
drone_interface_panda3d.py
--------------------------
Interface graphique 3D basée sur Panda3D.
Importe 'DroneSimulation' de drone_core pour piloter l'état du drone et afficher le STL.
"""

import numpy as np
from panda3d.core import (
    Point3, Geom, GeomTriangles, GeomVertexData, GeomVertexFormat,
    GeomVertexWriter, GeomNode, LineSegs, TextNode, LColor,
    DirectionalLight, AmbientLight
)
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenText import OnscreenText
from direct.gui.DirectButton import DirectButton

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
print(str(Path(__file__).resolve().parent))

from simulation_physique import DroneSimulation, SEUIL_VIT_ANGULAIRE_MAX
import manette_xbox_commande as mxc

# Import de la bibliothèque STL si présente
try:
    from stl import mesh
    HAS_STL_LIB = True
except ImportError:
    HAS_STL_LIB = False


class DroneAppPanda3D(ShowBase):
    def __init__(self, stl_file="drone.stl"):
        super().__init__()

        # Instanciation du cœur de simulation physique
        self.simu = DroneSimulation()

        # Configuration de la fenêtre Panda3D
        self.win.requestProperties(self.get_window_properties())
        self.setBackgroundColor(18/255.0, 22/255.0, 28/255.0)
        self.disableMouse()

        # Gestion des entrées (Manette / Clavier)
        self.setup_inputs()

        # Scène 3D & Modèle STL
        self.setup_lights()
        self.setup_grid()
        self.drone_node = self.load_drone_model(stl_file)

        # Interface Télémétrie (HUD)
        self.setup_ui()

        # Tâche principale de mise à jour à chaque frame
        self.taskMgr.add(self.update, "UpdateTask")

    def get_window_properties(self):
        from panda3d.core import WindowProperties
        props = WindowProperties()
        props.setTitle("Drone Flight Simulator — Panda3D + Core Library")
        props.setSize(1280, 720)
        return props

    def setup_inputs(self):
        import pygame
        pygame.init()
        pygame.joystick.init()
        self.manette = None
        if pygame.joystick.get_count() > 0:
            self.manette = pygame.joystick.Joystick(0)
            self.manette.init()
            print(f"Manette connectée : {self.manette.get_name()}")
        else:
            print("Aucune manette détectée : Utilisation du Clavier (Z/Q/S/D/Espace/R).")

        self.keys = {"z": False, "q": False, "s": False, "d": False}
        self.accept("z", self.update_key, ["z", True])
        self.accept("z-up", self.update_key, ["z", False])
        self.accept("s", self.update_key, ["s", True])
        self.accept("s-up", self.update_key, ["s", False])
        self.accept("q", self.update_key, ["q", True])
        self.accept("q-up", self.update_key, ["q", False])
        self.accept("d", self.update_key, ["d", True])
        self.accept("d-up", self.update_key, ["d", False])
        self.accept("space", self.simu.toggle_motors)
        self.accept("r", self.simu.reset)

    def update_key(self, key, state):
        self.keys[key] = state

    def setup_lights(self):
        dlight = DirectionalLight("dlight")
        dlight.setColor(LColor(0.9, 0.9, 0.9, 1))
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setHpr(45, -45, 0)
        self.render.setLight(dlnp)

        alight = AmbientLight("alight")
        alight.setColor(LColor(0.3, 0.3, 0.3, 1))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)

    def setup_grid(self):
        lines = LineSegs()
        lines.setColor(40/255.0, 55/255.0, 70/255.0, 1.0)
        lines.setThickness(1.0)
        grid_size = 30
        step = 2

        for i in range(-grid_size, grid_size + 1, step):
            lines.moveTo(i, -grid_size, 0)
            lines.drawTo(i, grid_size, 0)
            lines.moveTo(-grid_size, i, 0)
            lines.drawTo(grid_size, i, 0)

        grid_node = lines.create()
        self.render.attachNewNode(grid_node)

    def load_drone_model(self, stl_filename):
        try:
            stl_data = mesh.Mesh.from_file(stl_filename)
        except Exception as e:
            print(f"Erreur lors du chargement du STL : {e}")
            # Modèle de secours par défaut si le fichier est introuvable
            cube = self.loader.loadModel("models/box")
            cube.setScale(0.2, 0.2, 0.05)
            cube.setColor(0, 0, 0, 1) # Noir
            return cube

        vdata = GeomVertexData('stl_drone', GeomVertexFormat.getV3n3(), Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, 'vertex')
        normal = GeomVertexWriter(vdata, 'normal')

        prim = GeomTriangles(Geom.UHStatic)

        # Conversion millimètres -> mètres (division par 1000)
        SCALE_MM_TO_M = 0.001

        for i in range(len(stl_data.vectors)):
            n = stl_data.normals[i]
            tri = stl_data.vectors[i]
            
            for v in tri:
                # Application du facteur 1/1000 sur X, Y, Z
                vertex.addData3f(v[0] * SCALE_MM_TO_M, v[1] * SCALE_MM_TO_M, v[2] * SCALE_MM_TO_M)
                normal.addData3f(n[0], n[1], n[2])

            prim.addVertices(i * 3, i * 3 + 1, i * 3 + 2)

        geom = Geom(vdata)
        geom.addPrimitive(prim)

        node = GeomNode('drone_stl_node')
        node.addGeom(geom)

        drone_path = self.render.attachNewNode(node)
        
        # 🎨 Appliquer la couleur NOIRE (R=0, G=0, B=0, Alpha=1)
        drone_path.setColor(0.0, 0.0, 0.0, 1.0)
        
        return drone_path

    def create_fallback_drone(self):
        cube = self.loader.loadModel("models/box")
        cube.setScale(0.2, 0.2, 0.05)
        cube.setColor(0.0, 0.0, 0.0, 1.0)
        return cube

    def setup_ui(self):
        self.text_status = OnscreenText(
            text="", pos=(-1.25, 0.88), scale=0.06, fg=(0.2, 0.9, 0.4, 1), align=TextNode.ALeft
        )
        self.text_telemetry = OnscreenText(
            text="", pos=(-1.25, 0.80), scale=0.045, fg=(0.9, 0.9, 0.9, 1), align=TextNode.ALeft
        )
        self.text_alert = OnscreenText(
            text="", pos=(-1.25, 0.72), scale=0.045, fg=(1, 0.3, 0.3, 1), align=TextNode.ALeft
        )
        self.text_panel = OnscreenText(
            text="", pos=(0.45, 0.88), scale=0.033, fg=(0.85, 0.9, 0.95, 1), align=TextNode.ALeft, mayChange=True
        )

        self.btn_reset = DirectButton(
            text="RESET (R)", scale=0.05, pos=(-1.1, 0, -0.85),
            command=self.simu.reset, frameColor=(0.9, 0.2, 0.2, 1), text_fg=(1, 1, 1, 1)
        )

    def update(self, task):
        import pygame

        # 1. Lecture des entrées utilisateur
        stick_gx, stick_gy, stick_dy = 0.0, 0.0, 0.0

        if self.manette:
            pygame.event.pump()
            if self.manette.get_button(0):
                self.simu.drone_demarre = True
            (stick_gx, stick_gy, _, stick_dy_raw) = mxc.position_joystick_manette(self.manette)
            stick_dy = -stick_dy_raw
        else:
            if self.keys["z"]: stick_dy = 0.8
            if self.keys["s"]: stick_dy = -0.4
            if self.keys["q"]: stick_gx = -0.5
            if self.keys["d"]: stick_gx = 0.5

        # 2. Pas de calcul dans le module de simulation
        self.simu.step(stick_gx=stick_gx, stick_gy=stick_gy, stick_dy=stick_dy)

        # 3. Mise à jour de la position du maillage 3D
        self.drone_node.setPos(-self.simu.pos[1], -self.simu.pos[0], self.simu.pos[2])
        self.drone_node.setHpr(
            np.degrees(self.simu.angle[2]),  # Yaw
            np.degrees(self.simu.angle[1]),  # Pitch
            np.degrees(self.simu.angle[0])   # Roll
        )

        # 4. Caméra de suivi 3D
        cam_dist = 5.0
        cam_x = -self.simu.pos[1] - cam_dist * np.sin(self.simu.angle[2])
        cam_y = -self.simu.pos[0] - cam_dist * np.cos(self.simu.angle[2])
        cam_z = self.simu.pos[2] + 2.0
        self.camera.setPos(cam_x, cam_y, cam_z)
        self.camera.lookAt(Point3(-self.simu.pos[1], -self.simu.pos[0], self.simu.pos[2]))

        # 5. Mise à jour des textes HUD
        if self.simu.drone_demarre:
            self.text_status.setText("MOTORS: ONLINE")
            self.text_status.setFg((0.2, 0.9, 0.4, 1))
        else:
            self.text_status.setText("MOTORS: ARMED / PAUSED (Press A / Space)")
            self.text_status.setFg((0.9, 0.3, 0.3, 1))

        poussee_pct = (self.simu.poussee_totale / (self.simu.Masse * self.simu.g)) * 100 if self.simu.drone_demarre else 0.0
        self.text_telemetry.setText(
            f"ALT: {self.simu.pos[2]:.2f}m | SPEED: {self.simu.norme_vitesse:.2f}m/s | "
            f"ROLL: {np.degrees(self.simu.angle[0]):+.1f}° | PITCH: {np.degrees(self.simu.angle[1]):+.1f}° | "
            f"THRUST: {poussee_pct:.1f}%"
        )

        if self.simu.alerte_vitesse_angulaire:
            self.text_alert.setText(f"⚠️ ALERTE : VITESSE ANGULAIRE CRITIQUE (> {SEUIL_VIT_ANGULAIRE_MAX}°/s) !")
        else:
            self.text_alert.setText("")

        self.text_panel.setText(
            f"--- SYSTEM STATUS ---\n"
            f"Sim Time: {self.simu.t:.2f} s\n"
            f"Pos (X, Y, Z): {self.simu.pos[0]:+.2f}, {self.simu.pos[1]:+.2f}, {self.simu.pos[2]:+.2f} m\n"
            f"Speed ||V||: {self.simu.norme_vitesse:.2f} m/s\n"
            f"Ang. Speed Roll : {self.simu.vitesse_angulaire_deg[0]:+.1f} °/s\n"
            f"Ang. Speed Pitch: {self.simu.vitesse_angulaire_deg[1]:+.1f} °/s\n"
            f"Ang. Speed Yaw  : {self.simu.vitesse_angulaire_deg[2]:+.1f} °/s\n"
            f"JOYSTICK INTENSITY: {self.simu.POWER:.3f}\n"
            f"MOTOR THRUSTS (N):\n"
            f"  A: {self.simu.f_A:.3f} N | B: {self.simu.f_B:.3f} N\n"
            f"  C: {self.simu.f_C:.3f} N | D: {self.simu.f_D:.3f} N\n"
            f"---------------------\n"
            f"CONTROLS:\n"
            f"  [Space] : Start / Stop Motors\n"
            f"  [Z/S]   : Pitch Up / Down\n"
            f"  [Q/D]   : Roll Left / Right\n"
            f"  [R]     : Reset Simulation\n"
        )

        return task.cont


if __name__ == "__main__":
    app = DroneAppPanda3D("drone.stl")
    app.run()