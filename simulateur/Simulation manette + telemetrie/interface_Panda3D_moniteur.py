"""
interface_Panda3D_2.py
----------------------
Interface graphique 3D basée sur Panda3D.
Lance le moniteur de télémétrie Tkinter dans une fenêtre séparée.
"""

import threading
import numpy as np
from panda3d.core import (
    Point3, Geom, GeomTriangles, GeomVertexData, GeomVertexFormat,
    GeomVertexWriter, GeomNode, LineSegs, TextNode, LColor,
    DirectionalLight, AmbientLight, WindowProperties
)
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenText import OnscreenText
from direct.gui.DirectButton import DirectButton

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
print(str(Path(__file__).resolve().parent))

# Imports du projet
from simulation_physique import DroneSimulation, SEUIL_VIT_ANGULAIRE_MAX
import manette_xbox_commande as mxc
from telemetry_tkinder import TelemetryWindow

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
        self.simu.SEUIL_VIT_ANGULAIRE_MAX = SEUIL_VIT_ANGULAIRE_MAX

        # Configuration de la fenêtre Panda3D
        props = WindowProperties()
        props.setTitle("Drone Flight Simulator — Panda3D 3D View")
        props.setSize(1280, 720)
        self.win.requestProperties(props)
        self.setBackgroundColor(18/255.0, 22/255.0, 28/255.0)
        self.disableMouse()

        # Gestion des entrées (Manette / Clavier)
        self.setup_inputs()

        # Scène 3D & Modèle STL
        self.setup_lights()
        self.setup_grid()
        self.drone_node = self.load_drone_model(stl_file)

        # HUD minimaliste sur le rendu 3D
        self.setup_ui()

        # Démarrage de la 2ème fenêtre (Tkinter) sur un thread séparé
        self.start_telemetry_window()

        # Tâche principale de mise à jour à chaque frame
        self.taskMgr.add(self.update, "UpdateTask")

    def start_telemetry_window(self):
        def run_tkinter():
            telemetry_app = TelemetryWindow(self.simu)
            telemetry_app.start()

        t = threading.Thread(target=run_tkinter, daemon=True)
        t.start()

    def setup_inputs(self):
        import pygame
        pygame.init()
        pygame.joystick.init()
        self.manette = None
        if pygame.joystick.get_count() > 0:
            self.manette = pygame.joystick.Joystick(0)
            self.manette.init()

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
        if not HAS_STL_LIB:
            return self.create_fallback_drone()
        try:
            stl_data = mesh.Mesh.from_file(stl_filename)
        except Exception:
            return self.create_fallback_drone()

        vdata = GeomVertexData('stl_drone', GeomVertexFormat.getV3n3(), Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, 'vertex')
        normal = GeomVertexWriter(vdata, 'normal')
        prim = GeomTriangles(Geom.UHStatic)

        SCALE_MM_TO_M = 0.001

        for i in range(len(stl_data.vectors)):
            n = stl_data.normals[i]
            tri = stl_data.vectors[i]
            for v in tri:
                vertex.addData3f(v[0] * SCALE_MM_TO_M, v[1] * SCALE_MM_TO_M, v[2] * SCALE_MM_TO_M)
                normal.addData3f(n[0], n[1], n[2])
            prim.addVertices(i * 3, i * 3 + 1, i * 3 + 2)

        geom = Geom(vdata)
        geom.addPrimitive(prim)

        node = GeomNode('drone_stl_node')
        node.addGeom(geom)

        drone_path = self.render.attachNewNode(node)
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
        self.btn_reset = DirectButton(
            text="RESET (R)", scale=0.05, pos=(-1.1, 0, -0.85),
            command=self.simu.reset, frameColor=(0.9, 0.2, 0.2, 1), text_fg=(1, 1, 1, 1)
        )

    def update(self, task):
        import pygame

        # 1. Entrées
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

        # 2. Pas de simulation physique
        self.simu.step(stick_gx=stick_gx, stick_gy=stick_gy, stick_dy=stick_dy)

        # 3. Rendu Modèle 3D
        self.drone_node.setPos(-self.simu.pos[1], -self.simu.pos[0], self.simu.pos[2])
        self.drone_node.setHpr(
            np.degrees(self.simu.angle[2]),
            np.degrees(self.simu.angle[1]),
            np.degrees(self.simu.angle[0])
        )

        # 4. Caméra de Suivi 3D
        cam_dist = 5.0
        cam_x = -self.simu.pos[1] - cam_dist * np.sin(self.simu.angle[2])
        cam_y = -self.simu.pos[0] - cam_dist * np.cos(self.simu.angle[2])
        cam_z = self.simu.pos[2] + 2.0
        self.camera.setPos(cam_x, cam_y, cam_z)
        self.camera.lookAt(Point3(-self.simu.pos[1], -self.simu.pos[0], self.simu.pos[2]))

        # 5. Indication d'état sur la vue 3D
        if self.simu.drone_demarre:
            self.text_status.setText("MOTORS: ONLINE")
            self.text_status.setFg((0.2, 0.9, 0.4, 1))
        else:
            self.text_status.setText("MOTORS: ARMED / PAUSED")
            self.text_status.setFg((0.9, 0.3, 0.3, 1))

        return task.cont


if __name__ == "__main__":
    app = DroneAppPanda3D("drone.stl")
    app.run()