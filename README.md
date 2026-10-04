# Drone DIY

Projet de conception et de réalisation d'un **drone stabilisé de A à Z**.

Le projet comprend la conception électronique, la communication radio, la récupération des données de capteurs, la stabilisation ainsi que la conception de la structure en impression 3D.

## Architecture

### Télécommande

Les commandes sont récupérées depuis une **manette Xbox Series S** à l'aide d'un **ESP32-S3**, qui décode le protocole GIP.

Les données sont ensuite transmises en UART à un **Arduino Micro**, qui communique avec le drone via un **nRF24L01+**.

```text
Xbox Series S
      │
      ▼
   ESP32-S3
      │ UART
      ▼
 Arduino Micro
      │
      │ nRF24L01+
      ▼
     Radio
```

> La raison du choix de cette architecture, plutôt que de connecter directement le nRF24L01+ à l'ESP32-S3, sera détaillée dans la documentation du projet.

### Drone

L'électronique embarquée repose sur :

| Composant | Fonction |
|---|---|
| Arduino Micro | Contrôle du drone |
| MPU6050 | Accéléromètre + gyroscope |
| BMP180 | Mesure de l'altitude |
| nRF24L01+ | Communication radio |

## Simulation

Avant les essais réels, un **simulateur de stabilité** a été développé afin de tester les algorithmes de stabilisation et le comportement du drone.

## Conception

La structure du drone est conçue et fabriquée en **impression 3D**.

Les modèles 3D, le code, le simulateur et la documentation sont disponibles dans ce dépôt.

## État du projet

Le projet est actuellement en développement.

Les principaux objectifs sont :

- [ ] Communication radio
- [ ] Lecture des capteurs
- [ ] Stabilisation
- [ ] Finalisation de la structure 3D
- [ ] Assemblage
- [ ] Premiers essais réels

## Documentation

Les détails techniques du projet seront progressivement ajoutés dans le dossier [`documentation/`](documentation/).

## Contributeurs

- **Ixam3002**
- **Thomas127**
