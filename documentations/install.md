# Sécurité

Une analyse est effectuée automatiquement **avant chaque commit** afin de détecter les mots de passe, tokens, clés API ou clés privées dans les fichiers modifiés.

## Installation

```bash
git clone git@github.com:Ixam3002/Drone-code.git
cd Drone-code

pip install pre-commit
pre-commit install --config .github/workflows/.pre-commit-config.yaml
```

## Utilisation

À chaque :

```bash
git commit
```

les fichiers ajoutés au commit sont analysés.

Si un secret est détecté, **le commit est bloqué** et le fichier concerné est indiqué.

Pour lancer manuellement l'analyse :

```bash
pre-commit run --all-files --config .github/workflows/.pre-commit-config.yaml
```
