import ast


def lire_configuration(chemin_fichier, par_section=True):
  """Lit un fichier de paramètres et retourne un dictionnaire dynamique.

  :param chemin_fichier: chemin vers le fichier .txt ou .py
  :param par_section: True pour organiser par sections (#SECTION), False pour un
  dict plat
  :return: dictionnaire contenant l'ensemble des variables
  """
  with open(chemin_fichier, "r", encoding="utf-8") as f:
    contenu = f.read()

  # 1. Repérage des sections à partir des commentaires textuels
  lignes = contenu.splitlines()
  sections = []
  for idx, ligne in enumerate(lignes, start=1):
    l = ligne.strip()
    # Détection des lignes type #DONNEE DRONE (en ignorant les bandes de #)
    if l.startswith("#") and set(l) != {"#"}:
      sections.append((idx, l.lstrip("#").strip()))

  # 2. Analyse syntaxique sécurisée (AST)
  arbre = ast.parse(contenu)
  donnees = {}

  for node in arbre.body:
    if isinstance(node, ast.Assign):
      cle = node.targets[0].id
      valeur = ast.literal_eval(node.value)

      if par_section:
        # Trouver la section active pour la ligne de la variable
        section = "GENERAL"
        for num_ligne, nom_sec in sections:
          if num_ligne <= node.lineno:
            section = nom_sec

        if section not in donnees:
          donnees[section] = {}
        donnees[section][cle] = valeur
      else:
        donnees[cle] = valeur

  return donnees

if __name__ == "__main__":

    chemin_fichier = "Donnees_drone.txt"
    dic = lire_configuration(chemin_fichier, par_section=True)
    print(dic)
    print(dic["DONNEE DRONE"]["Matrice_Inertie"])