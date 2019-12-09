# Move

Réalisé par *Xavier Jannin*.

Ce projet a pour but de découvrir le contrôle d'un bras robotique `Scara` de manière analytique puis grâce à un outil appelé `MoveIt!`. Il est alors possible de déplacer un bras dans un simulateur mais aussi sur un robot réel.


L'ensemble de ce dossier contient le compte-rendu des différentes parties du module `Move` sur le robot `Scara` :
- [Partie 1]('./PART_1.md') : Introduction du robot
- [Partie 2]('./PART_2.md') : `MoveIt!`
- [Partie 3]('./PART_3.md') : Lien entre `Gazebo` et `MoveIt!`
- [Partie 4]('./PART_4.md') : Avec le vrai robot


(Tous les codes se trouvent dans le dossier `programmes/`)

---
**Aide pour facilement voir les images dans le compte-rendu :**

Utiliser [`Visual Studio Code`](https://code.visualstudio.com/) pour directement visualiser les fichiers `markdown` avec les images :
- Cliquer sur l'icône en haut à droite de la fenêtre qui ressemble à un livre ouvert avec une loupe en bas à droite
- Ou, exécuter le raccourci : `CTRL+K` puis appuyer sur la lettre `V`.


Ou sinon, lire depuis les *PDFs* générés à partir des fichiers *Markdown* grâce à l'extension `Markdown PDF` de `VS Code` (mais les liens vers les fichiers ne fonctionnent pas).


---
## Installation :

Avant de commencer, il faut compiler et sourcer les fichiers, depuis le dossier `programmes` :
```sh
$ catkin_make && source devel/setup.bash`
```

### Erreurs possibles :

Si des erreurs apparaissent lors de la compilation :
- Vérifier que le fichier `CMakeLists.txt` dans le dossier `src` est un lien symbolique. Sinon utiliser la commande, depuis le dossier `programmes/src` :`
```sh
$ ln -s /opt/ros/indigo/share/catkin/cmake/toplevel.cmake CMakeLists.txt
```
- Vérifier que tous les fichiers `Python` sont exécutables. Ou alors exécuter la commande, depuis le dossier `programmes/src` :
```sh
$ find . -type f -name "*.py" | xargs chmod u+x
```

