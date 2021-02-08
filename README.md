# Projet TR 2020-2021

Yakumo Kunimoto - Adrien Piednoel

## Figure imposée:

Pour executer le figure imposée il faut charger le module qui gère l'encodeur moteur:
```shell
cd module
make
sudo insmod kirq.ko
```

ensuite le main se trouve dans impose :

```shell
cd ../impose
make
sudo ./main
```

La vidéo de présentation se nomme "imposee.mp4"

## Figure libre:

Toutes les information d'installation et d'utilisation de la figure libre sont disponible dans le readme du dossier ros2/ 

La vidéo de présentation se nomme "libre.mp4"

### Tests unitaires:

Le dossier unitTestC contient tous les tests unitaires effectuées en début de projet. la compilations de tous les test se fait avec un simple "make".