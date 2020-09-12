# Info-2021_BeagleBone_Black

Le wiki donne des informations plus complètes sur la configuration des outils

## Répertoires

Le Git est constitué des répertoires suivants
### Cerveau
Contient sans surprise le cerveau du robot, on y retrouve le fichier *starter.py* qui permet de démarrer le cerveau. Il est aussi constitué des sous-répertoires suivants
#### node_manager
Contient des classes de bases pour la gestion des noeuds et leur communication via des events
#### nodes
Contient les différents noeuds qui composent le cerveau
#### events
Contient les différents events qui permettent aux noeuds de communiquer
#### interfaces
Contient les différentes interfaces externes du cerveau (can, GPIO)
### doc
Contient la documentation (pardon ? comment ça c'est vide ?)
### script
Contient des scripts et des fichiers de config pour le Debian de la BBB


## Programmation
Le code est conçu pour fonctionner sous Python 3.8
Le framework ROS était utilisé les années précédentes, ce qui changeait l'implémentation du fonctionnement du bus can et du robot.
L'objectif est de passer à du tout python, plus simple à prendre en main pour les nouveaux arrivants sur le projet (et accessoirement aussi pour ceux qui ne font pas d'études d'info), et donc plus facile de repartir du code existant d'une année sur l'autre.
