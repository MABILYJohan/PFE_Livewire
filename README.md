# PFE_Livewire

Github clone link :
https://github.com/MABILYJohan/PFE_Livewire.git

Projet de segmentation 3D par la méthode du Livewire.

Google doc:	
	https://docs.google.com/document/d/1RVFxTeM-f6cEvDbJKubclZv_31WDmTzwdZu6j72Ab0I/edit?usp=sharing

Release_270320 :

Exécuter le programme avec les tests proposés sur le lapin :

- Compiler et Exécuter le projet.

- Charger le maillage PFE_Livewire/donneesPFE M2GIG/Bunny/bunnyLowPoly.obj

- Sélectionner un numéro de test (compris entre 1 et 5).

- Cliquer sur "Livewire"

Exécuter le programme sur un maillage avec un contour issu d'un nuage de points réel :
- Ouvrir le document PFE_Livewire/projetQt/mainwindow.cpp

- Commenter les lignes 124 et 125 pour mettre en silence les tests sur le lapin.

- Tester, toujours dans le même document, le Livewire sur :

> Myson :
- Décommenter les lignes 128 et 132
- Sauvegarder, compiler et exécuter le programme
- Charger le maillage basse résolution de Myson PFE_Livewire/donneesPFE M2GIG/Myson/Modele/model3D_LOW_triangles_ScreenedPoisson.obj
- Cliquer sur "Livewire"

> Siva :
- Décommenter les lignes 130 et 132
- Sauvegarder, compiler et exécuter le programme
- Charger le maillage basse résolution de Siva PFE_Livewire/donneesPFE M2GIG/Siva/modele/modele3D_LOW_Siva_meshBallPivoting.obj
- Cliquer sur "Livewire"