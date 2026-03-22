# design2_equipe11
Contient le code pour la simulation de la lame encasstrée et le code arduino

ce code arduino est concu pour fonctionner avec l'arduino mega



## main.cpp

Dans les premières lignes du code vous trouverez la section CMD_ENUM suivie de la variable mode. Pour basculer du mode PID vers le mode step simplement changer la variable mode à celui voulu et renvoyé le code sur le arduino.


## cerial.py


Pour lancer ce code faites `python3 cerial.py` puis suivez les instructions qui vous sont donner dans le terminal. Ce code fonctionne de la sorte une fois l'ordinateur connecter au arduino, celui-ci attend que vous tapiez p pour commencer à enregistrer sur format csv. Vous pouvez modifié le nombre d'échantillon en tappant `t`, vous pouvez changer le signal à enregistrer dans le menu `m` il vous est possible de choisir parmis : le courant la commande pid, l'erreur et la position.