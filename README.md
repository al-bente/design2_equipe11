# design2_equipe11
Contient le code pour la simulation de la lame encasstrée et le code arduino

Ce code de arduino est concu pour fonctionner sur un arduinno UNO à noter qu'en ce moment la précision de la commande n'est que de 8 bit pour accomoder les UNO mais le mega est capable de faire du 10 bit.


Les pin A0 sont utiliser pour l'acquisition et D6 pour la commande n'oublié pas de grounder le arduino


## main.cpp


Vers la fin du code dans la section debug il y a une commande qui envoie du data par la connection usb vous pouvez en changer le contenue. Sur la photo, `pid représente la commande, mais si vous voulez imprimer la pose ou l'erreur simplement le changer dans le code et le pusher vers le arduino:

<img width="620" height="186" alt="2026-02-15-141546_620x186_scrot" src="https://github.com/user-attachments/assets/422aedbb-d17d-48f4-8b7a-de3819432b6f" />


## Cerial.py

Pour rouler ce code sur Windows aller dans les premières lignes du code commenter la partie PORT=/dev/ttyACM0 et décommenter la partie COM3 ensuite ajuster le chiffre après COM a celui qui correspond a votre arduino:

<img width="338" height="45" alt="2026-02-15-141117_338x45_scrot" src="https://github.com/user-attachments/assets/6009ab2d-ab1b-44f1-abf1-a2208f44326f" />


Ensuite faites cette commande dans le terminal `python3 cerial.py ` 

à chaque fois que vous aller appuyer sur p et enter dans le terminal une nouvelle courbe sera imprimer avec les dernières valeures publier depuis la dernière courbe
