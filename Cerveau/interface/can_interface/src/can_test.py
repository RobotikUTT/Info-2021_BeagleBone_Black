# !/usr/bin/python3
import can
import json
# import cantools

""" script de test des librairies can et cantools pour l'écriture, l'envoi et la lecture de données sur le bus can
essayer ici de tester les messages et frames du dossier data (fichiers json) """
""" Le point de blocage actuel est la traduction des données json vers les datas de frames can que l'on doit envoyer 
au microcontroleur pour l'asservissement est déjà programmé donc il faut qu'il reçoive des trames qu'il puisse interprété """
""" Il faut trouver à partir du code 2018 comment sont converties les données xml vers les trames can et faire de même """

def main():
    tx_bus = can.interface.Bus()

    print("préparation de la frame")
    message = can.Message(
        arbitration_id=1,
        data=[0, 25, 0, 1, 3, 1, 4, 1],
        is_extended_id=False,
    )

    print(message)

    try:
        print("envoi de la frame")
        tx_bus.send(message)
        print("Message envoyé {}".format(tx_bus.channel_info))

    except can.CanError:
        print("message non envoyé")

    # lecture des messages reçus
    # rx_bus = can.interface.Bus()
    # for msg in rx_bus:
        # print("{}: {}".format(msg.arbitration_id, msg.data))

    # récupération des données des json
    # db = cantools.db.Database('cerveau/interface/can_interface/data/frames.json')
    # db.messages()
    # extract = db.get_message_by_name()
    # data = db.encode_message()

    # parsing du fichier de frames
    with open('cerveau/interface/can_interface/data/frames.json', 'r') as fichier:
        frames = json.load(fichier)


if __name__ == '__main__':
    main()
