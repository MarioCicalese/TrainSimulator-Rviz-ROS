#!/usr/bin/python3
import rospy
import sys
from uno.srv import searchTrainService, searchTrainServiceRequest
from uno.msg import TrainData

def search_train_client(departure_station):
    rospy.wait_for_service('searchTrainService')

    try:
        # Creazione del client del servizio
        search_train = rospy.ServiceProxy('searchTrainService', searchTrainService)
        
        # Creazione della richiesta e assegnazione del valore
        request = searchTrainServiceRequest()
        request.departure_station = departure_station  # Passa il parametro in input
        
        # Chiamata al servizio
        response = search_train(request)

        # Controllo se ci sono treni disponibili
        if not response.train_list:
            rospy.loginfo(f"Nessun treno trovato in partenza da {departure_station} per il resto della giornata.")
            sys.exit(1)
        else:
            rospy.loginfo(f"Treni in partenza da {departure_station} durante la giornata:")
            for train in response.train_list:
                rospy.loginfo(f"[{train.date_of_journey}] {train.departure_time} -> {train.arrival_time} | {departure_station} -> {train.arrival_destination} | {train.journey_status}")
            sys.exit(1)

    except rospy.ServiceException as e:
        rospy.logerr(f"Errore durante la chiamata al servizio: {e}")

if __name__ == "__main__":
    # Inizializzazione del nodo
    rospy.init_node('searchTrainClient')

    # Controlla che l'argomento sia stato passato
    if len(sys.argv) < 2:
        rospy.logerr("Stazione di partenza come parametro mancante.")
        sys.exit(1)

    # Prendi la stazione di partenza dalla riga di comando
    departure_station = sys.argv[1]
    
    # Chiamata al service con la stazione specificata
    search_train_client(departure_station)
