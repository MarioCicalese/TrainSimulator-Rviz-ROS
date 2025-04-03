#!/usr/bin/python3
import rospy
from uno.srv import trainDataService
from uno.msg import TrainData  # Importiamo il messaggio personalizzato

def get_train_data():
    rospy.wait_for_service('trainDataService')
    try:
        trainGetData = rospy.ServiceProxy('trainDataService', trainDataService)
        response = trainGetData()  # Chiamata al service
        return response.train_list  # Restituisce la lista di treni
    except rospy.ServiceException as e: 
        rospy.logerr("Errore nel chiamare il servizio: %s" % e)
        return []  # Se c'è un errore, restituisce una lista vuota

def dataCollector():
    rospy.init_node('dataCollector', anonymous=True)
    pub = rospy.Publisher("/train_data", TrainData, queue_size=10)  # Pubblica dati di tipo TrainData
    
    #Parametro pubblico, definisce la frequenza di aggionamento, valore di default = 5
    update_frequency = rospy.get_param("update_frequency", 5)
    rosrate = rospy.Rate(1/update_frequency)  # Rallentiamo il rate per non sovraccaricare il sistema
    rospy.loginfo("Nodo avviato, in attesa della richiesta al service (frequenza %d secondi)." % update_frequency)

    while not rospy.is_shutdown():
        train_list = get_train_data()  # Otteniamo la lista dei treni da pubblicare dal service

        if len(train_list) !=0:
            for train in train_list:  # Iteriamo sulla lista dei treni ricevuti
                pub.publish(train)  # Pubblica ogni treno singolarmente
                rospy.loginfo(f"Treno pubblicato: {train.departure_station} -> {train.arrival_destination}")
            rospy.loginfo(f"{len(train_list)} treni pubblicati sul topic /train_data")
        else:
            #Se la lista restituita dal service è vuota restituiamo un messaggio di default senza pubblicare sul topic
            rospy.loginfo("Non sono previsti più treni durante l'arco della giornata.")

        rosrate.sleep()  #Rispettiamo il rate

if __name__ == "__main__":
    dataCollector()
