#!/usr/bin/python3
import rospy
from uno.srv import addTrainService, addTrainServiceRequest
from uno.msg import TrainData
import os

#TRAIN_FILE_PATH = "/home/mario/Desktop/trainList.txt"
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
TRAIN_FILE_PATH = os.path.join(parent_dir, "trainList.txt")

def read_trains_from_file():
    trains = []
    try:
        with open(TRAIN_FILE_PATH, "r") as file:
            for line in file:
                parts = line.strip().split(";")
                if len(parts) != 8:
                    rospy.logwarn(f"Formato riga non valido: {line}")
                    continue

                train = TrainData()
                train.departure_station = parts[0]
                train.arrival_destination = parts[1]
                train.date_of_journey = parts[2]
                train.departure_time = parts[3]
                train.arrival_time = parts[4]
                train.actual_arrival_time = parts[5] if parts[5] else ""
                train.journey_status = parts[6]
                train.reason_for_delay = parts[7] if parts[7] else ""

                trains.append(train)

    except Exception as e:
        rospy.logerr(f"Errore nella lettura del file: {e}")

    return trains

def add_train_client():
    rospy.init_node("add_train_client")
    rospy.loginfo("Nodo avviato, in attesa della richiesta al service")

    rospy.wait_for_service("add_train_service")
    try:
        add_train = rospy.ServiceProxy("add_train_service", addTrainService)
        trains = read_trains_from_file()
        
        if not trains:
            rospy.logwarn("Nessun treno da aggiungere.")
            return

        req = addTrainServiceRequest()
        req.trains = trains
        response = add_train(req)

        if response.success:
            rospy.loginfo("%d Treni aggiunti con successo al dataset." % len(trains))
        else:
            rospy.logerr("Errore durante l'aggiunta dei treni.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Errore nel chiamare il service: {e}")

if __name__ == "__main__":
    add_train_client()
