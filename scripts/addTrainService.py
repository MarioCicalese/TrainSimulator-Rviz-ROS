#!/usr/bin/python3
import rospy
import pandas as pd
from uno.srv import addTrainService, addTrainServiceResponse
from uno.msg import TrainData
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
CSV_FILE_PATH = os.path.join(parent_dir, "cleanRailwayDataset.csv")

def handle_add_train(req):
    try:
        # Converti la lista di TrainData in un DataFrame
        new_trains = [{
            "Departure Station": train.departure_station,
            "Arrival Destination": train.arrival_destination,
            "Date of Journey": train.date_of_journey,
            "Departure Time": train.departure_time,
            "Arrival Time": train.arrival_time,
            "Actual Arrival Time": train.actual_arrival_time,
            "Journey Status": train.journey_status,
            "Reason for Delay": train.reason_for_delay
        } for train in req.trains]

        df_new = pd.DataFrame(new_trains)

        # Carica il dataset esistente e aggiungi i nuovi treni
        df_existing = pd.read_csv(CSV_FILE_PATH, sep=";", dtype=str)
        df_final = pd.concat([df_existing, df_new], ignore_index=True)

        # Salva il dataset aggiornato
        df_final.to_csv(CSV_FILE_PATH, sep=";", index=False)

        rospy.loginfo("%d Nuovi treni aggiunti con successo." % len(req.trains))
        return addTrainServiceResponse(True)

    except Exception as e:
        rospy.logerr(f"Errore nell'aggiunta dei treni: {e}")
        return addTrainServiceResponse(False)

def add_train_service():
    rospy.init_node("add_train_service")
    rospy.Service("add_train_service", addTrainService, handle_add_train)
    rospy.loginfo("Service AddTrainService avviato, in attesa di richieste...")
    rospy.spin()

if __name__ == "__main__":
    add_train_service()
