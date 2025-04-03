#!/usr/bin/python3
import rospy
import pandas as pd
from uno.srv import trainDataService, trainDataServiceResponse
from uno.msg import TrainData
from datetime import datetime
import os

# Percorso del file CSV
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
CSV_FILE_PATH = os.path.join(parent_dir, "cleanRailwayDataset.csv")

#Ottengo la data attuale nel formato (DD/MM/YYYY)
today_date = datetime.today().strftime('%d/%m/%Y')

#Ottengo l'orario attuale nel formato HH:MM:SS
current_time = datetime.now().strftime("%H:%M:%S")

def load_train_data():
    """Carica i dati dei treni dal file CSV e li converte in una lista di tipo TrainData."""
    try:
        #Leggo il .csv considerando tutte le colonne come string
        df = pd.read_csv(CSV_FILE_PATH, sep=";", skipinitialspace=True, dtype=str)

        #filtro i treni considerando la giornata di oggi e l'orario attuale
        #"30/01/2024"
        #today_date
        df = df[ (df['Date of Journey'] == "30/01/2024") & (df['Departure Time'] >= current_time)]

        #ordino in maniera crescente i treni in base all'orario di partenza
        df = df.sort_values(by = 'Departure Time', ascending=True)

        train_list = []

        #itero sulle righe del dataset per creare la lista dei treni da restituire.
        for _, row in df.iterrows():
            train_msg = TrainData()

            train_msg.departure_station = row['Departure Station']
            train_msg.arrival_destination = row['Arrival Destination']
            train_msg.date_of_journey = row['Date of Journey']
            train_msg.departure_time = row['Departure Time']
            train_msg.arrival_time = row['Arrival Time']
            train_msg.actual_arrival_time = row['Actual Arrival Time'] if pd.notna(row['Actual Arrival Time']) else ""
            train_msg.journey_status = row['Journey Status']
            train_msg.reason_for_delay = row['Reason for Delay'] if pd.notna(row['Reason for Delay']) else ""
            
            train_list.append(train_msg)
        
        #restituisco la lista dei treni
        return train_list
    
    except Exception as e:
        rospy.logerr(f"Errore nel caricamento dei dati del treno: {e}")
        return []

def handle_request(req):
    """Risponde alla richiesta restituendo tutti i dati dei treni disponibili."""

    train_data_list = load_train_data()
    rospy.loginfo(f"Rispondo alla richiesta del client, {len(train_data_list)} treni restituiti. ")
    return trainDataServiceResponse(train_data_list)

def trainDataService_server():
    rospy.init_node('trainDataService')
    rospy.Service('trainDataService', trainDataService, handle_request)
    rospy.loginfo("Train Data Service avviato, in attesa di richieste...")
    rospy.spin()

if __name__ == "__main__":
    trainDataService_server()