#!/usr/bin/python3
import rospy
import pandas as pd
from uno.srv import searchTrainService, searchTrainServiceResponse
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

def load_train_data(departure_station):
    """Carica i dati dei treni dal file CSV e li converte in messaggi TrainData."""
    try:
        df = pd.read_csv(CSV_FILE_PATH, sep=";", skipinitialspace=True, dtype=str)

        #filtro i treni in partenza da oggi
        #today_date
        #"31/01/2024"
        df = df[ (df['Date of Journey'] == "31/01/2024") & (df['Departure Time'] >= current_time)]
        
        #filtro i treni rispetto alla stazione di partenza (se la stringa in input al service è contenuta nella stazione di partenza). 
        df = df[ df['Departure Station'].str.contains(departure_station, case=False, na=False)]

        #ordino i treni rispetto all'orario di partenza
        df = df.sort_values(by = 'Departure Time', ascending=True)

        train_list = []

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
            
        return train_list
    
    except Exception as e:
        rospy.logerr(f"Errore nel caricamento dei dati del treno: {e}")
        return []

def handle_request(req):
    """Risponde alla richiesta restituendo tutti i dati dei treni disponibili."""
    departure_station = req.departure_station
    rospy.loginfo(f"Rispondendo alla richiesta con i dati dei treni che partono da {departure_station}.")

    train_data_list = load_train_data(departure_station)
    return searchTrainServiceResponse(train_data_list)

def searchTrainService_server():
    rospy.init_node('searchTrainService')
    rospy.Service('searchTrainService', searchTrainService, handle_request)
    rospy.loginfo("search Train Service avviato, in attesa di richieste...")
    rospy.spin()

if __name__ == "__main__":
    searchTrainService_server()