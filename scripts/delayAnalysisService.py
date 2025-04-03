#!/usr/bin/python3
import rospy
import pandas as pd
from uno.srv import delayAnalysisService, delayAnalysisServiceResponse
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
    """Carica e filtra i dati dei treni in base al ritardo e alle cancellazioni."""
    try:
        # Caricamento del dataset
        df = pd.read_csv(CSV_FILE_PATH, sep=";", dtype=str)

        #filtro rispetto la data e l'orario attuale
        #today_date
        #"31/01/2024"
        df = df[ (df['Date of Journey'] == "31/01/2024") & (df['Departure Time'] >= current_time)]

        # Recupera il valore del parametro pubblico delay_threshold (in minuti)
        delay_threshold = rospy.get_param("/delay_threshold", 5)  # Default: 5 minuti

        # Converti l'orario di arrivo previsto e reale in minuti per calcolare il ritardo
        df['Arrival Time'] = pd.to_datetime(df['Arrival Time'], format="%H:%M:%S", errors="coerce")
        df['Actual Arrival Time'] = pd.to_datetime(df['Actual Arrival Time'], format="%H:%M:%S", errors="coerce")
        
        # Calcolo del ritardo in minuti (se il treno non è stato cancellato)
        df["Delay (minutes)"] = (df["Actual Arrival Time"] - df["Arrival Time"]).dt.total_seconds() / 60

        # Filtriamo i treni cancellati o con un ritardo maggiore o uguale a delay_threshold
        df_filtered = df[(df["Journey Status"] == "Cancelled") | (df["Delay (minutes)"] >= delay_threshold)]

        #ordino i treni in base all'orario di partenza
        df_filtered = df_filtered.sort_values(by = 'Departure Time', ascending=True)
        
        # Creiamo la lista di TrainData
        train_list = []
        for _, row in df_filtered.iterrows():
            train_msg = TrainData()
            train_msg.departure_station = row["Departure Station"]
            train_msg.arrival_destination = row["Arrival Destination"]
            train_msg.date_of_journey = row["Date of Journey"]
            train_msg.departure_time = row["Departure Time"]
            train_msg.arrival_time = row["Arrival Time"].strftime("%H:%M:%S") if pd.notna(row["Arrival Time"]) else "N/A"
            train_msg.actual_arrival_time = row["Actual Arrival Time"].strftime("%H:%M:%S") if pd.notna(row["Actual Arrival Time"]) else "N/A"
            train_msg.journey_status = row["Journey Status"]
            train_msg.reason_for_delay = row["Reason for Delay"] if pd.notna(row["Reason for Delay"]) else ""

            train_list.append(train_msg)

        return train_list

    except Exception as e:
        rospy.logerr(f"Errore nel caricamento dei dati del treno: {e}")
        return []

def handle_request(req):
    """Gestisce la richiesta al service e restituisce i dati dei treni filtrati."""

    rospy.loginfo("Rispondendo alla richiesta con i treni cancellati o in ritardo.")
    train_list = load_train_data()
    return delayAnalysisServiceResponse(train_list)

def delay_analysis_service():
    """Inizializza il nodo del service ROS."""

    rospy.init_node('delayAnalysisService')
    rospy.Service('delayAnalysisService', delayAnalysisService, handle_request)
    rospy.loginfo("Delay Analysis Service è in esecuzione, in attesa di richieste...")
    rospy.spin()

if __name__ == "__main__":
    delay_analysis_service()
