#!/usr/bin/python3
import rospy
import pandas as pd
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from datetime import datetime
import os

# Percorso del file CSV
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
CSV_FILE_PATH = os.path.join(parent_dir, "cleanRailwayDataset.csv")

def load_train_data():
    """Carica i dati dei treni dal file CSV e li converte in una lista di tipo TrainData."""

    #Ottengo la data attuale nel formato (DD/MM/YYYY)
    today_date = datetime.today().strftime('%d/%m/%Y')

    #Ottengo l'orario attuale nel formato HH:MM:SS
    current_time = datetime.now().strftime("%H:%M:%S")

    try:
        #Leggo il .csv considerando tutte le colonne come string
        df = pd.read_csv(CSV_FILE_PATH, sep=",", skipinitialspace=True, dtype=str)

        #filtro i treni considerando quelli in movimento in questo momento (considerando data e orario attuale)
        #"30/01/2024"
        #today_date
        df = df[
            (df['Date of Journey'] == today_date) & 
            (df['Departure Time'] <= current_time) & 
            ((df['Actual Arrival Time'] >= current_time) | (df['Arrival Time'] >= current_time))
        ]
        #rospy.loginfo(df)
        
        #ordino in maniera crescente i treni in base all'orario di partenza
        df = df.sort_values(by = 'Departure Time', ascending=True)
        
        nrows = df.shape[0]
        if nrows <= 3:
            df_sliced = df.head(nrows)
        else:
            #se sono presenti piu di 3 treni, considero solo i primi 3
            df_sliced = df.head(3)

        #Posizione delle stazioni di partenza e di arrivo
        x_departure = [-5,0,5]
        y_departure = [-5,-5,-5]
        x_arrival = [-5,0,5]
        y_arrival = [5,5,5]

        #aggiungo gli attributi relative alle posizioni delle stazioni se non sono presenti.
        if(len(df_sliced.columns) == 8):
            df_sliced.insert(len(df_sliced.columns),"x_departure", x_departure[:df.shape[0]])
            df_sliced.insert(len(df_sliced.columns),"y_departure", y_departure[:df.shape[0]])
            df_sliced.insert(len(df_sliced.columns),"x_arrival", x_arrival[:df.shape[0]])
            df_sliced.insert(len(df_sliced.columns),"y_arrival", y_arrival[:df.shape[0]])
        
        #Salvo il dataset contenente solo i treni da mostrare sul simulatore
        df_sliced.to_csv(os.path.join(parent_dir, "trainSimulated.csv"), sep = ";", index=False)

        return df_sliced
    
    except Exception as e:
        rospy.logerr(f"Errore nel caricamento dei dati del treno: {e}")
        return []

def get_stations(df_sliced):
    stations = {
        "departure": [],
        "arrival": []
    }

    for _, row in df_sliced.iterrows():
        departure_station = row['Departure Station']
        arrival_station = row['Arrival Destination']

        #memorizzo tutte le stazioni di partenza
        stations["departure"].append(departure_station)
        
        #memorizzo tutte le stazioni di arrivo
        stations["arrival"].append(arrival_station)
    
    return stations


def create_station_marker(name, x, y, marker_id):
    # Crea il marker per la stazione
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "stations"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    #Posizione del cubo
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0  

    # Dimensione del cubo
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    #Se l'id è pari stiamo considerando una stazione di partenza
    if(marker_id % 2 == 0):
        marker.color.r = 0.0
        marker.color.b = 1.0 #La imposto di colore blu
    else:
        marker.color.r = 1.0 #Imposto le stazioni di arrivo di colore rosso
        marker.color.b = 0.0
        
    marker.color.g = 0.0
    marker.color.a = 0.6

    return marker

def clear_all_markers(pub):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "trains"
    marker.action = Marker.DELETEALL  # Elimino tutti i marker del namespace "trains"

    pub.publish(marker)

def station_publisher():
    rospy.init_node('station_publisher', anonymous=True)
    pub = rospy.Publisher('/stations', Marker, queue_size=10)
    rate = rospy.Rate(1)

    # Posizioni delle stazioni (x,y)
    stat_positions = [
        (-5, -5),
        (0, -5),
        (5, -5),
    ]

    while not rospy.is_shutdown():
        #Ottengo i treni in movimento da mostrare sul simulatore
        df_sliced = load_train_data()

        #Ottengo le stazioni di partenza e di destinazione dei treni
        stations = get_stations(df_sliced)

        departure_stations = stations["departure"]
        arrival_stations = stations["arrival"]
        marker_id = 0

        #Elimino eventuali marker visibili nella scena
        clear_all_markers(pub)
        for i in range(len(departure_stations)):
            position = stat_positions[i]
            
            # Pubblica stazione di partenza
            station_marker = create_station_marker(departure_stations[i], position[0], position[1] - 0.5, marker_id)
            pub.publish(station_marker)
            marker_id += 1

            # Pubblica stazione di arrivo
            station_marker = create_station_marker(arrival_stations[i], position[0], position[1] + 10.5, marker_id)
            pub.publish(station_marker)
            marker_id += 1

        rate.sleep()

if __name__ == "__main__":
    station_publisher()