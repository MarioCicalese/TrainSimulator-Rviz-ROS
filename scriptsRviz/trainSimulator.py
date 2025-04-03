#!/usr/bin/python3
import rospy
import pandas as pd
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from datetime import datetime
import os

# Percorso del file CSV aggiornato
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
CSV_FILE_PATH = os.path.join(parent_dir, "trainSimulated.csv")

def load_train_data():
    """Carica i dati aggiornati dal CSV contenente le coordinate delle stazioni."""
    try:
        df = pd.read_csv(CSV_FILE_PATH, sep=";", skipinitialspace=True, dtype=str)
        return df

    except Exception as e:
        rospy.logerr(f"Errore nel caricamento dei dati dei treni: {e}")
        return pd.DataFrame()

def calculate_position(x_dep, y_dep, x_arr, y_arr, dep_time, arr_time):
    """
    Calcola la posizione attuale del treno in base al tempo trascorso tra partenza e arrivo.
    Il treno si muove in modo lineare tra la partenza e l'arrivo.
    """
    now = datetime.now().strftime("%H:%M:%S")

    # Converto gli orari in datetime
    fmt = "%H:%M:%S"
    dep_time = datetime.strptime(dep_time, fmt)
    arr_time = datetime.strptime(arr_time, fmt)
    now_time = datetime.strptime(now, fmt)

    # Se il treno non è ancora partito, rimane alla stazione di partenza
    if now_time < dep_time:
        return x_dep, y_dep

    # Se il treno è già arrivato, rimane alla stazione di arrivo
    if now_time >= arr_time:
        return x_arr, y_arr

    # Calcoliamo la percentuale di tempo trascorso rispetto al viaggio totale (Progesso)
    total_duration = (arr_time - dep_time).total_seconds()
    elapsed_time = (now_time - dep_time).total_seconds()
    progress = elapsed_time / total_duration  # Percentuale del viaggio completato

    # Interpolazione lineare tra partenza e arrivo
    x_current = x_dep + progress * (x_arr - x_dep)
    y_current = y_dep + progress * (y_arr - y_dep)

    return x_current, y_current

def create_train_marker(train_id, x, y, status):
    """Crea un marker per il treno con colore in base allo stato."""
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "trains"
    marker.id = train_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    # Posizione del treno
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    # Dimensioni del treno
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3

    # Assegna il colore in base allo stato del treno
    if status.lower() == "cancelled":
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif status.lower() == "delayed":
        marker.color.r = 1.0
        marker.color.g = 1.0 
        marker.color.b = 0.0
    else:  # On time
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

    marker.color.a = 1.0  # Opacità piena

    return marker

def create_train_timer_marker(train_id, x, y,arrival_time):
    """Mostra il tempo rimanente sopra il treno"""

    arrival_time = datetime.strptime(arrival_time, "%H:%M:%S")
    now_time = datetime.strptime(datetime.now().strftime("%H:%M:%S"),"%H:%M:%S")
    time_remaining = (arrival_time - now_time).total_seconds()

    minutes = int(time_remaining // 60)
    seconds = int(time_remaining % 60)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "train_timers"
    marker.id = train_id + 1000

    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 1.5  #Sopra il treno

    marker.scale.z = 0.5  #Dimensione del testo

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    marker.text = f"{minutes:02}:{seconds:02}"
    return marker

def clear_all_markers(pub):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "trains"
    marker.action = Marker.DELETEALL  # Elimino tutti i marker del namespace "trains"

    pub.publish(marker)

def train_publisher():
    """Nodo ROS per pubblicare la posizione dei treni in tempo reale."""
    rospy.init_node('train_publisher', anonymous=True)
    pub = rospy.Publisher('/trains', Marker, queue_size=10)
    rate = rospy.Rate(1)  # Aggiornamento 1 volta al secondo

    while not rospy.is_shutdown():
        #Ottengo il dataframe dei treni da simulare
        df_trains = load_train_data()
        if df_trains.empty:
            rate.sleep()
            continue

        #Prima di pubblicare elimino eventuali marker presenti nella scena
        clear_all_markers(pub)
        for index, row in df_trains.iterrows():
            train_id = index
           
            status = row["Journey Status"]

            x_dep, y_dep = float(row["x_departure"]), float(row["y_departure"])
            x_arr, y_arr = float(row["x_arrival"]), float(row["y_arrival"])

            # Determino l'orario di arrivo corretto in base allo stato del treno
            arr_time = row["Arrival Time"]
            if status.lower() == "delayed":
                arr_time = row["Actual Arrival Time"]

            #Se il treno è cancellato deve rimanere fermo nella stazione di partenza
            if status.lower() != "cancelled":
                x, y = calculate_position(x_dep, y_dep, x_arr, y_arr, row["Departure Time"], arr_time)

                # Creo il marker del treno
                train_marker = create_train_marker(train_id, x, y, status)
            else:
                #Se il treno non è cancellato si deve spostare verso la stazione di arrivo
                train_marker = create_train_marker(train_id, x_dep, y_dep, status)
            
            #Pubblico il marker che rappresenta il treno
            pub.publish(train_marker)

            #Se il treno non è stato cancellato pubblico un timer che indica il tempo rimanete per raggiungere la stazione di arrivo
            if status.lower() != "cancelled":
                timer_marker = create_train_timer_marker(train_id,x,y,arr_time)
                pub.publish(timer_marker)

        rate.sleep()

if __name__ == "__main__":
    train_publisher()