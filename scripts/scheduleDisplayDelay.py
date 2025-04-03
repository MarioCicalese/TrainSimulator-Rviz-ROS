#!/usr/bin/python3
import rospy
from uno.msg import TrainData

def callback(data):
    """Funzione di callback chiamata automaticamente quando il subscriber legge dal topic"""
    
    #Formatto i dati in una singola riga con separatori
    train_info = (
        f"[{data.date_of_journey}] "
        f"{data.departure_station} -> {data.arrival_destination} | "
        f"{data.departure_time} -> {data.arrival_time} "
        f"({data.actual_arrival_time if data.journey_status == 'Delayed' else 'N/A'}) | "
        f" {data.journey_status} "
        f"{'' + data.reason_for_delay if data.reason_for_delay else ''}"
    )

    rospy.loginfo(train_info)

def schedule_display_delay():
    rospy.init_node('scheduleDisplayDelay', anonymous=True)
    rospy.Subscriber('/delay_alerts', TrainData, callback)

    rospy.loginfo("In ascolto sul topic /delay_alerts...")
    rospy.spin()

if __name__ == "__main__":
    schedule_display_delay()
