#!/usr/bin/python3
import rospy
from uno.msg import TrainData

def callback(data):
    # Formatto i dati in una singola riga con separatori
    train_info = (
        f"[{data.date_of_journey}] "
        f"{data.departure_station} -> {data.arrival_destination} | "
        f"{data.departure_time} -> {data.arrival_time} "
        f"({data.actual_arrival_time if data.journey_status == 'Delayed' else 'N/A'}) | "
        f" {data.journey_status} "
        f"{'' + data.reason_for_delay if data.reason_for_delay else ''}"
    )

    rospy.loginfo(train_info)

def schedule_display():
    rospy.init_node('scheduleDisplay', anonymous=True)
    rospy.Subscriber('/train_data', TrainData, callback)

    rospy.loginfo("In ascolto sul topic /train_data...")
    rospy.spin()

if __name__ == "__main__":
    schedule_display()
