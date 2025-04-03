#!/usr/bin/python3
import rospy
from std_msgs.msg import String
from uno.srv import delayAnalysisService
from uno.msg import TrainData

def get_delayed_trains():
    """Chiama il servizio delayAnalysisService e ottiene la lista dei treni in ritardo/cancellati."""
    rospy.wait_for_service('delayAnalysisService')
    try:
        service_proxy = rospy.ServiceProxy('delayAnalysisService', delayAnalysisService)
        response = service_proxy()
        return response.train_list  # Lista di TrainData
    except rospy.ServiceException as e:
        rospy.logerr(f"Errore nel chiamare il servizio: {e}")
        return []

def delay_analysis_client():
    """Nodo che interroga il service e pubblica i risultati sul topic /delay_alerts."""
    rospy.init_node('delayAnalysisClient', anonymous=True)
    pub = rospy.Publisher('/delay_alerts', TrainData, queue_size=10)
    
    update_frequency = rospy.get_param("update_frequency", 5)
    rate = rospy.Rate(1/update_frequency)  # Rallentiamo il rate per non sovraccaricare il sistema
    
    while not rospy.is_shutdown():
        delayed_trains = get_delayed_trains()

        if delayed_trains:
            for train in delayed_trains:
                pub.publish(train)
                rospy.loginfo(f"Pubblicato treno in ritardo/cancellato: {train.departure_station} -> {train.arrival_destination}")
            rospy.loginfo("%d treni pubblicati sul topic /delay_alerts" % len(delayed_trains))
        else:
            rospy.loginfo("Nessun treno in ritardo o cancellato trovato.")

        rate.sleep()

if __name__ == "__main__":
    delay_analysis_client()
