#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import csv
import datetime

sub_amostragem = 10 # so sera registrada no arquivo uma a cada N mensagens

now = datetime.datetime.now()
filename = 'src/time_series_csv/joint_states_{}.csv'.format(now.strftime('%Y%m%d_%H%M%S'))  # cria um nome unico para cada nova execucao no diretorio especificado

count = 0
def joint_state_callback(data):
    global count
    count += 1
    
    if count % sub_amostragem == 0:
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.velocity)

        # cria o cabecalho do csv
        if count == 0:
            with open(filename, mode='a') as csv_file:
                writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerow(['time', 'pos_1', 'pos_2', 'pos_3', 'pos_4', 'pos_5', 'pos_6', 'vel_1', 'vel_2', 'vel_3', 'vel_4', 'vel_5', 'vel_6', 'eff_1', 'eff_2', 'eff_3', 'eff_4', 'eff_5', 'eff_6'])

        # Abre o arquivo CSV e adiciona uma nova linha com os dados atuais
        with open(filename, mode='a') as csv_file:
            writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow([data.header.stamp.to_sec()] + list(data.position) + list(data.velocity) + list(data.effort))
    
def joint_state_subscriber():
    rospy.init_node('joint_state_listener', anonymous=True)

    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        joint_state_subscriber()
    except rospy.ROSInterruptException:
        pass