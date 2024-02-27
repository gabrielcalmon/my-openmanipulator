#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import csv
import datetime
import tf
import geometry_msgs.msg

sub_amostragem = 100  # so sera registrada no arquivo uma a cada N mensagens
# por padrao /joint_states opera a 990~1000hz
# float_precision = 50

now = datetime.datetime.now()  # recebe o horario atual
filename = '/home/calmon/ws-manipulator/src/my-openmanipulator/_time_series_csv/joint_states_{}.csv'.format(now.strftime('%Y%m%d_%H%M%S'))  # cria um nome unico para cada nova execucao no diretorio especificado

count = 0

# Inicializa o nó ROS
rospy.init_node('joint_state_listener', anonymous=True)

# Cria o ouvinte de transformações
tf_listener = tf.TransformListener()

# Espera para garantir que as transformações estejam disponíveis
rospy.sleep(1.0)

# substitui o marcador decimal ponto por virgula
def localize_floats(row):
    return [str(el).replace('.', ',') for el in row]

def joint_state_callback(data):
    global count
    count += 1

    # realiza a subamostragem
    if count % sub_amostragem == 0:
        try:
            # Obtém a transformação de "end_link" para "world"
            tf_listener.waitForTransform("world", "end_link", rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = tf_listener.lookupTransform("world", "end_link", rospy.Time())

            # Adiciona informações de transformação ao arquivo CSV
            with open(filename, mode='a') as csv_file:
                writer = csv.writer(csv_file, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)

                # cria o cabeçalho do CSV CASO o arquivo esteja vazio
                if csv_file.tell() == 0:
                    writer.writerow(['time', 'pos_1', 'pos_2', 'pos_3', 'pos_4', 'pos_5', 'pos_6', 'vel_1', 'vel_2', 'vel_3', 'vel_4', 'vel_5', 'vel_6', 'eff_1', 'eff_2', 'eff_3', 'eff_4', 'eff_5', 'eff_6', 'endeff_x', 'endeff_y', 'endeff_z'])
                
                # concatena os dados da mensagem e da transformação numa lista
                formattedList = [data.header.stamp.to_sec()] + list(data.position) + list(data.velocity) + list(data.effort) + [trans[0], trans[1], trans[2]]
                writer.writerow(localize_floats(formattedList))
                print(f'Registering data ...')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Não foi possível obter a transformação.")

def joint_state_subscriber():
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    # spin() simply keeps Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        joint_state_subscriber()
    except rospy.ROSInterruptException:
        pass
