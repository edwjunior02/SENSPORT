import serial
import json
import time
import matplotlib.pyplot as plt
import os
from datetime import datetime

# Configuración del puerto serie
ser = serial.Serial('/dev/cu.usbmodem141201', 9600, timeout=1)

# Variables para almacenar las muestras
deviceID = 0
acel_data = []
giro_data = []
magnet_data = []
orient_data = []
timestamp_data = []

# Tiempo de muestreo en segundos
sampling_time = 10

# Tiempo inicial
start_time = time.time()

while (time.time() - start_time) < sampling_time:
    # Lectura de los datos del puerto serie
    line = ser.readline().decode().strip()
    values = []
    
    # Extracción de los datos de la línea
    deviceID = line.split(';')[0]
    for data_sensor in line.split(';')[1:5]:
        values.append([float(x) for x in data_sensor.split(',')])
    
    # Almacenamiento de los datos
    acel_data.append({'ax': values[0][0], 'ay': values[0][1], 'az': values[0][2]})
    timestamp_data.append({'timestamp': time.time() - start_time})
    giro_data.append({'gx': values[1][0], 'gy': values[1][1], 'gz': values[1][2]})
    magnet_data.append({'mx': values[2][0], 'my': values[2][1], 'mz': values[2][2]})
    orient_data.append({'yaw': values[3][0], 'pitch': values[3][1], 'roll': values[3][2]})
    
    # Espera de un tiempo para el siguiente muestreo
    time.sleep(0.01)

# Creación del diccionario completo
data_dict = {'Accelerometer': acel_data, 
             'Gyroscope': giro_data,
             'Magnetometer': magnet_data,
             'Orientation': orient_data,
             'Timestamp': timestamp_data}

# Creación del archivo JSON
with open('get_data.json', 'w') as outfile:
    json.dump(data_dict, outfile)


####################VISUALIZACIÓN DE LA CAPTURA#################################
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(10, 8))
fig.suptitle('Evaluación del golpe a través de la IMU')
ax1.set_title('Datos del acelerómetro')
ax2.set_title('Datos del giroscopio')
ax3.set_title('Datos del magnetómetro')
ax4.set_title('Datos de orientación')

# Obtener listas de valores para cada eje del acelerómetro y para los timestamps
ax_values = [d['ax'] for d in data_dict['Accelerometer']]
ay_values = [d['ay'] for d in data_dict['Accelerometer']]
az_values = [d['az'] for d in data_dict['Accelerometer']]
gx_values = [d['gx'] for d in data_dict['Gyroscope']]
gy_values = [d['gy'] for d in data_dict['Gyroscope']]
gz_values = [d['gz'] for d in data_dict['Gyroscope']]
mx_values = [d['mx'] for d in data_dict['Magnetometer']]
my_values = [d['my'] for d in data_dict['Magnetometer']]
mz_values = [d['mz'] for d in data_dict['Magnetometer']]
roll = [d['roll'] for d in data_dict['Orientation']]
pitch = [d['pitch'] for d in data_dict['Orientation']]
yaw = [d['yaw'] for d in data_dict['Orientation']]
timestamps = [d['timestamp'] for d in data_dict['Timestamp']]

ax1.plot(timestamps, ax_values, label='Eje X Acelerómetro')
ax1.plot(timestamps, ay_values, label='Eje y Acelerómetro')
ax1.plot(timestamps, az_values, label='Eje z Acelerómetro')
ax1.set_xlabel('Tiempo (s)')
ax1.set_ylabel('Aceleración (g = 9.8 m/s2)')
ax1.legend()

ax2.plot(timestamps, gx_values, label='Eje x Giroscopio')
ax2.plot(timestamps, gy_values, label='Eje y Giroscopio')
ax2.plot(timestamps, gz_values, label='Eje z Giroscopio')
ax2.set_xlabel('Tiempo (s)')
ax2.set_ylabel('Velocidad angular (dps)')
ax2.legend()

ax3.plot(timestamps, mx_values, label='Eje x Magnetómetro')
ax3.plot(timestamps, my_values, label='Eje y Magnetómetro')
ax3.plot(timestamps, mz_values, label='Eje z Magnetómetro')
ax3.set_xlabel('Tiempo (s)')
ax3.set_ylabel('Centro de gravedad (Gauss)')
ax3.legend()

ax4.plot(timestamps, roll, label='Roll')
ax4.plot(timestamps, pitch, label='Pitch')
ax4.plot(timestamps, yaw, label='Yaw')
ax4.set_xlabel('Tiempo (s)')
ax4.legend()

plt.subplots_adjust(hspace=0.5, wspace=0.3)

plt.show()

current_date = datetime.now()
day = current_date.day
month = current_date.month
year = current_date.year

route = "figures/image"

if not os.path.exists(route):
    os.makedirs(route)

fileName = current_date.strftime('%Y-%m-%d_%H%M%S.png')

route_saved = os.path.join(route, fileName)

fig.savefig(route_saved)





