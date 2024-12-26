import pandas as pd
import serial
import time
from datetime import datetime
import os
from picamera import PiCamera

# Configurar el puerto serial
arduino_port = '/dev/ttyACM0'
baud_rate = 9600
timeout_reconexion = 10  # Tiempo en segundos para reiniciar el puerto si no hay datos
ultimo_setpoint = {'T': None, 'H': None}  # Almacenar el último setpoint enviado

# Inicializar la cámara
camera = PiCamera()
camera.resolution = (1920, 1080)  # Configurar resolución de la cámara
camera.rotation = 0  # Ajustar rotación si es necesario

# Tiempo para tomar fotos (en segundos)
photo_interval = 420  # Cambia este valor para ajustar el intervalo (600 = 10 minutos)
last_photo_time = time.time()  # Inicializar el último tiempo de foto
prephoto_time = 10  # Tiempo previo al disparo de la foto para enviar "on"

def reiniciar_puerto():
    global arduino
    arduino.close()
    print("Reiniciando el puerto serial...")
    time.sleep(2)  # Pausa para liberar el puerto
    arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)  # Esperar a que el puerto se inicie
    if ultimo_setpoint['T'] is not None and ultimo_setpoint['H'] is not None:
        arduino.write(f"T{ultimo_setpoint['T']}\n".encode())
        time.sleep(1)
        arduino.write(f"H{ultimo_setpoint['H']}\n".encode())
        print(f"Reenviado último setpoint: T={ultimo_setpoint['T']}, H={ultimo_setpoint['H']}")

# Conectar al puerto serial
arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)

# Generar un nombre de archivo único con fecha y hora actuales
timestamp = datetime.now().strftime('%H%M_%d%m%Y')
output_file_path = f'/home/pi/PID/Datos/DatosRecibidos_{timestamp}.txt'

def guardar_datos_en_archivo(dato):
    with open(output_file_path, 'a') as file:
        file.write(dato + '\n')

def enviar_on():
    print("Enviando 'on' al Arduino...")
    arduino.write("on\n".encode())
    time.sleep(1)

def tomar_foto(actual_temp, actual_hum):
    global last_photo_time
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    foto_path = f'/home/pi/PID/Fotos/Foto_{timestamp}_Temp{actual_temp:.2f}_Hum{actual_hum:.2f}.jpg'

    # Tomar la foto
    print(f"Tomando foto: {foto_path}")
    try:
        camera.capture(foto_path)
        print(f"Foto capturada: {foto_path}")
    except Exception as e:
        print(f"Error al tomar la foto: {e}")

    # Enviar "off" al Arduino después de tomar la foto
    print("Enviando 'off' al Arduino...")
    arduino.write("off\n".encode())
    time.sleep(1)

    # Actualizar el tiempo de la última foto
    last_photo_time = time.time()

def verificar_setpoint(setpoint_temp, setpoint_hum, tolerancia=0.4):
    tiempo_ultimo_dato = time.time()
    while True:
        if arduino.in_waiting > 0:
            sensor_data = arduino.readline().decode().strip()
            if sensor_data:
                guardar_datos_en_archivo(sensor_data)
                print("Datos recibidos del Arduino:", sensor_data)
                tiempo_ultimo_dato = time.time()  # Resetear temporizador

                try:
                    sensor_data = sensor_data.replace('%', '').strip()
                    data_values = list(map(float, sensor_data.split()))

                    actual_temp = data_values[6]
                    actual_hum = data_values[7]

                    print(f"Actual Temp={actual_temp}, Actual Hum={actual_hum}, Setpoint Temp={setpoint_temp}, Setpoint Hum={setpoint_hum}")

                    # Verificar si los valores están dentro del rango de tolerancia
                    if abs(actual_temp - setpoint_temp) <= tolerancia and abs(actual_hum - setpoint_hum) <= tolerancia:
                        print(f"Setpoints alcanzados: Temp={actual_temp}, Hum={actual_hum}")
                        return actual_temp, actual_hum
                except (ValueError, IndexError):
                    print("Error procesando los datos:", sensor_data)
                    continue

        # Verificar si el tiempo sin datos supera el límite
        if time.time() - tiempo_ultimo_dato > timeout_reconexion:
            reiniciar_puerto()
            tiempo_ultimo_dato = time.time()

# Cargar el archivo de datos
file_path = '/home/pi/PID/ControlTempHum/DatosEmbarque3.txt'
data = pd.read_csv(file_path, sep='\t')

# Recorre el archivo con setpoints
for index, row in data.iterrows():
    setpoint_temp = row['T']
    setpoint_hum = row['HR']
    tiempo_setpoint = row['t']  # Tiempo en segundos para mantener el setpoint
    
    # Enviar setpoints al Arduino
    arduino.write(f"T{setpoint_temp}\n".encode())
    time.sleep(1)
    arduino.write(f"H{setpoint_hum}\n".encode())
    print(f"Enviado setpoint de temperatura: {setpoint_temp} y humedad: {setpoint_hum}")
    
    ultimo_setpoint = {'T': setpoint_temp, 'H': setpoint_hum}  # Guardar último setpoint
    arduino.write("off\n".encode())  # Apagar después de enviar setpoint
    time.sleep(1)
    print("Esperando a alcanzar los setpoints...")

    # Verificar si se alcanzan los setpoints
    actual_temp, actual_hum = verificar_setpoint(setpoint_temp, setpoint_hum)
    print("Setpoints alcanzados. Manteniendo durante el tiempo especificado...")

    # Mantener el setpoint durante el tiempo especificado
    inicio_tiempo = time.time()
    while time.time() - inicio_tiempo < tiempo_setpoint:
        current_time = time.time()
        
        # Capturar datos del Arduino
        if arduino.in_waiting > 0:
            sensor_data = arduino.readline().decode().strip()
            if sensor_data:
                print("Datos recibidos del Arduino:", sensor_data)
                guardar_datos_en_archivo(sensor_data)
        
        # Enviar "on" 10 segundos antes de tomar una foto
        if current_time - last_photo_time >= photo_interval - prephoto_time and current_time - last_photo_time < photo_interval - prephoto_time + 1:
            enviar_on()
        
        # Tomar una foto si ha pasado el intervalo completo
        if current_time - last_photo_time >= photo_interval:
            tomar_foto(actual_temp, actual_hum)
    
    print(f"Setpoint mantenido durante {tiempo_setpoint} segundos. Pasando al siguiente...")

# Cerrar la comunicación serial
arduino.close()
