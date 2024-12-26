import pandas as pd
import matplotlib
matplotlib.use('TkAgg')  # Cambia el backend
import matplotlib.pyplot as plt

# Función para leer el archivo de texto ignorando las líneas que contienen 'Nuevos setpoints' y encabezados
def cargar_datos_txt(archivo):
    data = []
    with open(archivo, 'r') as file:
        for line in file:
            if "Nuevos setpoints" not in line and not line.startswith("Sensor1"):
                data.append(line.strip().split('\t'))  # Separar por tabulaciones
    
    # Crear un DataFrame y asignar nombres de columnas
    columnas = ["Sensor1", "Sensor2", "Sensor3", "Sensor4", "Sensor5", 
                "Sensor1Hum", "PromTemp", "PromHum", "SetTemp", "SetHum", "Potencia"]
    df = pd.DataFrame(data, columns=columnas)
    
    # Quitar el símbolo '%' de la columna Potencia y convertir a float
    df['Potencia'] = df['Potencia'].str.replace('%', '', regex=False)
    
    # Convertir todas las columnas a float
    for col in columnas:
        df[col] = pd.to_numeric(df[col], errors='coerce')  # Convierte valores no numéricos a NaN

    # Eliminar filas con valores NaN
    df = df.dropna()
    df['Tiempo'] = df.index * 5
    
    return df

# Cargar los datos desde el archivo de texto
data = cargar_datos_txt('/home/pi/PID/Datos/DatosRecibidos_1346_22112024.txt')

# Filtrar los datos hasta el índice 3000
filtered_data = data.iloc[0:6000]

# Crear el gráfico combinado
fig, ax1 = plt.subplots(figsize=(12, 6))

# Gráfico de PromTemp y SetTemp en el eje izquierdo
ax1.plot(filtered_data['Tiempo'], filtered_data['PromTemp'], label='Promedio Temp', color='blue', linestyle='-')
ax1.plot(filtered_data['Tiempo'], filtered_data['SetTemp'], label='Setpoint Temp', color='orange', linestyle='--')
ax1.set_ylabel('Temperatura (°C)')
ax1.legend(loc='upper left')
ax1.grid(True)

# Crear el segundo eje Y para PromHum y SetHum
ax2 = ax1.twinx()
ax2.plot(filtered_data['Tiempo'], filtered_data['PromHum'], label='Promedio Humedad', color='#FF6347', linestyle='-')
ax2.plot(filtered_data['Tiempo'], filtered_data['SetHum'], label='Setpoint Humedad', color='#FF0000', linestyle='--')
ax2.set_ylabel('Humedad (%)')
ax2.legend(loc='upper right')

# Título del gráfico combinado
plt.title('Temperatura y Humedad con sus Setpoints')

plt.tight_layout()
plt.show()

# Crear gráfico individual para Temperatura
plt.figure(figsize=(12, 6))
plt.plot(filtered_data['Tiempo'], filtered_data['PromTemp'], label='Promedio Temp', color='blue', linestyle='-')
plt.plot(filtered_data['Tiempo'], filtered_data['SetTemp'], label='Setpoint Temp', color='orange', linestyle='--')
plt.ylabel('Temperatura (°C)')
plt.title('Temperatura con Setpoint')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Crear gráfico individual para Humedad
plt.figure(figsize=(12, 6))
plt.plot(filtered_data['Tiempo'], filtered_data['PromHum'], label='Promedio Humedad', color='#FF6347', linestyle='-')
plt.plot(filtered_data['Tiempo'], filtered_data['SetHum'], label='Setpoint Humedad', color='#FF0000', linestyle='--')
plt.ylabel('Humedad (%)')
plt.title('Humedad con Setpoint')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
