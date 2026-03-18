import requests
import time
import statistics
import csv
import matplotlib.pyplot as plt

# Configuración
URL = "http://192.168.4.1/stress"
NUM_PRUEBAS = 50
tiempos = []

print(f"--- Iniciando prueba profesional en {URL} ---")

for i in range(NUM_PRUEBAS):
    try:
        inicio = time.perf_counter()
        r = requests.get(URL, timeout=2)
        fin = time.perf_counter()
        
        latencia = (fin - inicio) * 1000
        tiempos.append(latencia)
        print(f"Muestra {i+1}: {latencia:.2f} ms")
        time.sleep(0.05) 
    except Exception as e:
        print(f"Error: {e}")

# 1. Guardar datos en CSV
with open('resultados_neuro.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["Muestra", "Latencia_ms"])
    for idx, t in enumerate(tiempos):
        writer.writerow([idx + 1, round(t, 2)])

# 2. Generar Análisis Estadístico
promedio = statistics.mean(tiempos)
desviacion = statistics.stdev(tiempos)

# 3. Crear Gráfica
plt.figure(figsize=(10, 5))
plt.plot(tiempos, marker='o', linestyle='-', color='b', label='Latencia HTTP')
plt.axhline(y=promedio, color='r', linestyle='--', label=f'Promedio ({promedio:.2f} ms)')
plt.fill_between(range(len(tiempos)), promedio - desviacion, promedio + desviacion, 
                 color='r', alpha=0.2, label='Desviación Estándar')

plt.title('Análisis de Latencia - ESP32 Stress Test')
plt.xlabel('Número de Petición')
plt.ylabel('Tiempo de Respuesta (ms)')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)

# Guardar la gráfica como imagen
plt.savefig('grafica_latencia.png')
print("\n Archivos generados: 'resultados_neuro.csv' y 'grafica_latencia.png'")
plt.show()