import requests
import time
import statistics

# Configuración
URL = "http://192.168.4.1/stress"  # Puedes cambiarlo a /data o /ping
NUM_PRUEBAS = 50
tiempos = []

print(f"--- Iniciando prueba de latencia en {URL} ({NUM_PRUEBAS} peticiones) ---")

for i in range(NUM_PRUEBAS):
    try:
        inicio = time.perf_counter()
        r = requests.get(URL, timeout=2)
        fin = time.perf_counter()
        
        # Calculamos en milisegundos
        latencia = (fin - inicio) * 1000
        tiempos.append(latencia)
        
        print(f"Prueba {i+1}: {latencia:.2f} ms")
        time.sleep(0.1)  # Pequeña pausa para no saturar el buffer del ESP32
    except Exception as e:
        print(f"Error en prueba {i+1}: {e}")

# Cálculos finales
if tiempos:
    promedio = statistics.mean(tiempos)
    desviacion = statistics.stdev(tiempos)
    print("\n" + "="*30)
    print(f"RESULTADOS FINALES:")
    print(f"Promedio: {promedio:.2f} ms")
    print(f"Desviación Estándar: {desviacion:.2f} ms")
    print(f"Mínimo: {min(tiempos):.2f} ms")
    print(f"Máximo: {max(tiempos):.2f} ms")
    print("="*30)