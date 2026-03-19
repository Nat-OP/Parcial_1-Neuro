import socket
import time
import matplotlib.pyplot as plt # Importamos la librería

ESP32_IP = "192.168.4.1"
PORT = 4210

# Listas para almacenar los resultados
latencias = []
intentos = []

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)

print("Enviando ráfaga... (Presiona Ctrl+C para detener)")

try:
    for i in range(100):
        try:
            inicio = time.perf_counter()
            sock.sendto(b"ping", (ESP32_IP, PORT))
            data, addr = sock.recvfrom(1024)
            fin = time.perf_counter()
            
            ms = (fin - inicio) * 1000
            latencias.append(ms)
            intentos.append(i)
            
            print(f"Respuesta {i}: {data.decode()} | Latencia: {ms:.2f} ms")
        except socket.timeout:
            print(f"Intento {i}: Timeout... saltando")
            # Opcional: latencias.append(500) # Para visualizar el timeout como un pico
        
        time.sleep(0.1) 

except KeyboardInterrupt:
    print("\nPrueba interrumpida por el usuario.")

# --- Generación y Guardado del Gráfico ---
if latencias:
    plt.figure(figsize=(10, 5))
    plt.plot(intentos, latencias, marker='o', linestyle='-', color='b', label='Latencia UDP')
    
    # Línea de promedio (muy útil para análisis)
    promedio = sum(latencias) / len(latencias)
    plt.axhline(y=promedio, color='r', linestyle='--', label=f'Promedio: {promedio:.2f} ms')
    
    plt.title('Análisis de Latencia ESP32')
    plt.xlabel('Número de Intento')
    plt.ylabel('Tiempo (ms)')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()

    # 1. Guardar el archivo (puedes usar .png, .jpg o .pdf)
    nombre_archivo = f"reporte_latencia_{int(time.time())}.png"
    plt.savefig(nombre_archivo, dpi=300, bbox_inches='tight')
    print(f"✅ Gráfico guardado como: {nombre_archivo}")

    # 2. Mostrar en pantalla (opcional)
    plt.show() 
else:
    print("No hay datos para guardar.")