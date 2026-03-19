import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Enviamos un mensaje de prueba
sock.sendto(b"HOLA ROBOT", ("192.168.4.1", 4210))
# Esperamos ver si el robot responde algo
data, addr = sock.recvfrom(1024)
print(f"Recibido: {data.decode()}")