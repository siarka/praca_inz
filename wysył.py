import math
import random
import time
import socket

# Parametry generowania waypointów


# Parametry komunikacji UDP
udp_host = "localhost"  # Adres docelowy hosta
udp_port = 12345  # Port docelowy

# Inicjalizacja gniazda UDP
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

for i in range(0, 11):
    value = i / 10.0
    if i % 2 == 1:
        value = -value
    waypoint = f"{value}\n"

    # Wysyłanie waypointu do hosta i portu za pomocą protokołu UDP
    udp_socket.sendto(waypoint.encode(), (udp_host, udp_port))

    print(f"Generated waypoint and sent it via UDP.")
    time.sleep(1)  # Czekaj 1 sekundę przed wygenerowaniem kolejnego waypointu
