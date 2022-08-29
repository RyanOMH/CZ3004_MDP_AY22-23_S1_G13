import socket


# PyGame settings
SCALING_FACTOR = 6
FRAMES = 60
WINDOW_SIZE = 1200, 1200

# Connection to RPi
RPI_HOST: str = "192.168.8.8"
RPI_PORT: int = 4160

# Connection to PC
PC_HOST: str = socket.gethostbyname(socket.gethostname())
PC_PORT: int = 4161