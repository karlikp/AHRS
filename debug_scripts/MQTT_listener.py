import numpy as np

def parse_icp_matrix_binary(payload):
    # Odczytaj timestamp (pierwsze 8 bajtów)
    timestamp = np.frombuffer(payload[:8], dtype=np.float64)[0]

    # Odczytaj macierz (pozostałe bajty)
    matrix = np.frombuffer(payload[8:], dtype=np.float32).reshape((4,4))

    return timestamp, matrix
