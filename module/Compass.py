import numpy as np
from scipy.spatial.transform import Rotation as R

class Compass:
    
    def __init__(self):
        self.calibration_offset = None  
    
    def calibrate(self, quaternion):
        self.calibration_offset = self.quaternion_to_azimuth(quaternion)
        
    def quaternion_to_azimuth(self, quaternion):
        
        # Konwersja quaternionu na macierz rotacji
        r = R.from_quat(quaternion)
        
        # Ekstrakcja wektora skierowanego do przodu (oś X)
        forward_vector = r.apply([1, 0, 0])

        # Obliczenie azymutu (kąta względem osi Z w układzie globalnym)
        azimuth = np.arctan2(forward_vector[1], forward_vector[0])  # y, x
        azimuth_degrees = np.degrees(azimuth)

        # Normalizacja do zakresu [-180, 180]
        if azimuth_degrees > 180:
            azimuth_degrees -= 360

        return azimuth_degrees

    def get_corrected_azimuth(self, quaternion):

        if self.calibration_offset is None:
            print("Brak kalibracji! Użyj funkcji calibrate().")
            return None

        raw_azimuth = self.quaternion_to_azimuth(quaternion)
        corrected_azimuth = raw_azimuth - self.calibration_offset

        return corrected_azimuth

