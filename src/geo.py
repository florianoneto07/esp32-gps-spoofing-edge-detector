"""Funções geodésicas leves sobre uma Terra esférica.

Usam apenas `math` e operações em ponto flutuante, para facilitar o porte
para C ou MicroPython no ESP32. Ângulos de entrada e saída em graus.
"""

import math

EARTH_RADIUS_M = 6371008.8  # raio médio da Terra (IUGG)


def haversine_m(lat1, lon1, lat2, lon2):
    """Distância de grande círculo entre dois pontos, em metros."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = phi2 - phi1
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlmb / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Rumo inicial do ponto 1 para o ponto 2, em [0, 360): 0 = norte, 90 = leste."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlmb = math.radians(lon2 - lon1)
    x = math.sin(dlmb) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlmb)
    return math.degrees(math.atan2(x, y)) % 360.0


def offset_latlon(lat, lon, north_m, east_m):
    """Desloca (lat, lon) por north_m e east_m metros.

    Aproximação plana local (equiretangular), adequada para deslocamentos de
    até alguns quilômetros, que é o caso de um voo de sUAS.
    """
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)
