# importation des bibliothèques

# bibli pour les changements de système de coordonnées
import pyproj

# bibli pour le calcul du champ mag
from pygeomag import GeoMag
from pygeomag import calculate_decimal_year
import datetime

#bibli pour rotation
import numpy as numpy
from scipy.spatial.transform import Rotation



# fonction qui passe du sytème de coordonées x,y,z à WGS84 (longitude, latitude, altitude)

def xyz_to_wgs84(x, y, z) :
    lon, lat = pyproj.transform(self.proj_utm, self.proj_wgs84, x, y)
    return lon, lat, z


def wgs84_to_xyz(lat0, long0) :
    utm_zone = math.floor((lon_0 + 180)/6) + 1
    proj_wgs84 = pyproj.Proj(proj = 'latlong', datum='WGS84')
    proj_utm = pyproj.Proj(proj = 'utm', zone = utm_zone, datum='WGS84')
    X_0, Y_0 = pyproj.transform(proj_wgs84, proj_utm, lon_0, lat_0)
    return X_0, Y_0


def mag_field(lon, lat, z):
    now = datetime.datetime.now()
    t = calculate_decimal_year(now)
    geomag = GeoMag(coefficients_file = "wmm/WMM_2025.COF")
    field = geomag.calculate(glat = lat, glon = lon, alt = -z, time = t)
    return field

def quaternion_to_rotation(qx,qy,qz,qw):
    R = Rotation.from_quat([qx,qy,qz,qw])
    return R.as_matrix()


def magfield_NED(qx,qy,qz,qw,field):
    R = quaternion_to_rotation(qx,qy,qz,qw)
    B = np.array([field.x ,field.y ,field.z])
    B_NED = np.matmul(np.transpose(R), B)
    return B_NED