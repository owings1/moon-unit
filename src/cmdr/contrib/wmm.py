from __future__ import annotations

import math
import time
from collections import namedtuple

try:
  from typing import Iterable
except ImportError:
  pass

"""
Adapted for CircuitPython from PyWMM:

https://github.com/dougc95/pywmm

MIT License

Copyright (c) 2024 Douglas Rojas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

class Calculation(
  namedtuple(
    'Calculation', (
      'declination',
      'dip_angle',
      'intensity',
      'horizontal_intensity',
      'north_intensity',
      'east_intensity',
      'vertical_intensity'))):
  declination: float
  """
  Magnetic declination in degrees (positive east)
  """
  dip_angle: float
  """
  Magnetic dip angle in degrees (positive downward)

  Dip angle is the angle between the horizontal plane and the magnetic field vector,
  positive when the magnetic field points downward.
  """
  intensity: float
  """
  Total magnetic field intensity in nanoteslas (nT)
  """
  horizontal_intensity: float
  """
  Horizontal magnetic field intensity in nanoteslas (nT)

  The horizontal intensity is the magnitude of the horizontal components (north and east).
  """
  north_intensity: float
  """
  Northward magnetic field component in nanoteslas (nT)
  """
  east_intensity: float
  """
  Eastward magnetic field component in nanoteslas (nT)
  """
  vertical_intensity: float
  """
  Downward magnetic field component in nanoteslas (nT)
  """

class WMMv2:
  """
  World Magnetic Model (WMM) version 2 implementation.
  
  This class implements the World Magnetic Model which provides magnetic field parameters
  (declination, inclination, intensity) at given coordinates and time. The WMM is the standard
  model used by the U.S. Department of Defense, the U.K. Ministry of Defence, NATO, and the
  International Hydrographic Organization.
  
  Attributes:
    maxdeg (int): Maximum degree of spherical harmonic model.
    maxord (int): Maximum order of spherical harmonic model.
    defaultDate (float): Default decimal year to use if none specified.
    dec (float): Declination in degrees (positive east).
    dip (float): Dip/inclination angle in degrees (positive down).
    ti (float): Total intensity of the magnetic field in nT.
    bx (float): North component of the magnetic field in nT.
    by (float): East component of the magnetic field in nT.
    bz (float): Vertical/downward component of the magnetic field in nT.
    bh (float): Horizontal intensity of the magnetic field in nT.
  """

  def __init__(self, epoch: float, cof: Iterable[tuple[int, int, float, float, float, float]]):
    """
    Initialize the WMMv2 model.
    """
    try:
      import ulab.numpy as np
    except ModuleNotFoundError:
      import numpy as np # type: ignore

    self.maxdeg = 12
    self.maxord = self.maxdeg
    self.defaultDate = epoch + 2.5

    # Magnetic field outputs (nT and degrees)
    self.dec = 0.0   # declination
    self.dip = 0.0   # dip angle
    self.ti = 0.0    # total intensity
    self.bx = 0.0    # north intensity
    self.by = 0.0    # east intensity
    self.bz = 0.0    # vertical intensity
    self.bh = 0.0    # horizontal intensity

    # Epoch and caching variables (for geodetic conversion)
    self.epoch = epoch
    self.otime = self.oalt = self.olat = self.olon = -1000.0

    # WGS-84/IAU constants
    self.a = 6378.137           # semi-major axis (km) of WGS-84 ellipsoid
    self.b = 6356.7523142       # semi-minor axis (km) of WGS-84 ellipsoid
    self.re = 6371.2            # Earth's mean radius (km)
    self.a2 = self.a * self.a   # a squared
    self.b2 = self.b * self.b   # b squared
    self.c2 = self.a2 - self.b2 # c squared (c is linear eccentricity)
    self.a4 = self.a2 * self.a2 # a to the fourth power
    self.b4 = self.b2 * self.b2 # b to the fourth power
    self.c4 = self.a4 - self.b4 # c to the fourth power

    # Allocate arrays for spherical harmonic coefficients and calculations
    self.c    = [[0.0 for _ in range(13)] for _ in range(13)]  # Main field coefficients
    self.cd   = [[0.0 for _ in range(13)] for _ in range(13)]  # Secular variation coefficients
    self.tc   = [[0.0 for _ in range(13)] for _ in range(13)]  # Time-adjusted coefficients
    self.dp   = [[0.0 for _ in range(13)] for _ in range(13)]  # Legendre derivative function
    self.snorm = np.zeros(169)  # Schmidt normalization factors (13x13 = 169 entries)
    self.sp   = np.zeros(13)    # sin(m*phi) (longitude)
    self.cp   = np.zeros(13)    # cos(m*phi) (longitude)
    self.fn   = np.zeros(13)    # n+1 factors
    self.fm   = np.zeros(13)    # m factors
    self.pp   = np.zeros(13)    # Associated Legendre polynomial values
    self.k    = [[0.0 for _ in range(13)] for _ in range(13)]  # Recursion coefficients

    # Variables for geodetic-to-spherical conversion
    self.ct = 0.0  # cos(theta)
    self.st = 0.0  # sin(theta)
    self.r  = 0.0  # distance from center of Earth
    self.d  = 0.0  # down component
    self.ca = 0.0  # cos(alpha)
    self.sa = 0.0  # sin(alpha)

    for (n, m, gnm, hnm, dgnm, dhnm) in cof:
      self.c[m][n] = gnm
      self.cd[m][n] = dgnm
      if m != 0:
        self.c[n][m - 1] = hnm
        self.cd[n][m - 1] = dhnm
    self.calculation: Calculation|None = None
    self.setup()

  def setup(self) -> None:
    """
    Initialize the model by setting up normalization factors and processing coefficients.
    
    This method:
    1. Calculates Schmidt normalization factors
    2. Performs normalization on the coefficients
    3. Initializes recursion coefficients for Legendre functions
    """
    self.maxord = self.maxdeg
    self.sp[0] = 0.0
    self.cp[0] = self.snorm[0] = self.pp[0] = 1.0
    self.dp[0][0] = 0.0

    # Schmidt normalization factors
    self.snorm[0] = 1.0
    n = 1
    while n <= self.maxord:
      self.snorm[n] = self.snorm[n - 1] * (2 * n - 1) / n
      j = 2
      m = 0
      D1 = 1
      D2 = (n - m + D1) / D1
      while D2 > 0:
        self.k[m][n] = float(((n - 1)**2 - m**2)) / float((2 * n - 1) * (2 * n - 3))
        if m > 0:
          flnmj = ((n - m + 1) * j) / float(n + m)
          self.snorm[n + m * 13] = self.snorm[n + (m - 1) * 13] * math.sqrt(flnmj)
          j = 1
          self.c[n][m - 1] = self.snorm[n + m * 13] * self.c[n][m - 1]
          self.cd[n][m - 1] = self.snorm[n + m * 13] * self.cd[n][m - 1]
        self.c[m][n] = self.snorm[n + m * 13] * self.c[m][n]
        self.cd[m][n] = self.snorm[n + m * 13] * self.cd[m][n]
        D2 = D2 - 1
        m = m + D1
      self.fn[n] = n + 1
      self.fm[n] = n
      n = n + 1
    self.k[1][1] = 0.0
    self.otime = self.oalt = self.olat = self.olon = -1000.0

  def observe(self, fLat: float, fLon: float, year: float|time.struct_time|tuple[int, ...], altitude: float = 0) -> Calculation:
    """
    Calculate geomagnetic field components at a specified location and time.
    
    This function implements the spherical harmonic synthesis algorithm for the World 
    Magnetic Model (WMM). It computes the magnetic field vector components and derived
    quantities at the specified geodetic coordinates and time, storing results in the 
    given instance.
    
    Parameters:
      fLat (float): Geodetic latitude in decimal degrees (-90 to 90)
      fLon (float): Geodetic longitude in decimal degrees (-180 to 180)
      year (float): Decimal year (e.g., 2025.5 for July 1, 2025)
      altitude (float, optional): Altitude above WGS-84 ellipsoid in kilometers. 
                                  Default is 0 (sea level).
    
    Returns:
      Calculation
    
    Algorithm notes:
      1. Converts geodetic coordinates to spherical coordinates
      2. Computes time-adjusted Gauss coefficients
      3. Calculates associated Legendre functions
      4. Synthesizes magnetic field components in spherical coordinates
      5. Converts to geodetic components (north, east, down)
      6. Derives derived quantities (declination, inclination, intensities)
        
    The function uses caching to avoid redundant calculations when called repeatedly
    with the same latitude, longitude, or altitude.
    """
    if isinstance(year, tuple):
      year = decimyear(year)
    self.glat = fLat
    self.glon = fLon
    self.alt = altitude
    self.time = year

    dt = self.time - self.epoch  # Time difference from epoch in years
    pi = math.pi
    dtr = pi / 180.0  # Degrees to radians conversion factor
    rlon = self.glon * dtr  # Longitude in radians
    rlat = self.glat * dtr  # Latitude in radians
    srlon = math.sin(rlon)
    srlat = math.sin(rlat)
    crlon = math.cos(rlon)
    crlat = math.cos(rlat)
    srlat2 = srlat * srlat
    crlat2 = crlat * crlat

    # Initialize sine and cosine of longitude for recursion
    self.sp[1] = srlon
    self.cp[1] = crlon

    # Geodetic to spherical coordinate transformation
    # Recalculate only if latitude or altitude has changed
    if altitude != self.oalt or fLat != self.olat:
      # Calculate geocentric coordinates from geodetic
      q = math.sqrt(self.a2 - self.c2 * srlat2)  # Distance from rotation axis
      q1 = altitude * q
      q2 = ((q1 + self.a2) / (q1 + self.b2)) ** 2
      ct = srlat / math.sqrt(q2 * crlat2 + srlat2)  # cos(theta) - theta is geocentric colatitude
      st = math.sqrt(1.0 - ct * ct)                 # sin(theta)
      r2 = altitude * altitude + 2.0 * q1 + (self.a4 - self.c4 * srlat2) / (q * q)
      r = math.sqrt(r2)  # Geocentric radius
      d = math.sqrt(self.a2 * crlat2 + self.b2 * srlat2)  # Distance from rotation axis
      ca = (altitude + d) / r  # cos(alpha) - alpha is angle between radial vector and horizontal
      sa = self.c2 * crlat * srlat / (r * d)    # sin(alpha)
      
      # Cache the calculated values
      self.ct = ct
      self.st = st
      self.r  = r
      self.d  = d
      self.ca = ca
      self.sa = sa
    else:
      # Use cached values
      ct = self.ct
      st = self.st
      r  = self.r
      d  = self.d
      ca = self.ca
      sa = self.sa

    # Compute recursion of sin(m*lon) and cos(m*lon)
    # Recalculate only if longitude has changed
    if fLon != self.olon:
      m = 2
      while m <= self.maxord:
        self.sp[m] = self.sp[1] * self.cp[m - 1] + self.cp[1] * self.sp[m - 1]
        self.cp[m] = self.cp[1] * self.cp[m - 1] - self.sp[1] * self.sp[m - 1]
        m += 1

    # Powers of (earth_radius/geocentric_radius)
    aor = self.re / r  # Ratio of reference sphere radius to geocentric radius
    ar = aor * aor        # Initial value for recursion
    
    # Initialize magnetic field components
    br = 0.0  # Radial component
    bt = 0.0  # Theta (colatitude) component
    bp = 0.0  # Phi (longitude) component
    bpp = 0.0  # Special case for poles

    # Main spherical harmonic synthesis loop
    n = 1
    while n <= self.maxord:
      ar = ar * aor  # (earth_radius/geocentric_radius)^(n+2)
      m = 0
      D1 = 1
      D2 = (n + m + D1) / D1
      while D2 > 0:
        # Compute Schmidt quasi-normalized associated Legendre functions
        # and their theta derivatives
        if altitude != self.oalt or fLat != self.olat:
          if n == m:
            self.snorm[n + m * 13] = st * self.snorm[n - 1 + (m - 1) * 13]
            self.dp[m][n] = st * self.dp[m - 1][n - 1] + ct * self.snorm[n - 1 + (m - 1) * 13]
          if n == 1 and m == 0:
            self.snorm[n + m * 13] = ct * self.snorm[n - 1 + m * 13]
            self.dp[m][n] = ct * self.dp[m][n - 1] - st * self.snorm[n - 1 + m * 13]
          if n > 1 and n != m:
            if m > n - 2:
              self.snorm[n - 2 + m * 13] = 0.0
              self.dp[m][n - 2] = 0.0
            self.snorm[n + m * 13] = ct * self.snorm[n - 1 + m * 13] - self.k[m][n] * self.snorm[n - 2 + m * 13]
            self.dp[m][n] = ct * self.dp[m][n - 1] - st * self.snorm[n - 1 + m * 13] - self.k[m][n] * self.dp[m][n - 2]

        # Time-adjusted Gauss coefficients
        self.tc[m][n] = self.c[m][n] + dt * self.cd[m][n]
        if m != 0:
          self.tc[n][m - 1] = self.c[n][m - 1] + dt * self.cd[n][m - 1]
        
        # Spherical harmonic terms
        par = ar * self.snorm[n + m * 13]
        if m == 0:
          temp1 = self.tc[m][n] * self.cp[m]
          temp2 = self.tc[m][n] * self.sp[m]
        else:
          temp1 = self.tc[m][n] * self.cp[m] + self.tc[n][m - 1] * self.sp[m]
          temp2 = self.tc[m][n] * self.sp[m] - self.tc[n][m - 1] * self.cp[m]
        
        # Accumulate field components in spherical coordinates
        bt = bt - ar * temp1 * self.dp[m][n]     # theta component
        bp = bp + self.fm[m] * temp2 * par       # phi component
        br = br + self.fn[n] * temp1 * par       # radial component
        
        # Special case for poles (where latitude = +/- 90 degrees)
        if st == 0.0 and m == 1:
          if n == 1:
            self.pp[n] = self.pp[n - 1]
          else:
            self.pp[n] = ct * self.pp[n - 1] - self.k[m][n] * self.pp[n - 2]
          parp = ar * self.pp[n]
          bpp = bpp + self.fm[m] * temp2 * parp
        
        D2 -= 1
        m += 1
      n += 1

    # Handle special case for poles
    if st == 0.0:
      bp = bpp
    else:
      bp = bp / st  # Adjust phi component

    # Convert from spherical to geodetic components
    self.bx = -bt * ca - br * sa  # North component
    self.by = bp                  # East component
    self.bz = bt * sa - br * ca   # Down component

    # Calculate derived quantities
    self.bh = math.sqrt(self.bx * self.bx + self.by * self.by)  # Horizontal intensity
    self.ti = math.sqrt(self.bh * self.bh + self.bz * self.bz)  # Total intensity

    # Calculate declination and inclination (dip) angles
    self.dec = math.atan2(self.by, self.bx) / dtr  # Declination in degrees
    self.dip = math.atan2(self.bz, self.bh) / dtr  # Inclination in degrees

    # Update cached location and time
    self.otime = self.time
    self.oalt = altitude
    self.olat = fLat
    self.olon = fLon

    self.calculation = Calculation(
      declination=self.dec,
      dip_angle=self.dip,
      intensity=self.ti,
      horizontal_intensity=self.bh,
      north_intensity=self.bx,
      east_intensity=self.by,
      vertical_intensity=self.bz)
    return self.calculation

def decimyear(t: time.struct_time|tuple[int, ...]) -> float:
  """
  Convert time struct to year with fractional component
  """
  t = time.struct_time(t)
  # A year is a leap year if divisible by 4, but not 100 (unless also by 400)
  # Simple check for our GPS lifespan:
  is_leap = (t.tm_year % 4 == 0) and (t.tm_year % 100 != 0 or t.tm_year % 400 == 0)
  days_in_year = 366 if is_leap else 365
  # Use (yday - 1) so Jan 1st 00:00:00 is exactly 2025.0
  # Add fractional day (hours/mins/secs) for higher precision if needed
  fractional_day = (t.tm_yday - 1) + (t.tm_hour / 24.0) + (t.tm_min / 1440.0)
  return t.tm_year + (fractional_day / days_in_year)