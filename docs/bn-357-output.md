


## **For more info lookup NEMA 0183 standart**
# First two letters meaning - NMEA Message Talker ID
- **GPS, SBAS, QZSS** -P G
- **GLONASS russian** - GL 
- **GALILEO european** - GA
- **BEIDOU chinese** - GB
- **Any combination of GNSS** - GN


# Letters after first two

- **$xxBOD** – Bearing, origin to destination
- **$xxBWC** – Bearing and distance to waypoint, great circle
- **$xxGGA** – Global Positioning System Fix Data
- **$xxGLL** – Geographic position, latitude / longitude
- **$xxGSA** – GPS DOP and active satellites 
- **$xxGSV** – GPS Satellites in view
- **$xxHDT** – Heading, True
- **$xxR00** – List of waypoints in currently active route
- **$xxRMA** – Recommended minimum specific Loran-C data
- **$xxRMB** – Recommended minimum navigation info
- **$xxRMC** – Recommended minimum specific GPS/Transit data
- **$xxRTE** – Routes
- **$xxTRF** – Transit Fix Data
- **$xxSTN** – Multiple Data ID
- **$xxVBW** – Dual Ground / Water Speed
- **$xxVTG** – Track made good and ground speed
- **$xxWPL** – Waypoint location
- **$xxXTE** – Cross-track error, Measured
- **$xxZDA** – Date & Time

# Example parsing

<p>$GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>

- GNGGA         - Any GNSS satellite, sending global positioning system fix data
- 182459.00     - Time 
- 5551.75853,N  - Longitude. N 55°51'75.853''       DMS (degrees, minutes, seconds)*
- 00950.61588,E - Latitude. E 9°50'61.588''         DMS (degrees, minutes, seconds)*
- 1             - Quality of gps: 0 - invalid, 1 - standard GPS, 2 - differential GPS, 6 - estimated fix
- 05            - Satellites visible
- 5.31          - HDOP horizontal dilution of precision
- -14.8         - altitude
- M             - Units of altitude
- 43.4          - Geoid seperation
- *56           - Checksum

# Rest of the data
<p>$GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
<p>$GNRMC,182459.00,A,5551.75853,N,00950.61588,E,0.247,,131122,,,A*68</p>
<p>$GNVTG,,T,,M,0.247,N,0.457,K,A*3A</p>
<p>$GNGGA,182459.00,5551.75853,N,00950.61588,E,1,05,5.31,-14.8,M,43.4,M,,*56</p>
<p>$GNGSA,A,3,32,22,25,,,,,,,,,,9.21,5.31,7.53*16</p>
<p>$GNGSA,A,3,82,71,,,,,,,,,,,9.21,5.31,7.53*1C</p>
<p>$GPGSV,3,1,10,03,07,003,,06,26,078,,17,02,036,,19,28,049,*7E</p>
<p>$GPGSV,3,2,10,22,27,313,25,24,44,147,,25,54,256,26,29,08,201,19*75</p>
<p>$GPGSV,3,3,10,31,05,303,,32,40,280,26*74</p>
<p>$GLGSV,3,1,09,70,27,257,15,71,31,316,29,72,07,359,,73,14,285,19*68</p>
<p>$GLGSV,3,2,09,78,01,099,,79,50,087,13,81,50,092,,82,26,151,22*69</p>
<p>$GLGSV,3,3,09,88,25,028,*51</p>
<p>$GNGLL,5551.75853,N,00950.61588,E,182459.00,A,A*72</p>
<p>$GNRMC,182500.00,A,5551.75843,N,00950.61576,E,0.856,,131122,,,A*6F</p>
<p>$GNVTG,,T,,M,0.856,N,1.586,K,A*3C</p>
<p>$GNGGA,182500.00,5551.75843,N,00950.61576,E,1,05,5.31,-14.6,M,43.4,M,,*55</p>
<p>$GNGSA,A,3,32,22,25,,,,,,,,,,9.21,5.31,7.52*17</p>
<p>$GNGSA,A,3,82,71,,,,,,,,,,,9.21,5.31,7.52*1D</p>
<p>$GPGSV,3,1,10,03,07,003,,06,26,078,,17,02,036,,19,28,049,*7E</p>
<p>$GPGSV,3,2,10,22,27,313,25,24,44,147,,25,54,256,27,29,08,201,20*7E</p>
<p>$GPGSV,3,3,10,31,05,303,,32,40,280,26*74</p>
<p>$GLGSV,3,1,09,70,27,257,14,71,31,316,29,72,07,359,,73,14,285,19*69</p>
<p>$GLGSV,3,2,09,78,01,099,,79,50,087,12,81,50,092,,82,26,151,22*68</p>
<p>$GLGSV,3,3,09,88,25,028,*51</p>
<p>$GNGLL,5551.75843,N,00950.61576,E,182500.00,A,A*7F</p>
<p>$GNRMC,182501.00,A,5551.75853,N,00950.61573,E,0.817,,131122,,,A*6F</p>
<p>$GNVTG,,T,,M,0.817,N,1.512,K,A*34</p>
<p>$GNGGA,182501.00,5551.75853,N,00950.61573,E,1,05,5.31,-14.4,M,43.4,M,,*52</p>
<p>$GNGSA,A,3,32,22,25,,,,,,,,,,9.20,5.31,7.52*16</p>
<p>$GNGSA,A,3,82,71,,,,,,,,,,,9.20,5.31,7.52*1C</p>
<p>$GPGSV,3,1,10,03,07,003,,06,26,078,,17,02,036,,19,28,049,*7E</p>
<p>$GPGSV,3,2,10,22,27,313,25,24,44,147,,25,54,256,26,29,08,201,19*75</p>
<p>$GPGSV,3,3,10,31,05,303,,32,40,280,26*74</p>
<p>$GLGSV,3,1,09,70,27,257,12,71,31,316,29,72,07,359,,73,14,285,19*6F</p>
<p>$GLGSV,3,2,09,78,01,099,,79,50,087,13,81,50,092,,82,26,151,22*69</p>
<p>$GLGSV,3,3,09,88,25,028,*51</p>
<p>$GNGLL,5551.75853,N,00950.61573,E,182501.00,A,A*7A</p>
<p>$GNRMC,182502.00,A,5551.75849,N,00950.61628,E,0.293,,131122,,,A*6C</p>
<p>$GNVTG,,T,,M,0.293,N,0.542,K,A*36</p>
<p>$GNGGA,182502.00,5551.75849,N,00950.61628,E,1,05,5.31,-14.5,M,43.4,M,,*56</p>
<p>$GNGSA,A,3,32,22,25,,,,,,,,,,9.20,5.31,7.51*15</p>
<p>$GNGSA,A,3,82,71,,,,,,,,,,,9.20,5.31,7.51*1F</p>
<p>$GPGSV,3,1,10,03,07,003,,06,26,078,,17,02,036,,19,28,049,*7E</p>
<p>$GPGSV,3,2,10,22,27,313,24,24,44,147,,25,54,256,26,29,08,201,19*74</p>
<p>$GPGSV,3,3,10,31,05,303,,32,40,280,26*74</p>
<p>$GLGSV,3,1,09,70,27,257,11,71,31,316,29,72,07,359,,73,14,285,19*6C</p>
<p>$GLGSV,3,2,09,78,01,099,,79,50,087,14,81,50,092,,82,26,151,</p>