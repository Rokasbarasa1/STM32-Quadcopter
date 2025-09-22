#include <stdio.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>

#define M_DEG_TO_RAD 0.0174533f
#define M_RAD_TO_DEG 57.29578f
#ifndef M_PI
#define M_PI 3.1415927f
#endif
#define NMAX 12
#define WMM_EARTH_RADIUS_KM 6371.2
#define WMM_EARTH_RADIUS_M 6371200.0f


// The full WMM.COF string, shortened here with "..."
const char* wmm_cof_2025_content = 
"    2025.0            WMM-2025     11/13/2024\n"
"  1  0  -29351.8       0.0       12.0        0.0\n"
"  1  1   -1410.8    4545.4        9.7      -21.5\n"
"  2  0   -2556.6       0.0      -11.6        0.0\n"
"  2  1    2951.1   -3133.6       -5.2      -27.7\n"
"  2  2    1649.3    -815.1       -8.0      -12.1\n"
"  3  0    1361.0       0.0       -1.3        0.0\n"
"  3  1   -2404.1     -56.6       -4.2        4.0\n"
"  3  2    1243.8     237.5        0.4       -0.3\n"
"  3  3     453.6    -549.5      -15.6       -4.1\n"
"  4  0     895.0       0.0       -1.6        0.0\n"
"  4  1     799.5     278.6       -2.4       -1.1\n"
"  4  2      55.7    -133.9       -6.0        4.1\n"
"  4  3    -281.1     212.0        5.6        1.6\n"
"  4  4      12.1    -375.6       -7.0       -4.4\n"
"  5  0    -233.2       0.0        0.6        0.0\n"
"  5  1     368.9      45.4        1.4       -0.5\n"
"  5  2     187.2     220.2        0.0        2.2\n"
"  5  3    -138.7    -122.9        0.6        0.4\n"
"  5  4    -142.0      43.0        2.2        1.7\n"
"  5  5      20.9     106.1        0.9        1.9\n"
"  6  0      64.4       0.0       -0.2        0.0\n"
"  6  1      63.8     -18.4       -0.4        0.3\n"
"  6  2      76.9      16.8        0.9       -1.6\n"
"  6  3    -115.7      48.8        1.2       -0.4\n"
"  6  4     -40.9     -59.8       -0.9        0.9\n"
"  6  5      14.9      10.9        0.3        0.7\n"
"  6  6     -60.7      72.7        0.9        0.9\n"
"  7  0      79.5       0.0       -0.0        0.0\n"
"  7  1     -77.0     -48.9       -0.1        0.6\n"
"  7  2      -8.8     -14.4       -0.1        0.5\n"
"  7  3      59.3      -1.0        0.5       -0.8\n"
"  7  4      15.8      23.4       -0.1        0.0\n"
"  7  5       2.5      -7.4       -0.8       -1.0\n"
"  7  6     -11.1     -25.1       -0.8        0.6\n"
"  7  7      14.2      -2.3        0.8       -0.2\n"
"  8  0      23.2       0.0       -0.1        0.0\n"
"  8  1      10.8       7.1        0.2       -0.2\n"
"  8  2     -17.5     -12.6        0.0        0.5\n"
"  8  3       2.0      11.4        0.5       -0.4\n"
"  8  4     -21.7      -9.7       -0.1        0.4\n"
"  8  5      16.9      12.7        0.3       -0.5\n"
"  8  6      15.0       0.7        0.2       -0.6\n"
"  8  7     -16.8      -5.2       -0.0        0.3\n"
"  8  8       0.9       3.9        0.2        0.2\n"
"  9  0       4.6       0.0       -0.0        0.0\n"
"  9  1       7.8     -24.8       -0.1       -0.3\n"
"  9  2       3.0      12.2        0.1        0.3\n"
"  9  3      -0.2       8.3        0.3       -0.3\n"
"  9  4      -2.5      -3.3       -0.3        0.3\n"
"  9  5     -13.1      -5.2        0.0        0.2\n"
"  9  6       2.4       7.2        0.3       -0.1\n"
"  9  7       8.6      -0.6       -0.1       -0.2\n"
"  9  8      -8.7       0.8        0.1        0.4\n"
"  9  9     -12.9      10.0       -0.1        0.1\n"
" 10  0      -1.3       0.0        0.1        0.0\n"
" 10  1      -6.4       3.3        0.0        0.0\n"
" 10  2       0.2       0.0        0.1       -0.0\n"
" 10  3       2.0       2.4        0.1       -0.2\n"
" 10  4      -1.0       5.3       -0.0        0.1\n"
" 10  5      -0.6      -9.1       -0.3       -0.1\n"
" 10  6      -0.9       0.4        0.0        0.1\n"
" 10  7       1.5      -4.2       -0.1        0.0\n"
" 10  8       0.9      -3.8       -0.1       -0.1\n"
" 10  9      -2.7       0.9       -0.0        0.2\n"
" 10 10      -3.9      -9.1       -0.0       -0.0\n"
" 11  0       2.9       0.0        0.0        0.0\n"
" 11  1      -1.5       0.0       -0.0       -0.0\n"
" 11  2      -2.5       2.9        0.0        0.1\n"
" 11  3       2.4      -0.6        0.0       -0.0\n"
" 11  4      -0.6       0.2        0.0        0.1\n"
" 11  5      -0.1       0.5       -0.1       -0.0\n"
" 11  6      -0.6      -0.3        0.0       -0.0\n"
" 11  7      -0.1      -1.2       -0.0        0.1\n"
" 11  8       1.1      -1.7       -0.1       -0.0\n"
" 11  9      -1.0      -2.9       -0.1        0.0\n"
" 11 10      -0.2      -1.8       -0.1        0.0\n"
" 11 11       2.6      -2.3       -0.1        0.0\n"
" 12  0      -2.0       0.0        0.0        0.0\n"
" 12  1      -0.2      -1.3        0.0       -0.0\n"
" 12  2       0.3       0.7       -0.0        0.0\n"
" 12  3       1.2       1.0       -0.0       -0.1\n"
" 12  4      -1.3      -1.4       -0.0        0.1\n"
" 12  5       0.6      -0.0       -0.0       -0.0\n"
" 12  6       0.6       0.6        0.1       -0.0\n"
" 12  7       0.5      -0.1       -0.0       -0.0\n"
" 12  8      -0.1       0.8        0.0        0.0\n"
" 12  9      -0.4       0.1        0.0       -0.0\n"
" 12 10      -0.2      -1.0       -0.1       -0.0\n"
" 12 11      -1.3       0.1       -0.0        0.0\n"
" 12 12      -0.7       0.2       -0.1       -0.1\n"
"999999999999999999999999999999999999999999999999\n"
"999999999999999999999999999999999999999999999999";

const char* wmm_cof_2020_content = 
"    2020.0            WMM-2020        12/10/2019\n"
"  1  0  -29404.5       0.0        6.7        0.0\n"
"  1  1   -1450.7    4652.9        7.7      -25.1\n"
"  2  0   -2500.0       0.0      -11.5        0.0\n"
"  2  1    2982.0   -2991.6       -7.1      -30.2\n"
"  2  2    1676.8    -734.8       -2.2      -23.9\n"
"  3  0    1363.9       0.0        2.8        0.0\n"
"  3  1   -2381.0     -82.2       -6.2        5.7\n"
"  3  2    1236.2     241.8        3.4       -1.0\n"
"  3  3     525.7    -542.9      -12.2        1.1\n"
"  4  0     903.1       0.0       -1.1        0.0\n"
"  4  1     809.4     282.0       -1.6        0.2\n"
"  4  2      86.2    -158.4       -6.0        6.9\n"
"  4  3    -309.4     199.8        5.4        3.7\n"
"  4  4      47.9    -350.1       -5.5       -5.6\n"
"  5  0    -234.4       0.0       -0.3        0.0\n"
"  5  1     363.1      47.7        0.6        0.1\n"
"  5  2     187.8     208.4       -0.7        2.5\n"
"  5  3    -140.7    -121.3        0.1       -0.9\n"
"  5  4    -151.2      32.2        1.2        3.0\n"
"  5  5      13.7      99.1        1.0        0.5\n"
"  6  0      65.9       0.0       -0.6        0.0\n"
"  6  1      65.6     -19.1       -0.4        0.1\n"
"  6  2      73.0      25.0        0.5       -1.8\n"
"  6  3    -121.5      52.7        1.4       -1.4\n"
"  6  4     -36.2     -64.4       -1.4        0.9\n"
"  6  5      13.5       9.0       -0.0        0.1\n"
"  6  6     -64.7      68.1        0.8        1.0\n"
"  7  0      80.6       0.0       -0.1        0.0\n"
"  7  1     -76.8     -51.4       -0.3        0.5\n"
"  7  2      -8.3     -16.8       -0.1        0.6\n"
"  7  3      56.5       2.3        0.7       -0.7\n"
"  7  4      15.8      23.5        0.2       -0.2\n"
"  7  5       6.4      -2.2       -0.5       -1.2\n"
"  7  6      -7.2     -27.2       -0.8        0.2\n"
"  7  7       9.8      -1.9        1.0        0.3\n"
"  8  0      23.6       0.0       -0.1        0.0\n"
"  8  1       9.8       8.4        0.1       -0.3\n"
"  8  2     -17.5     -15.3       -0.1        0.7\n"
"  8  3      -0.4      12.8        0.5       -0.2\n"
"  8  4     -21.1     -11.8       -0.1        0.5\n"
"  8  5      15.3      14.9        0.4       -0.3\n"
"  8  6      13.7       3.6        0.5       -0.5\n"
"  8  7     -16.5      -6.9        0.0        0.4\n"
"  8  8      -0.3       2.8        0.4        0.1\n"
"  9  0       5.0       0.0       -0.1        0.0\n"
"  9  1       8.2     -23.3       -0.2       -0.3\n"
"  9  2       2.9      11.1       -0.0        0.2\n"
"  9  3      -1.4       9.8        0.4       -0.4\n"
"  9  4      -1.1      -5.1       -0.3        0.4\n"
"  9  5     -13.3      -6.2       -0.0        0.1\n"
"  9  6       1.1       7.8        0.3       -0.0\n"
"  9  7       8.9       0.4       -0.0       -0.2\n"
"  9  8      -9.3      -1.5       -0.0        0.5\n"
"  9  9     -11.9       9.7       -0.4        0.2\n"
" 10  0      -1.9       0.0        0.0        0.0\n"
" 10  1      -6.2       3.4       -0.0       -0.0\n"
" 10  2      -0.1      -0.2       -0.0        0.1\n"
" 10  3       1.7       3.5        0.2       -0.3\n"
" 10  4      -0.9       4.8       -0.1        0.1\n"
" 10  5       0.6      -8.6       -0.2       -0.2\n"
" 10  6      -0.9      -0.1       -0.0        0.1\n"
" 10  7       1.9      -4.2       -0.1       -0.0\n"
" 10  8       1.4      -3.4       -0.2       -0.1\n"
" 10  9      -2.4      -0.1       -0.1        0.2\n"
" 10 10      -3.9      -8.8       -0.0       -0.0\n"
" 11  0       3.0       0.0       -0.0        0.0\n"
" 11  1      -1.4      -0.0       -0.1       -0.0\n"
" 11  2      -2.5       2.6       -0.0        0.1\n"
" 11  3       2.4      -0.5        0.0        0.0\n"
" 11  4      -0.9      -0.4       -0.0        0.2\n"
" 11  5       0.3       0.6       -0.1       -0.0\n"
" 11  6      -0.7      -0.2        0.0        0.0\n"
" 11  7      -0.1      -1.7       -0.0        0.1\n"
" 11  8       1.4      -1.6       -0.1       -0.0\n"
" 11  9      -0.6      -3.0       -0.1       -0.1\n"
" 11 10       0.2      -2.0       -0.1        0.0\n"
" 11 11       3.1      -2.6       -0.1       -0.0\n"
" 12  0      -2.0       0.0        0.0        0.0\n"
" 12  1      -0.1      -1.2       -0.0       -0.0\n"
" 12  2       0.5       0.5       -0.0        0.0\n"
" 12  3       1.3       1.3        0.0       -0.1\n"
" 12  4      -1.2      -1.8       -0.0        0.1\n"
" 12  5       0.7       0.1       -0.0       -0.0\n"
" 12  6       0.3       0.7        0.0        0.0\n"
" 12  7       0.5      -0.1       -0.0       -0.0\n"
" 12  8      -0.2       0.6        0.0        0.1\n"
" 12  9      -0.5       0.2       -0.0       -0.0\n"
" 12 10       0.1      -0.9       -0.0       -0.0\n"
" 12 11      -1.1      -0.0       -0.0        0.0\n"
" 12 12      -0.3       0.5       -0.1       -0.1\n"
"999999999999999999999999999999999999999999999999\n"
"999999999999999999999999999999999999999999999999";





// Base model epoch year
const double WMM_EPOCH_YEAR = 2025.0;

// Global coefficient arrays
double g[NMAX+1][NMAX+1] = {0};
double h[NMAX+1][NMAX+1] = {0};
double g_sv[NMAX+1][NMAX+1] = {0};
double h_sv[NMAX+1][NMAX+1] = {0};

// ===================================================================================== HELPER FUNCTIONS
int parse_wmm_line_manual(const char* line, int* n, int* m,
                          double* g, double* h,
                          double* g_sv, double* h_sv) {
    char field_buf[32];
    int field_index = 0;
    int buf_index = 0;
    int i = 0;

    if (strncmp(line, "9999", 4) == 0) {
        return 0;
    }

    for (;;) {
        char c = line[i];
        if (c == '\0' || c == '\n') {
            if (buf_index > 0) {
                field_buf[buf_index] = '\0';
                switch (field_index) {
                    case 0: *n = atoi(field_buf); break;
                    case 1: *m = atoi(field_buf); break;
                    case 2: *g = atof(field_buf); break;
                    case 3: *h = atof(field_buf); break;
                    case 4: *g_sv = atof(field_buf); break;
                    case 5: *h_sv = atof(field_buf); break;
                    default: break;
                }
                field_index++;
            }
            break;
        } else if (isspace(c)) {
            if (buf_index > 0) {
                field_buf[buf_index] = '\0';
                switch (field_index) {
                    case 0: *n = atoi(field_buf); break;
                    case 1: *m = atoi(field_buf); break;
                    case 2: *g = atof(field_buf); break;
                    case 3: *h = atof(field_buf); break;
                    case 4: *g_sv = atof(field_buf); break;
                    case 5: *h_sv = atof(field_buf); break;
                    default: break;
                }
                field_index++;
                buf_index = 0;
            }
        } else {
            if (buf_index < (int)(sizeof(field_buf) - 1)) {
                field_buf[buf_index] = c;
                buf_index++;
            } else {
                return 0;
            }
        }
        i++;
    }

    return (field_index == 6);
}

// All WMM parsing and output happens here to prepare for modularization
void magnetic_declination_init() {
    int line_num = 0;
    const char* p = wmm_cof_2020_content;
    char line[128];

    while (*p) {
        int i = 0;
        while (*p && *p != '\n' && i < 127) {
            line[i] = *p;
            i++;
            p++;
        }
        line[i] = '\0';

        if (*p == '\n') {
            p++;
        }

        // Optional: print each line for debugging
        printf("Line %d: \"%s\"\n", line_num, line);

        if (line_num != 0) { // Skip header
            int n, m;
            double g_value, h_value, g_sv_value, h_sv_value;
            if (parse_wmm_line_manual(line, &n, &m, &g_value, &h_value, &g_sv_value, &h_sv_value)) {
                if (n <= NMAX && m <= n) {
                    g[n][m] = g_value;
                    h[n][m] = h_value;
                    g_sv[n][m] = g_sv_value;
                    h_sv[n][m] = h_sv_value;
                }
            }
        }
        line_num++;
    }

    printf("line_num %d\n", line_num);

    // Print parsed data
    printf("\nn m g h g_sv h_sv\n");
    for (int n = 1; n <= NMAX; n++) {
        for (int m = 0; m <= n; m++) {
            printf("%2d %2d %10.1f %10.1f %7.1f %7.1f\n", n, m, g[n][m], h[n][m], g_sv[n][m], h_sv[n][m]);
        }
    }
}




/** Return the position in International Terrestrial Reference System coordinates, units meters.
Using the WGS 84 ellipsoid and the algorithm from https://geographiclib.sourceforge.io/
 INPUT:
    lat: Geodetic latitude in degrees, -90 at the south pole, 90 at the north pole.
    lon: Geodetic longitude in degrees.
    h: Height above the WGS 84 ellipsoid in meters.
**/
void geodetic2ecef(float lat, float lon, float h, float* data){
    // Convert to radians
    float phi = lat*((float)(M_PI/180.0));
    float lam = lon*((float)(M_PI/180.0));
    // WGS 84 constants
    const float a = 6378137;
    // const float f = 1.0/298.257223563;
    const float e2 = 0.0066943799901413165;//f*(2-f);
    const float e2m = 0.9933056200098587;//(1-f)*(1-f);
    float sphi = sinf(phi);
    float cphi = cosf(phi);
    float slam = sinf(lam);
    float clam = cosf(lam);
    float n = a/sqrtf(1.0f - e2*(sphi*sphi));
    float z = (e2m*n + h) * sphi;
    float r = (n + h) * cphi;

    data[0] = r*clam;
    data[1] = r*slam;
    data[2] = z;
}

enum AngPosUnit {
  DEG,  // Degrees
  RAD,  // Radians
  REV   // Revolutions
};



float g_year_comp(uint8_t n, uint8_t m, float two_digit_year){
    return g[n][m]+((2000+two_digit_year)-WMM_EPOCH_YEAR)*g_sv[n][m];
}

float h_year_comp(uint8_t n, uint8_t m, float two_digit_year){
    return h[n][m]+((2000+two_digit_year)-WMM_EPOCH_YEAR)*h_sv[n][m];
}


// Inspired by https://github.com/bolderflight/wmm
void GeoMag(float two_digit_year, float*  position_itrs, float* output){
    float x= position_itrs[0];
    float y= position_itrs[1];
    float z= position_itrs[2];
    float px= 0;
    float py= 0;
    float pz= 0;
    float rsqrd= x*x+y*y+z*z;
    float temp= WMM_EARTH_RADIUS_M/rsqrd;
    float a= x*temp;
    float b= y*temp;
    float f= z*temp;
    float g= WMM_EARTH_RADIUS_M*temp;

    int n,m;
    //first m==0 row, just solve for the Vs
    float Vtop= WMM_EARTH_RADIUS_M/sqrtf(rsqrd);//V0,0
    float Wtop= 0;//W0,0
    float Vprev= 0;
    float Wprev= 0;
    float Vnm= Vtop;
    float Wnm= Wtop;

    //iterate through all ms
    for ( m = 0; m <= NMAX+1; m++)
    {
        // iterate through all ns
        for (n = m; n <= NMAX+1; n++)
        {
            if (n==m){
                if(m!=0){
                    temp= Vtop;
                    Vtop= (2*m-1)*(a*Vtop-b*Wtop);
                    Wtop= (2*m-1)*(a*Wtop+b*temp);
                    Vprev= 0;
                    Wprev= 0;
                    Vnm= Vtop;
                    Wnm= Wtop;
                }
            }
            else{
                temp= Vnm;
                float invs_temp=1.0f/((float)(n-m));
                Vnm= ((2*n-1)*f*Vnm - (n+m-1)*g*Vprev)*invs_temp;
                Vprev= temp;
                temp= Wnm;
                Wnm= ((2*n-1)*f*Wnm - (n+m-1)*g*Wprev)*invs_temp;
                Wprev= temp;
            }
            if (m<NMAX && n>=m+2){
                px+= 0.5f*(n-m)*(n-m-1)*(g_year_comp(n-1,m+1,two_digit_year)*Vnm+h_year_comp(n-1,m+1,two_digit_year)*Wnm);
                py+= 0.5f*(n-m)*(n-m-1)*(-g_year_comp(n-1,m+1,two_digit_year)*Wnm+h_year_comp(n-1,m+1,two_digit_year)*Vnm);
            }
            if (n>=2 && m>=2){
                px+= 0.5f*(-g_year_comp(n-1,m-1,two_digit_year)*Vnm-h_year_comp(n-1,m-1,two_digit_year)*Wnm);
                py+= 0.5f*(-g_year_comp(n-1,m-1,two_digit_year)*Wnm+h_year_comp(n-1,m-1,two_digit_year)*Vnm);
            }
            if (m==1 && n>=2){
                px+= -g_year_comp(n-1,0,two_digit_year)*Vnm;
                py+= -g_year_comp(n-1,0,two_digit_year)*Wnm;
            }
            if (n>=2 && n>m){
                pz+= (n-m)*(-g_year_comp(n-1,m,two_digit_year)*Vnm-h_year_comp(n-1,m,two_digit_year)*Wnm);
            }
        }
    }

    output[0] = -px*1.0E-9f;
    output[1] = -py*1.0E-9f;
    output[2] = -pz*1E-9f;
}

void magField2Elements(float* mag_field_itrs, float lat, float lon, float *output){
    float x = mag_field_itrs[0]*1E9f;
    float y = mag_field_itrs[1]*1E9f;
    float z = mag_field_itrs[2]*1E9f;
    float phi = lat*((float)(M_PI/180.0));
    float lam = lon*((float)(M_PI/180.0));
    float sphi = sinf(phi);
    float cphi = cosf(phi);
    float slam = sinf(lam);
    float clam = cosf(lam);
    float x1 = clam*x + slam*y;
    float north = -sphi*x1 + cphi*z;
    float east = -slam*x + clam*y;
    float down = -cphi*x1 + -sphi*z;
    float horizontal = sqrtf(north*north + east*east);
    float total = sqrtf(horizontal*horizontal + down*down);
    float inclination = atan2f(down, horizontal)*((float)(180.0/M_PI));
    float declination = atan2f(east, north)*((float)(180.0/M_PI));

    output[0] = north;
    output[1] = east;
    output[2] = down;
    output[3] = horizontal;
    output[4] = total;
    output[5] = inclination;
    output[6] = declination;
}

int main() {
    double altitude_kilometers_above_sea_level = 0.004;
    // double latitude =  55.651328;
    // double longitude = 12.574926;

    double latitude =  55.64841;
    double longitude = 12.56699;

    float latitude_f =  55.64841;
    float longitude_f = 12.56699;


    // double latitude =  54.43597;
    // double longitude = 48.86719;
    
    // float latitude_f =  54.43597;
    // float longitude_f = 48.86719;

    uint8_t day = 17;
    uint8_t month = 9;
    uint8_t year = 22;

    float geoid_altitude_meters = 1.3;
    float position[] = {0.0, 0.0, 0.0};

    magnetic_declination_init();


    geodetic2ecef(
        latitude_f, 
        longitude_f, 
        geoid_altitude_meters, 
        position
    );
    printf("x  %f, y %f, z %f \n", 
        position[0], 
        position[1], 
        position[2] 
    );


    float mag_field[] = {0.0, 0.0, 0.0};
    GeoMag((float)year, position, mag_field);

    printf("mag_field x  %f, mag_field y %f, mag_field z %f \n", 
        mag_field[0], 
        mag_field[1], 
        mag_field[2] 
    );

    float data[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    magField2Elements(mag_field, latitude_f, longitude_f, data);

    printf("North %f, East %f, Down %f, Horizontal %f, Total %f, inclination %f, declination %f \n", 
        data[0], 
        data[1], 
        data[2], 
        data[3], 
        data[4], 
        data[5],
        data[6]
    );

    return 0;
}



// Can you try to replicate the exact code i have in python and see what the calculation gives there?

// Skip the parsing of my WMM.COF and use the ones i have printed out here:

// "n m g h g_sv h_sv
//  1  0   -29351.8        0.0    12.0     0.0
//  1  1    -1410.8     4545.4     9.7   -21.5
//  2  0    -2556.6        0.0   -11.6     0.0
//  2  1     2951.1    -3133.6    -5.2   -27.7
//  2  2     1649.3     -815.1    -8.0   -12.1
//  3  0     1361.0        0.0    -1.3     0.0
//  3  1    -2404.1      -56.6    -4.2     4.0
//  3  2     1243.8      237.5     0.4    -0.3
//  3  3      453.6     -549.5   -15.6    -4.1
//  4  0      895.0        0.0    -1.6     0.0
//  4  1      799.5      278.6    -2.4    -1.1
//  4  2       55.7     -133.9    -6.0     4.1
//  4  3     -281.1      212.0     5.6     1.6
//  4  4       12.1     -375.6    -7.0    -4.4
//  5  0     -233.2        0.0     0.6     0.0
//  5  1      368.9       45.4     1.4    -0.5
//  5  2      187.2      220.2     0.0     2.2
//  5  3     -138.7     -122.9     0.6     0.4
//  5  4     -142.0       43.0     2.2     1.7
//  5  5       20.9      106.1     0.9     1.9
//  6  0       64.4        0.0    -0.2     0.0
//  6  1       63.8      -18.4    -0.4     0.3
//  6  2       76.9       16.8     0.9    -1.6
//  6  3     -115.7       48.8     1.2    -0.4
//  6  4      -40.9      -59.8    -0.9     0.9
//  6  5       14.9       10.9     0.3     0.7
//  6  6      -60.7       72.7     0.9     0.9
//  7  0       79.5        0.0    -0.0     0.0
//  7  1      -77.0      -48.9    -0.1     0.6
//  7  2       -8.8      -14.4    -0.1     0.5
//  7  3       59.3       -1.0     0.5    -0.8
//  7  4       15.8       23.4    -0.1     0.0
//  7  5        2.5       -7.4    -0.8    -1.0
//  7  6      -11.1      -25.1    -0.8     0.6
//  7  7       14.2       -2.3     0.8    -0.2
//  8  0       23.2        0.0    -0.1     0.0
//  8  1       10.8        7.1     0.2    -0.2
//  8  2      -17.5      -12.6     0.0     0.5
//  8  3        2.0       11.4     0.5    -0.4
//  8  4      -21.7       -9.7    -0.1     0.4
//  8  5       16.9       12.7     0.3    -0.5
//  8  6       15.0        0.7     0.2    -0.6
//  8  7      -16.8       -5.2    -0.0     0.3
//  8  8        0.9        3.9     0.2     0.2
//  9  0        4.6        0.0    -0.0     0.0
//  9  1        7.8      -24.8    -0.1    -0.3
//  9  2        3.0       12.2     0.1     0.3
//  9  3       -0.2        8.3     0.3    -0.3
//  9  4       -2.5       -3.3    -0.3     0.3
//  9  5      -13.1       -5.2     0.0     0.2
//  9  6        2.4        7.2     0.3    -0.1
//  9  7        8.6       -0.6    -0.1    -0.2
//  9  8       -8.7        0.8     0.1     0.4
//  9  9      -12.9       10.0    -0.1     0.1
// 10  0       -1.3        0.0     0.1     0.0
// 10  1       -6.4        3.3     0.0     0.0
// 10  2        0.2        0.0     0.1    -0.0
// 10  3        2.0        2.4     0.1    -0.2
// 10  4       -1.0        5.3    -0.0     0.1
// 10  5       -0.6       -9.1    -0.3    -0.1
// 10  6       -0.9        0.4     0.0     0.1
// 10  7        1.5       -4.2    -0.1     0.0
// 10  8        0.9       -3.8    -0.1    -0.1
// 10  9       -2.7        0.9    -0.0     0.2
// 10 10       -3.9       -9.1    -0.0    -0.0
// 11  0        2.9        0.0     0.0     0.0
// 11  1       -1.5        0.0    -0.0    -0.0
// 11  2       -2.5        2.9     0.0     0.1
// 11  3        2.4       -0.6     0.0    -0.0
// 11  4       -0.6        0.2     0.0     0.1
// 11  5       -0.1        0.5    -0.1    -0.0
// 11  6       -0.6       -0.3     0.0    -0.0
// 11  7       -0.1       -1.2    -0.0     0.1
// 11  8        1.1       -1.7    -0.1    -0.0
// 11  9       -1.0       -2.9    -0.1     0.0
// 11 10       -0.2       -1.8    -0.1     0.0
// 11 11        2.6       -2.3    -0.1     0.0
// 12  0       -2.0        0.0     0.0     0.0
// 12  1       -0.2       -1.3     0.0    -0.0
// 12  2        0.3        0.7    -0.0     0.0
// 12  3        1.2        1.0    -0.0    -0.1
// 12  4       -1.3       -1.4    -0.0     0.1
// 12  5        0.6       -0.0    -0.0    -0.0
// 12  6        0.6        0.6     0.1    -0.0
// 12  7        0.5       -0.1    -0.0    -0.0
// 12  8       -0.1        0.8     0.0     0.0
// 12  9       -0.4        0.1     0.0    -0.0
// 12 10       -0.2       -1.0    -0.1    -0.0
// 12 11       -1.3        0.1    -0.0     0.0
// 12 12       -0.7        0.2    -0.1    -0.1"