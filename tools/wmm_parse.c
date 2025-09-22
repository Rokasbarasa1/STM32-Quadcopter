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

#define NMAX 12
#define WMM_EARTH_RADIUS_KM 6371.2



// The full WMM.COF string, shortened here with "..."
const char* wmm_cof_content = 
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

// Base model epoch year
const double WMM_EPOCH_YEAR = 2025.0;

// Global coefficient arrays
double g[NMAX+1][NMAX+1] = {0};
double h[NMAX+1][NMAX+1] = {0};
double g_sv[NMAX+1][NMAX+1] = {0};
double h_sv[NMAX+1][NMAX+1] = {0};

// Arrays to store propagated coefficients at the requested year
double g_prop[NMAX+1][NMAX+1];
double h_prop[NMAX+1][NMAX+1];

// Legendre polynomials storage
double P[NMAX + 1][NMAX + 1];
double dP[NMAX + 1][NMAX + 1];

// Schmidt semi-normalization factors
static double snorm[NMAX+1][NMAX+1];
// ===================================================================================== DATE CALCULATIONS
// Returns 1 if leap year, 0 otherwise
static int is_leap_year(uint16_t year_two_digit) {
    uint16_t full_year = 2000 + year_two_digit;
    return ((full_year % 4 == 0) && (full_year % 100 != 0)) || (full_year % 400 == 0);
}

// Days in each month (1-based) for non-leap year
static const int days_in_month[12] = {
    31, // January
    28, // February
    31, // March
    30, // April
    31, // May
    30, // June
    31, // July
    31, // August
    30, // September
    31, // October
    30, // November
    31  // December
};

// Returns number of days since January 1st for given day/month/year
// Example: Jan 1 -> 0, Jan 2 -> 1, Feb 1 -> 31, Mar 1 (non-leap) -> 59, Mar 1 (leap) -> 60
uint16_t days_since_jan_1(uint8_t day, uint8_t month, uint16_t year_two_digit) {
    uint16_t days = 0;
    for (int m = 1; m < month; m++) { // months before input month
        days += days_in_month[m-1];
        if (m == 2 && is_leap_year(year_two_digit)) {
            days += 1; // Add leap day for February
        }
    }
    days += (day - 1); // days into current month, January 1 = 0
    return days;
}

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


// Compute Schmidt quasi-normalized associated Legendre polynomials and derivatives
// Function to compute factorial (helper for normalization)
double factorial(int n) {
    double f = 1.0;
    for (int i = 2; i <= n; i++) {
        f *= i;
    }
    return f;
}

void compute_snorm() {
    // Precomputed Schmidt semi-normalization factors for n <= 12
    const double snorm_table[13][13] = {
        {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.6666666666666666, 0.4714045207910317, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.5773502691896257, 0.4841229182759272, 0.3779644730092272, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.5477225575051661, 0.4714045207910317, 0.39083795463276156, 0.31622776601683794, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.5345224838248473, 0.4643482420147146, 0.4009068100861693, 0.34641016151377544, 0.2958039891549808, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.5270462766947299, 0.45970084315207784, 0.3971486609760961, 0.34996703859505326, 0.30901699437494745, 0.27386127875258306, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.5222279680951035, 0.45643546458763424, 0.3947640298876548, 0.34792027381505496, 0.3073593257540631, 0.2725276761659259, 0.24253562503633297, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.5189160772039032, 0.45415400597152074, 0.3931226720573634, 0.34641016151377544, 0.3055050463303884, 0.2692582403567252, 0.23866258673137812, 0.21516744691677203, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.5165289256003831, 0.4525424593239109, 0.3919452804546556, 0.3454258557433203, 0.3046223251403624, 0.2685507495348089, 0.23790255351067578, 0.21081851064701122, 0.18944271909999156, 0.0, 0.0, 0.0},
        {1.0, 0.5147624902073091, 0.45138266522971415, 0.3911185213105995, 0.3447329709512935, 0.3039277958169263, 0.2680172632726777, 0.2373759412939391, 0.21036567508533248, 0.18819756368322394, 0.16814097252700298, 0.0, 0.0},
        {1.0, 0.5134010567667639, 0.4505309449221994, 0.39052919322013856, 0.3442347678084078, 0.3033806882640979, 0.26759302174301076, 0.23699502369896412, 0.21003656750853324, 0.18759566502541563, 0.1672955945342059, 0.14955852179582952, 0.0},
        {1.0, 0.5123379506265175, 0.4498944006562308, 0.3900936704690008, 0.3438666304751147, 0.3029445950858425, 0.2672494274546046, 0.23669002369896412, 0.20967501954801078, 0.1871461307218063, 0.16677835153540068, 0.1488027525933121, 0.13301519735346032},
        {1.0, 0.5114972484030016, 0.4493968853400536, 0.3897688259913605, 0.3435840720483989, 0.3025939926467205, 0.2670000016322036, 0.23644644217165048, 0.20940722540765228, 0.18676607010558162, 0.16639471603096087, 0.1484222368350174, 0.13262911924611136},
        {1.0, 0.5108271466146824, 0.4490025690315454, 0.3895193235474291, 0.3433621198099262, 0.30230623892757424, 0.2668033840209231, 0.2362472266705659, 0.2091775441967555, 0.18644414755049108, 0.1660696614950925, 0.1480492368350174, 0.13224547386055442, 0.11838515903606502},
        {1.0, 0.5102896413624375, 0.4486892455704255, 0.3893251701043261, 0.3431811610262184, 0.30206366037135824, 0.26664356500408207, 0.2360800037450506, 0.20900722540765228, 0.1861674702103202, 0.1657926074283069, 0.14772565966692545, 0.13190626331250094, 0.11799107653297765},
        {1.0, 0.5098578361002305, 0.4484396163384085, 0.3891702616156707, 0.3430331192170141, 0.3018531281590489, 0.26652356500408207, 0.23594644217165048, 0.20887501954801078, 0.18592603723518003, 0.16555455239684002, 0.14744349888206208, 0.1316065626177978, 0.11762952383615356}
    };
    for (int n = 0; n <= NMAX; n++) {
        for (int m = 0; m <= n; m++) {
            snorm[n][m] = snorm_table[n][m];
        }
    }
}

// Compute Schmidt semi-normalized associated Legendre polynomials and derivatives
// Input: x = cos(colatitude)
// Compute Schmidt semi-normalized associated Legendre polynomials and derivatives
void compute_legendre(double x) {
    double sin_theta = sqrt(1.0 - x*x);
    if (sin_theta < 1e-14) sin_theta = 1e-14;

    // Initialize
    for (int n = 0; n <= NMAX; n++) {
        for (int m = 0; m <= n; m++) {
            P[n][m] = 0.0;
            dP[n][m] = 0.0;
        }
    }

    P[0][0] = 1.0;
    dP[0][0] = 0.0;

    for (int n = 1; n <= NMAX; n++) {
        for (int m = 0; m <= n; m++) {
            if (n == m) {
                P[n][m] = sin_theta * P[n-1][m-1];
                dP[n][m] = sin_theta * dP[n-1][m-1] + cos(acos(x)) * P[n-1][m-1];
            } else if (m == n-1) {
                P[n][m] = x * P[n-1][m];
                dP[n][m] = x * dP[n-1][m] - sin_theta * P[n-1][m];
            } else {
                P[n][m] = ((2*n-1)*x*P[n-1][m] - (n+m-1)*P[n-2][m])/(n-m);
                dP[n][m] = ((2*n-1)*(x*dP[n-1][m]-sin_theta*P[n-1][m])-(n+m-1)*dP[n-2][m])/(n-m);
            }
            P[n][m] *= snorm[n][m];       // Apply Schmidt semi-normalization
            dP[n][m] *= snorm[n][m];      // Apply same scaling for derivative
        }
    }
}





// ===================================================================================== ACTUAL FUNCTIONS

// All WMM parsing and output happens here to prepare for modularization
void magnetic_declination_init() {
    int line_num = 0;
    const char* p = wmm_cof_content;
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
        // printf("Line %d: \"%s\"\n", line_num, line);

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

    // Print parsed data
    printf("\nn m g h g_sv h_sv\n");
    for (int n = 1; n <= NMAX; n++) {
        for (int m = 0; m <= n; m++) {
            printf("%2d %2d %10.1f %10.1f %7.1f %7.1f\n", n, m, g[n][m], h[n][m], g_sv[n][m], h_sv[n][m]);
        }
    }
}

void magnetic_declination_propagate_coefficients_based_on_year(uint8_t day, uint8_t month, uint8_t year_two_digit){
    // double year = 2000.0 + ((double)year_two_digit + ((double)days_since_jan_1(day, month, year_two_digit)/365.0));  // Convert to full year (e.g., 25 -> 2025)
    // double year = 2000.0 + (double)year_two_digit ;  // Convert to full year (e.g., 25 -> 2025)
    double year = 2000.0 + (double)year_two_digit 
              + ((double)days_since_jan_1(day, month, year_two_digit) /
                 (is_leap_year(year_two_digit) ? 366.0 : 365.0));

    double delta_year = year - WMM_EPOCH_YEAR;


    for (int n = 1; n <= NMAX; n++) {
        for (int m = 0; m <= n; m++) {
            // linear temporal propagation only
            g_prop[n][m] = g[n][m] + delta_year * g_sv[n][m];
            h_prop[n][m] = h[n][m] + delta_year * h_sv[n][m];
        }
    }

}

double magnetic_declination_get_declination(double latitude, double longitude, double altitude_km) {
    double colat = (90.0 - latitude) * M_DEG_TO_RAD;
    double lon = longitude * M_DEG_TO_RAD;

    double cos_colat = cos(colat);
    double sin_colat = sin(colat);
    if (sin_colat < 1e-14) sin_colat = 1e-14;

    double r = WMM_EARTH_RADIUS_KM + altitude_km;
    double a_over_r = WMM_EARTH_RADIUS_KM / r;

    compute_legendre(cos_colat);

    double Br=0.0, Btheta=0.0, Bphi=0.0;

    for (int n=1; n<=NMAX; n++) {
        double a_r_pow = pow(a_over_r, n+2);
        for (int m=0; m<=n; m++) {
            double gnm = g_prop[n][m];
            double hnm = h_prop[n][m];
            double cos_m_lon = cos(m*lon);
            double sin_m_lon = sin(m*lon);

            double tmp = gnm*cos_m_lon + hnm*sin_m_lon;

            Br    += a_r_pow*(n+1)*tmp*P[n][m];
            Btheta -= a_r_pow*tmp*dP[n][m]; // southward
            if (m>0) {
                Bphi -= a_r_pow*m*(gnm*sin_m_lon - hnm*cos_m_lon)*P[n][m]/sin_colat; // eastward
            }
        }
    }

    // Convert to NED
    double X = -Btheta; // north
    double Y = Bphi;    // east
    double Z = -Br;     // down

    double declination_rad = atan2(Y, X);
    double declination_deg = declination_rad * M_RAD_TO_DEG;

    if (declination_deg > 180.0) declination_deg -= 360.0;
    if (declination_deg < -180.0) declination_deg += 360.0;

    return declination_deg;
}

int main() {
    double altitude_kilometers_above_sea_level = 0.004;
    // double latitude =  55.651328;
    // double longitude = 12.574926;

    // double latitude =  55.64841;
    // double longitude = 12.56699;

    
    double latitude =  54.43597;
    double longitude = 48.86719;
    


    uint8_t day = 17;
    uint8_t month = 9;
    uint8_t year = 25;

    magnetic_declination_init();
    compute_snorm();
    magnetic_declination_propagate_coefficients_based_on_year(day, month, year);

    double declination = magnetic_declination_get_declination(latitude, longitude, altitude_kilometers_above_sea_level);
    printf("declination %f\n", declination);
    // Later, you will only call this from other files or modules.
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