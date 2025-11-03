#include <stdio.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>

#ifndef TRUE
#define TRUE            ((int)1)
#endif
#ifndef FALSE
#define FALSE           ((int)0)
#endif

#define MAG_PS_MIN_LAT_DEGREE  -55 /* Minimum Latitude for  Polar Stereographic projection in degrees   */
#define MAG_PS_MAX_LAT_DEGREE  55  /* Maximum Latitude for Polar Stereographic projection in degrees     */
#define MAG_UTM_MIN_LAT_DEGREE -80.5  /* Minimum Latitude for UTM projection in degrees   */
#define MAG_UTM_MAX_LAT_DEGREE  84.5  /* Maximum Latitude for UTM projection in degrees     */
#define ATanH(x)	    (0.5f * logf((1 + x) / (1 - x)))
#define ATanH(x)	    (0.5f * logf((1 + x) / (1 - x)))

#define WMM_UNCERTAINTY_F 138
#define WMM_UNCERTAINTY_H 133
#define WMM_UNCERTAINTY_X 137
#define WMM_UNCERTAINTY_Y 89
#define WMM_UNCERTAINTY_Z 141
#define WMM_UNCERTAINTY_I 0.20
#define WMM_UNCERTAINTY_D_OFFSET 0.26
#define WMM_UNCERTAINTY_D_COEF 5417

#define MAG_USE_GEOID	1    /* 1 Geoid - Ellipsoid difference should be corrected, 0 otherwise */

#define M_DEG_TO_RAD 0.0174533f
#define M_RAD_TO_DEG 57.29578f
#ifndef M_PI
#define M_PI 3.1415927f
#endif
#define NMAX 12
#define NMAXSV 12

#define WMM_EARTH_RADIUS_KM 6371.2
#define WMM_EARTH_RADIUS_M 6371200.0f

#define DEG2RAD(deg) ((deg) * M_PI / 180.0f)
#define RAD2DEG(rad) ((rad) * 180.0f / M_PI)


/// @brief replace this with the WMM.COF that you want to use
const char* wmm_cof_2025_content = 
"    2025.0            WMM-2025     11/13/2024\n"
"  1  0  -29351.8       0.0f       12.0        0.0f\n"
"  1  1   -1410.8    4545.4        9.7      -21.5\n"
"  2  0   -2556.6       0.0f      -11.6        0.0f\n"
"  2  1    2951.1   -3133.6       -5.2      -27.7\n"
"  2  2    1649.3    -815.1       -8.0      -12.1\n"
"  3  0    1361.0       0.0f       -1.3        0.0f\n"
"  3  1   -2404.1     -56.6       -4.2        4.0\n"
"  3  2    1243.8     237.5        0.4       -0.3\n"
"  3  3     453.6    -549.5      -15.6       -4.1\n"
"  4  0     895.0       0.0f       -1.6        0.0f\n"
"  4  1     799.5     278.6       -2.4       -1.1\n"
"  4  2      55.7    -133.9       -6.0        4.1\n"
"  4  3    -281.1     212.0        5.6        1.6\n"
"  4  4      12.1    -375.6       -7.0       -4.4\n"
"  5  0    -233.2       0.0f        0.6        0.0f\n"
"  5  1     368.9      45.4        1.4       -0.5\n"
"  5  2     187.2     220.2        0.0f        2.2\n"
"  5  3    -138.7    -122.9        0.6        0.4\n"
"  5  4    -142.0      43.0        2.2        1.7\n"
"  5  5      20.9     106.1        0.9        1.9\n"
"  6  0      64.4       0.0f       -0.2        0.0f\n"
"  6  1      63.8     -18.4       -0.4        0.3\n"
"  6  2      76.9      16.8        0.9       -1.6\n"
"  6  3    -115.7      48.8        1.2       -0.4\n"
"  6  4     -40.9     -59.8       -0.9        0.9\n"
"  6  5      14.9      10.9        0.3        0.7\n"
"  6  6     -60.7      72.7        0.9        0.9\n"
"  7  0      79.5       0.0f       -0.0f        0.0f\n"
"  7  1     -77.0     -48.9       -0.1        0.6\n"
"  7  2      -8.8     -14.4       -0.1        0.5\n"
"  7  3      59.3      -1.0f        0.5       -0.8\n"
"  7  4      15.8      23.4       -0.1        0.0f\n"
"  7  5       2.5      -7.4       -0.8       -1.0f\n"
"  7  6     -11.1     -25.1       -0.8        0.6\n"
"  7  7      14.2      -2.3        0.8       -0.2\n"
"  8  0      23.2       0.0f       -0.1        0.0f\n"
"  8  1      10.8       7.1        0.2       -0.2\n"
"  8  2     -17.5     -12.6        0.0f        0.5\n"
"  8  3       2.0      11.4        0.5       -0.4\n"
"  8  4     -21.7      -9.7       -0.1        0.4\n"
"  8  5      16.9      12.7        0.3       -0.5\n"
"  8  6      15.0       0.7        0.2       -0.6\n"
"  8  7     -16.8      -5.2       -0.0f        0.3\n"
"  8  8       0.9       3.9        0.2        0.2\n"
"  9  0       4.6       0.0f       -0.0f        0.0f\n"
"  9  1       7.8     -24.8       -0.1       -0.3\n"
"  9  2       3.0      12.2        0.1        0.3\n"
"  9  3      -0.2       8.3        0.3       -0.3\n"
"  9  4      -2.5      -3.3       -0.3        0.3\n"
"  9  5     -13.1      -5.2        0.0f        0.2\n"
"  9  6       2.4       7.2        0.3       -0.1\n"
"  9  7       8.6      -0.6       -0.1       -0.2\n"
"  9  8      -8.7       0.8        0.1        0.4\n"
"  9  9     -12.9      10.0       -0.1        0.1\n"
" 10  0      -1.3       0.0f        0.1        0.0f\n"
" 10  1      -6.4       3.3        0.0f        0.0f\n"
" 10  2       0.2       0.0f        0.1       -0.0f\n"
" 10  3       2.0       2.4        0.1       -0.2\n"
" 10  4      -1.0f       5.3       -0.0f        0.1\n"
" 10  5      -0.6      -9.1       -0.3       -0.1\n"
" 10  6      -0.9       0.4        0.0f        0.1\n"
" 10  7       1.5      -4.2       -0.1        0.0f\n"
" 10  8       0.9      -3.8       -0.1       -0.1\n"
" 10  9      -2.7       0.9       -0.0f        0.2\n"
" 10 10      -3.9      -9.1       -0.0f       -0.0f\n"
" 11  0       2.9       0.0f        0.0f        0.0f\n"
" 11  1      -1.5       0.0f       -0.0f       -0.0f\n"
" 11  2      -2.5       2.9        0.0f        0.1\n"
" 11  3       2.4      -0.6        0.0f       -0.0f\n"
" 11  4      -0.6       0.2        0.0f        0.1\n"
" 11  5      -0.1       0.5       -0.1       -0.0f\n"
" 11  6      -0.6      -0.3        0.0f       -0.0f\n"
" 11  7      -0.1      -1.2       -0.0f        0.1\n"
" 11  8       1.1      -1.7       -0.1       -0.0f\n"
" 11  9      -1.0f      -2.9       -0.1        0.0f\n"
" 11 10      -0.2      -1.8       -0.1        0.0f\n"
" 11 11       2.6      -2.3       -0.1        0.0f\n"
" 12  0      -2.0       0.0f        0.0f        0.0f\n"
" 12  1      -0.2      -1.3        0.0f       -0.0f\n"
" 12  2       0.3       0.7       -0.0f        0.0f\n"
" 12  3       1.2       1.0f       -0.0f       -0.1\n"
" 12  4      -1.3      -1.4       -0.0f        0.1\n"
" 12  5       0.6      -0.0f       -0.0f       -0.0f\n"
" 12  6       0.6       0.6        0.1       -0.0f\n"
" 12  7       0.5      -0.1       -0.0f       -0.0f\n"
" 12  8      -0.1       0.8        0.0f        0.0f\n"
" 12  9      -0.4       0.1        0.0f       -0.0f\n"
" 12 10      -0.2      -1.0f       -0.1       -0.0f\n"
" 12 11      -1.3       0.1       -0.0f        0.0f\n"
" 12 12      -0.7       0.2       -0.1       -0.1\n"
"999999999999999999999999999999999999999999999999\n"
"999999999999999999999999999999999999999999999999";

int NumTerms = ((NMAX + 1) * (NMAX + 2) / 2);

float wmm_epoch_year = 0.0f;      // Corresponds to: MagneticModel->epoch
char wmm_model_name[32] = {0};    // Corresponds to: MagneticModel->ModelName
char wmm_edition_date[32] = {0};  // Corresponds to: MagneticModel->EditionDate

typedef struct {
    float EditionDate;
    float epoch; /*Base time of Geomagnetic model epoch (yrs)*/
    float min_year;
    char ModelName[32];
    float *Main_Field_Coeff_G; /* C - Gauss coefficients of main geomagnetic model (nT) Index is (n * (n + 1) / 2 + m) */
    float *Main_Field_Coeff_H; /* C - Gauss coefficients of main geomagnetic model (nT) */
    float *Secular_Var_Coeff_G; /* CD - Gauss coefficients of secular geomagnetic model (nT/yr) */
    float *Secular_Var_Coeff_H; /* CD - Gauss coefficients of secular geomagnetic model (nT/yr) */
    int nMax; /* Maximum degree of spherical harmonic model */
    int nMaxSecVar; /* Maximum degree of spherical harmonic secular model */
    int SecularVariationUsed; /* Whether or not the magnetic secular variation vector will be needed by program*/
    float CoefficientFileEndDate; 
    
} MAGtype_MagneticModel;

typedef struct {
    float a; /*semi-major axis of the ellipsoid*/
    float b; /*semi-minor axis of the ellipsoid*/
    float fla; /* flattening */
    float epssq; /*first eccentricity squared */
    float eps; /* first eccentricity */
    float re; /* mean radius of  ellipsoid*/
} MAGtype_Ellipsoid;

typedef struct {
    int NumbGeoidCols; /* 360 degrees of longitude at 15 minute spacing */
    int NumbGeoidRows; /* 180 degrees of latitude  at 15 minute spacing */
    int NumbHeaderItems; /* min, max lat, min, max long, lat, long spacing*/
    int ScaleFactor; /* 4 grid cells per degree at 15 minute spacing  */
    float *GeoidHeightBuffer;
    int NumbGeoidElevs;
    int Geoid_Initialized; /* indicates successful initialization */
    int UseGeoid; /*Is the Geoid being used?*/
} MAGtype_Geoid;

typedef struct {
    float lambda; /* longitude*/
    float phig; /* geocentric latitude*/
    float r; /* distance from the center of the ellipsoid*/
} MAGtype_CoordSpherical;

typedef struct {
    float lambda; /* longitude */
    float phi; /* geodetic latitude */
    float HeightAboveEllipsoid; /* height above the ellipsoid (HaE) */
    float HeightAboveGeoid; /* (height above the EGM96 geoid model ) */
    int UseGeoid;
} MAGtype_CoordGeodetic;

typedef struct {
    int Year;
    int Month;
    int Day;
    float DecimalYear; /* decimal years */
} MAGtype_Date;

typedef struct {
    float Decl; /* 1. Angle between the magnetic field vector and true north, positive east*/
    float Incl; /*2. Angle between the magnetic field vector and the horizontal plane, positive down*/
    float F; /*3. Magnetic Field Strength*/
    float H; /*4. Horizontal Magnetic Field Strength*/
    float X; /*5. Northern component of the magnetic field vector*/
    float Y; /*6. Eastern component of the magnetic field vector*/
    float Z; /*7. Downward component of the magnetic field vector*/
    float GV; /*8. The Grid Variation*/
    float Decldot; /*9. Yearly Rate of change in declination*/
    float Incldot; /*10. Yearly Rate of change in inclination*/
    float Fdot; /*11. Yearly rate of change in Magnetic field strength*/
    float Hdot; /*12. Yearly rate of change in horizontal field strength*/
    float Xdot; /*13. Yearly rate of change in the northern component*/
    float Ydot; /*14. Yearly rate of change in the eastern component*/
    float Zdot; /*15. Yearly rate of change in the downward component*/
    float GVdot; /*16. Yearly rate of change in grid variation*/
} MAGtype_GeoMagneticElements;

typedef struct {
    float *Pcup; /* Legendre Function */
    float *dPcup; /* Derivative of Legendre fcn */
} MAGtype_LegendreFunction;

typedef struct {
    float *RelativeRadiusPower; /* [earth_reference_radius_km / sph. radius ]^n  */
    float *cos_mlambda; /*cp(m)  - cosine of (m*spherical coord. longitude)*/
    float *sin_mlambda; /* sp(m)  - sine of (m*spherical coord. longitude) */
} MAGtype_SphericalHarmonicVariables;

typedef struct {
    float Bx; /* North */
    float By; /* East */
    float Bz; /* Down */
} MAGtype_MagneticResults;

typedef struct {
    float Easting; /* (X) in meters*/
    float Northing; /* (Y) in meters */
    int Zone; /*UTM Zone*/
    char HemiSphere;
    float CentralMeridian;
    float ConvergenceOfMeridians;
    float PointScale;
} MAGtype_UTMParameters;

size_t MAG_strlcpy_equivalent(char *dst, char *src, size_t dstlen){
/*The strlcpy is not the standard C library on Linux*/
    size_t srclen;
    if (src != NULL){
        srclen = strlen(src);
    }
    else{
        return 0;
    }
    if (dstlen > 0){
        int copy_size = (srclen >= dstlen) ? dstlen - 1:srclen;
        memcpy(dst, src, copy_size);
        dst[dstlen-1] = '\0';
    }
    
    return srclen;
}
// ===================================================================================== HELPER FUNCTIONS
int parse_wmm_line_manual(const char* line, int* n, int* m,
                          float* g, float* h,
                          float* g_sv, float* h_sv) {
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

struct tm
{
    int tm_sec;   // seconds after the minute - [0, 60] including leap second
    int tm_min;   // minutes after the hour - [0, 59]
    int tm_hour;  // hours since midnight - [0, 23]
    int tm_mday;  // day of the month - [1, 31]
    int tm_mon;   // months since January - [0, 11]
    int tm_year;  // years since 1900
    int tm_wday;  // days since Sunday - [0, 6]
    int tm_yday;  // days since January 1 - [0, 365]
    int tm_isdst; // daylight savings time flag
};

float MAG_dtstr_to_dyear(char* edit_date){

    /* Parse the date string format as mm/dd/yyyy*/
    
    int day, month, year;
    float extra_day = 0;
    int months[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int total_days = 0;
    float dyear;
    struct tm tm = {0};

    if(sscanf(edit_date, "%d/%d/%d", &month, &day, &year) != 3){
        printf("Failed to parse the date string. Please use the format mm/dd/yyyy\n");
        return -1;
    }
    
    if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0){
        extra_day = 1;
    }

    float total_year_days = 365.0f + extra_day;
    months[2] = months[2] + extra_day;

    for(int i = 0; i < month; i++){
        total_days += months[i];
    }
    total_days += day;

    dyear = (float) year + (float)(total_days - 1)/total_year_days;

    return dyear;     
    
}

MAGtype_MagneticModel *MAG_AllocateModelMemory(int NumTerms)

/* Allocate memory for WMM Coefficients
 * Should be called before reading the model file *

  INPUT: NumTerms : int : Total number of spherical harmonic coefficients in the model


 OUTPUT:    Pointer to data structure MAGtype_MagneticModel with the following elements
                        float EditionDate;
                        float epoch;       Base time of Geomagnetic model epoch (yrs)
                        char  ModelName[20];
                        float *Main_Field_Coeff_G;          C - Gauss coefficients of main geomagnetic model (nT)
                        float *Main_Field_Coeff_H;          C - Gauss coefficients of main geomagnetic model (nT)
                        float *Secular_Var_Coeff_G;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
                        float *Secular_Var_Coeff_H;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
                        int nMax;  Maximum degree of spherical harmonic model
                        int nMaxSecVar; Maxumum degree of spherical harmonic secular model
                        int SecularVariationUsed; Whether or not the magnetic secular variation vector will be needed by program

                        FALSE: Failed to allocate memory
CALLS : none
 */
{
    MAGtype_MagneticModel *MagneticModel;
    int i;


    MagneticModel = (MAGtype_MagneticModel *) calloc(1, sizeof (MAGtype_MagneticModel));

    if(MagneticModel == NULL)
    {
        return NULL;
    }

    MagneticModel->Main_Field_Coeff_G = (float *) malloc((NumTerms + 1) * sizeof ( float));

    if(MagneticModel->Main_Field_Coeff_G == NULL)
    {
        return NULL;
    }

    MagneticModel->Main_Field_Coeff_H = (float *) malloc((NumTerms + 1) * sizeof ( float));

    if(MagneticModel->Main_Field_Coeff_H == NULL)
    {
        return NULL;
    }
    MagneticModel->Secular_Var_Coeff_G = (float *) malloc((NumTerms + 1) * sizeof ( float));
    if(MagneticModel->Secular_Var_Coeff_G == NULL)
    {
        return NULL;
    }
    MagneticModel->Secular_Var_Coeff_H = (float *) malloc((NumTerms + 1) * sizeof ( float));
    if(MagneticModel->Secular_Var_Coeff_H == NULL)
    {
        return NULL;
    }
    MagneticModel->CoefficientFileEndDate = 0;
    MagneticModel->EditionDate = 0;
    MAG_strlcpy_equivalent(MagneticModel->ModelName, "", sizeof(MagneticModel->ModelName));
    MagneticModel->SecularVariationUsed = 0;
    MagneticModel->epoch = 0;
    MagneticModel->nMax = 0;
    MagneticModel->nMaxSecVar = 0;
    
    for(i=0; i<NumTerms; i++) {
        MagneticModel->Main_Field_Coeff_G[i] = 0;
        MagneticModel->Main_Field_Coeff_H[i] = 0;
        MagneticModel->Secular_Var_Coeff_G[i] = 0;
        MagneticModel->Secular_Var_Coeff_H[i] = 0;
    }
    
    return MagneticModel;

} /*MAG_AllocateModelMemory*/

// All WMM parsing and output happens here to prepare for modularization
void magnetic_declination_init(MAGtype_MagneticModel *(*magneticmodels)[]) {
    int line_num = 0;
    const char* p = wmm_cof_2025_content;
    char line[128];

    
    (*magneticmodels)[0] = MAG_AllocateModelMemory(NumTerms);

    (*magneticmodels)[0]->Main_Field_Coeff_H[0] = 0.0f;
    (*magneticmodels)[0]->Main_Field_Coeff_G[0] = 0.0f;
    (*magneticmodels)[0]->Secular_Var_Coeff_H[0] = 0.0f;
    (*magneticmodels)[0]->Secular_Var_Coeff_G[0] = 0.0f;

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
        if (line_num == 0) {
            // Example first line: "   2025.0            WMM-2025     11/13/2024"
            // Use sscanf to extract components
            sscanf(line, "%lf %31s %31s", &wmm_epoch_year, wmm_model_name, wmm_edition_date);
            printf("WMM.COF Epoch: %.1f, Model Name: %s, Edition Date: %s\n", wmm_epoch_year, wmm_model_name, wmm_edition_date);

            (*magneticmodels)[0]->min_year = MAG_dtstr_to_dyear(wmm_edition_date);
            if ((*magneticmodels)[0]->min_year == -1){
                (*magneticmodels)[0]->min_year = wmm_epoch_year;
            } 



        } else {
            int n, m;
            float g_value, h_value, g_sv_value, h_sv_value;
            if (parse_wmm_line_manual(line, &n, &m, &g_value, &h_value, &g_sv_value, &h_sv_value)) {
                if (n <= NMAX && m <= n) {
                    int index = n * (n + 1) / 2 + m;
                    (*magneticmodels)[0]->Main_Field_Coeff_G[index] = g_value;
                    (*magneticmodels)[0]->Secular_Var_Coeff_G[index] = g_sv_value;
                    (*magneticmodels)[0]->Main_Field_Coeff_H[index] = h_value;
                    (*magneticmodels)[0]->Secular_Var_Coeff_H[index] = h_sv_value;
                }
            }
        }
        line_num++;
    }

    (*magneticmodels)[0]->nMax = NMAX;
    (*magneticmodels)[0]->nMaxSecVar = NMAX;
    (*magneticmodels)[0]->epoch = wmm_epoch_year;
    (*magneticmodels)[0]->CoefficientFileEndDate = (*magneticmodels)[0]->epoch + 5;

}

int MAG_DateToYear(MAGtype_Date *CalendarDate, char *Error)

/* Converts a given calendar date into a decimal year,
it also outputs an error string if there is a problem
INPUT  CalendarDate  Pointer to the  data  structure with the following elements
                        int	Year;
                        int	Month;
                        int	Day;
                        float DecimalYear;      decimal years
OUTPUT  CalendarDate  Pointer to the  data  structure with the following elements updated
                        float DecimalYear;      decimal years
                Error	pointer to an error string
CALLS : none

 */
{
    int temp = 0; /*Total number of days */
    int MonthDays[13];
    int ExtraDay = 0;
    int i;
    int Error_size = 255;
    if(CalendarDate->Month == 0)
    {
        CalendarDate->DecimalYear = CalendarDate->Year;
        return TRUE;
    }
    if((CalendarDate->Year % 4 == 0 && CalendarDate->Year % 100 != 0) || CalendarDate->Year % 400 == 0)
        ExtraDay = 1;
    MonthDays[0] = 0;
    MonthDays[1] = 31;
    MonthDays[2] = 28 + ExtraDay;
    MonthDays[3] = 31;
    MonthDays[4] = 30;
    MonthDays[5] = 31;
    MonthDays[6] = 30;
    MonthDays[7] = 31;
    MonthDays[8] = 31;
    MonthDays[9] = 30;
    MonthDays[10] = 31;
    MonthDays[11] = 30;
    MonthDays[12] = 31;

    /******************Validation********************************/
    if(CalendarDate->Month <= 0 || CalendarDate->Month > 12)
    {
        // The Error is passed as pointer and its size if defined outside of the function. The functions used to pass Error to MAG_DateToYear() defines the size of DMSstring are all 255.
        MAG_strlcpy_equivalent(Error, "\nError: The Month entered is invalid, valid months are '1 to 12'\n", Error_size);
        return 0;
    }
    if(CalendarDate->Day <= 0 || CalendarDate->Day > MonthDays[CalendarDate->Month])
    {
        printf("\nThe number of days in month %d is %d\n", CalendarDate->Month, MonthDays[CalendarDate->Month]);
        MAG_strlcpy_equivalent(Error, "\nError: The day entered is invalid\n",Error_size);
        return 0;
    }
    /****************Calculation of t***************************/
    for(i = 1; i <= CalendarDate->Month; i++)
        temp += MonthDays[i - 1];
    temp += CalendarDate->Day;
    CalendarDate->DecimalYear = CalendarDate->Year + (temp - 1) / (365.0f + ExtraDay);
    return 1;

} /*MAG_DateToYear*/

MAGtype_MagneticModel* allocate_coefsArr_memory(int nMax, MAGtype_MagneticModel* MagneticModel){
    int NumTerms;

    MAGtype_MagneticModel* TimedMagneticModel;

    if(nMax < MagneticModel->nMax) nMax = MagneticModel->nMax;
    NumTerms = ((nMax + 1) * (nMax + 2) / 2);
    TimedMagneticModel = MAG_AllocateModelMemory(NumTerms); /* For storing the time modified WMM Model parameters */

    return TimedMagneticModel;
}

int MAG_SetDefaults(MAGtype_Ellipsoid *Ellip, MAGtype_Geoid *Geoid)

/*
        Sets default values for WMM subroutines.

        UPDATES : Ellip
                        Geoid

        CALLS : none
 */
{

    /* Sets WGS-84 parameters */
    Ellip->a = 6378.137f; /*semi-major axis of the ellipsoid in */
    Ellip->b = 6356.7523142f; /*semi-minor axis of the ellipsoid in */
    Ellip->fla = 1 / 298.257223563f; /* flattening */
    Ellip->eps = sqrtf(1 - (Ellip->b * Ellip->b) / (Ellip->a * Ellip->a)); /*first eccentricity */
    Ellip->epssq = (Ellip->eps * Ellip->eps); /*first eccentricity squared */
    Ellip->re = 6371.2f; /* Earth's radius */

    /* Sets EGM-96 model file parameters */
    Geoid->NumbGeoidCols = 1441; /* 360 degrees of longitude at 15 minute spacing */
    Geoid->NumbGeoidRows = 721; /* 180 degrees of latitude  at 15 minute spacing */
    Geoid->NumbHeaderItems = 6; /* min, max lat, min, max long, lat, long spacing*/
    Geoid->ScaleFactor = 4; /* 4 grid cells per degree at 15 minute spacing  */
    Geoid->NumbGeoidElevs = Geoid->NumbGeoidCols * Geoid->NumbGeoidRows;
    Geoid->Geoid_Initialized = 0; /*  Geoid will be initialized only if this is set to zero */
    Geoid->UseGeoid = MAG_USE_GEOID;

    return TRUE;
} /*MAG_SetDefaults */

int MAG_GeodeticToSpherical(MAGtype_Ellipsoid Ellip, MAGtype_CoordGeodetic CoordGeodetic, MAGtype_CoordSpherical *CoordSpherical)

/* Converts Geodetic coordinates to Spherical coordinates

  INPUT   Ellip  data  structure with the following elements
                        float a; semi-major axis of the ellipsoid
                        float b; semi-minor axis of the ellipsoid
                        float fla;  flattening
                        float epssq; first eccentricity squared
                        float eps;  first eccentricity
                        float re; mean radius of  ellipsoid

                CoordGeodetic  Pointer to the  data  structure with the following elements updates
                        float lambda; ( longitude )
                        float phi; ( geodetic latitude )
                        float HeightAboveEllipsoid; ( height above the WGS84 ellipsoid (HaE) )
                        float HeightAboveGeoid; (height above the EGM96 Geoid model )

 OUTPUT		CoordSpherical 	Pointer to the data structure with the following elements
                        float lambda; ( longitude)
                        float phig; ( geocentric latitude )
                        float r;  	  ( distance from the center of the ellipsoid)

CALLS : none

 */
{
    float CosLat, SinLat, rc, xp, zp; /*all local variables */

    /*
     ** Convert geodetic coordinates, (defined by the WGS-84
     ** reference ellipsoid), to Earth Centered Earth Fixed Cartesian
     ** coordinates, and then to spherical coordinates.
     */

    CosLat = cosf(DEG2RAD(CoordGeodetic.phi));
    SinLat = sinf(DEG2RAD(CoordGeodetic.phi));

    /* compute the local radius of curvature on the WGS-84 reference ellipsoid */

    rc = Ellip.a / sqrtf(1.0f - Ellip.epssq * SinLat * SinLat);

    /* compute ECEF Cartesian coordinates of specified point (for longitude=0) */

    xp = (rc + CoordGeodetic.HeightAboveEllipsoid) * CosLat;
    zp = (rc * (1.0f - Ellip.epssq) + CoordGeodetic.HeightAboveEllipsoid) * SinLat;

    /* compute spherical radius and angle lambda and phi of specified point */

    CoordSpherical->r = sqrtf(xp * xp + zp * zp);
    CoordSpherical->phig = RAD2DEG(asinf(zp / CoordSpherical->r)); /* geocentric latitude */
    CoordSpherical->lambda = CoordGeodetic.lambda; /* longitude */

    return TRUE;
}/*MAG_GeodeticToSpherical*/


int MAG_TimelyModifyMagneticModel(MAGtype_Date UserDate, MAGtype_MagneticModel *MagneticModel, MAGtype_MagneticModel *TimedMagneticModel)

/* Time change the Model coefficients from the base year of the model using secular variation coefficients.
Store the coefficients of the static model with their values advanced from epoch t0 to epoch t.
Copy the SV coefficients.  If input "t�" is the same as "t0", then this is merely a copy operation.
If the address of "TimedMagneticModel" is the same as the address of "MagneticModel", then this procedure overwrites
the given item "MagneticModel".

INPUT: UserDate
           MagneticModel
OUTPUT:TimedMagneticModel
CALLS : none
 */
{
    int n, m, index, a, b;
    TimedMagneticModel->EditionDate = MagneticModel->EditionDate;
    TimedMagneticModel->epoch = MagneticModel->epoch;
    TimedMagneticModel->nMax = MagneticModel->nMax;
    TimedMagneticModel->nMaxSecVar = MagneticModel->nMaxSecVar;
    a = TimedMagneticModel->nMaxSecVar;
    b = (a * (a + 1) / 2 + a);
    MAG_strlcpy_equivalent(TimedMagneticModel->ModelName, MagneticModel->ModelName, sizeof(TimedMagneticModel->ModelName));
    for(n = 1; n <= MagneticModel->nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            if(index <= b)
            {
                TimedMagneticModel->Main_Field_Coeff_H[index] = MagneticModel->Main_Field_Coeff_H[index] + (UserDate.DecimalYear - MagneticModel->epoch) * MagneticModel->Secular_Var_Coeff_H[index];
                TimedMagneticModel->Main_Field_Coeff_G[index] = MagneticModel->Main_Field_Coeff_G[index] + (UserDate.DecimalYear - MagneticModel->epoch) * MagneticModel->Secular_Var_Coeff_G[index];
                TimedMagneticModel->Secular_Var_Coeff_H[index] = MagneticModel->Secular_Var_Coeff_H[index]; /* We need a copy of the secular var coef to calculate secular change */
                TimedMagneticModel->Secular_Var_Coeff_G[index] = MagneticModel->Secular_Var_Coeff_G[index];
            } else
            {
                TimedMagneticModel->Main_Field_Coeff_H[index] = MagneticModel->Main_Field_Coeff_H[index];
                TimedMagneticModel->Main_Field_Coeff_G[index] = MagneticModel->Main_Field_Coeff_G[index];
            }
        }
    }
    return TRUE;
} /* MAG_TimelyModifyMagneticModel */


MAGtype_LegendreFunction *MAG_AllocateLegendreFunctionMemory(int NumTerms)

/* Allocate memory for Associated Legendre Function data types.
   Should be called before computing Associated Legendre Functions.

 INPUT: NumTerms : int : Total number of spherical harmonic coefficients in the model


 OUTPUT:    Pointer to data structure MAGtype_LegendreFunction with the following elements
                        float *Pcup;  (  pointer to store Legendre Function  )
                        float *dPcup; ( pointer to store  Derivative of Legendre function )

                        FALSE: Failed to allocate memory

CALLS : none

 */
{
    MAGtype_LegendreFunction *LegendreFunction;

    LegendreFunction = (MAGtype_LegendreFunction *) calloc(1, sizeof (MAGtype_LegendreFunction));

    if(!LegendreFunction)
    {
        return NULL;
    }
    LegendreFunction->Pcup = (float *) malloc((NumTerms + 1) * sizeof ( float));
    if(LegendreFunction->Pcup == 0)
    {
        return NULL;
    }
    LegendreFunction->dPcup = (float *) malloc((NumTerms + 1) * sizeof ( float));
    if(LegendreFunction->dPcup == 0)
    {
        return NULL;
    }
    return LegendreFunction;
} /*MAGtype_LegendreFunction*/

MAGtype_SphericalHarmonicVariables* MAG_AllocateSphVarMemory(int nMax)
{
    MAGtype_SphericalHarmonicVariables* SphVariables;
    SphVariables  = (MAGtype_SphericalHarmonicVariables*) calloc(1, sizeof(MAGtype_SphericalHarmonicVariables));
    SphVariables->RelativeRadiusPower = (float *) malloc((nMax + 1) * sizeof ( float));
    SphVariables->cos_mlambda = (float *) malloc((nMax + 1) * sizeof (float));
    SphVariables->sin_mlambda = (float *) malloc((nMax + 1) * sizeof (float));
    return SphVariables;
} /*MAG_AllocateSphVarMemory*/


int MAG_ComputeSphericalHarmonicVariables(MAGtype_Ellipsoid Ellip, MAGtype_CoordSpherical CoordSpherical, int nMax, MAGtype_SphericalHarmonicVariables *SphVariables)

/* Computes Spherical variables
       Variables computed are (a/r)^(n+2), cos_m(lamda) and sin_m(lambda) for spherical harmonic
       summations. (Equations 10-12 in the WMM Technical Report)
       INPUT   Ellip  data  structure with the following elements
                             float a; semi-major axis of the ellipsoid
                             float b; semi-minor axis of the ellipsoid
                             float fla;  flattening
                             float epssq; first eccentricity squared
                             float eps;  first eccentricity
                             float re; mean radius of  ellipsoid
                     CoordSpherical 	A data structure with the following elements
                             float lambda; ( longitude)
                             float phig; ( geocentric latitude )
                             float r;  	  ( distance from the center of the ellipsoid)
                     nMax   integer 	 ( Maxumum degree of spherical harmonic secular model)\

     OUTPUT  SphVariables  Pointer to the   data structure with the following elements
             float RelativeRadiusPower[MAG_MAX_MODEL_DEGREES+1];   [earth_reference_radius_km  sph. radius ]^n
             float cos_mlambda[MAG_MAX_MODEL_DEGREES+1]; cp(m)  - cosine of (mspherical coord. longitude)
             float sin_mlambda[MAG_MAX_MODEL_DEGREES+1];  sp(m)  - sine of (mspherical coord. longitude)
     CALLS : none
 */
{
    float cos_lambda, sin_lambda;
    int m, n;
    cos_lambda = cosf(DEG2RAD(CoordSpherical.lambda));
    sin_lambda = sinf(DEG2RAD(CoordSpherical.lambda));
    /* for n = 0 ... model_order, compute (Radius of Earth / Spherical radius r)^(n+2)
    for n  1..nMax-1 (this is much faster than calling pow MAX_N+1 times).      */
    SphVariables->RelativeRadiusPower[0] = (Ellip.re / CoordSpherical.r) * (Ellip.re / CoordSpherical.r);
    for(n = 1; n <= nMax; n++)
    {
        SphVariables->RelativeRadiusPower[n] = SphVariables->RelativeRadiusPower[n - 1] * (Ellip.re / CoordSpherical.r);
    }

    /*
     Compute cosf(m*lambda), sinf(m*lambda) for m = 0 ... nMax
           cosf(a + b) = cosf(a)*cosf(b) - sinf(a)*sinf(b)
           sinf(a + b) = cosf(a)*sinf(b) + sinf(a)*cosf(b)
     */
    
    SphVariables->cos_mlambda[0] = 1.0f; // The size if cos_mlambda and sin_mlambda is nMax+1
    SphVariables->sin_mlambda[0] = 0.0f;
    if (nMax + 1 >= 2){
        SphVariables->cos_mlambda[1] = cos_lambda;
        SphVariables->sin_mlambda[1] = sin_lambda;
    }
    
    if (nMax + 1 >= 3){
        for(m = 2; m <= nMax; m++)
        {
            SphVariables->cos_mlambda[m] = SphVariables->cos_mlambda[m - 1] * cos_lambda - SphVariables->sin_mlambda[m - 1] * sin_lambda;
            SphVariables->sin_mlambda[m] = SphVariables->cos_mlambda[m - 1] * sin_lambda + SphVariables->sin_mlambda[m - 1] * cos_lambda;
        }
    }
    return TRUE;
} /*MAG_ComputeSphericalHarmonicVariables*/


int MAG_PcupLow(float *Pcup, float *dPcup, float x, int nMax)

/*   This function evaluates all of the Schmidt-semi normalized associated Legendre
        functions up to degree nMax.

        Calling Parameters:
                INPUT
                        nMax:	 Maximum spherical harmonic degree to compute.
                        x:		cosf(colatitude) or sinf(latitude).

                OUTPUT
                        Pcup:	A vector of all associated Legendgre polynomials evaluated at
                                        x up to nMax.
                   dPcup: Derivative of Pcup(x) with respect to latitude

        Notes: Overflow may occur if nMax > 20 , especially for high-latitudes.
        Use MAG_PcupHigh for large nMax.

   Written by Manoj Nair, June, 2009 . Manoj.C.Nair@Noaa.Gov.

  Note: In geomagnetism, the derivatives of ALF are usually found with
  respect to the colatitudes. Here the derivatives are found with respect
  to the latitude. The difference is a sign reversal for the derivative of
  the Associated Legendre Functions.
 */
{
    int n, m, index, index1, index2, NumTerms;
    float k, z, *schmidtQuasiNorm;
    Pcup[0] = 1.0f;
    dPcup[0] = 0.0f;
    /*sinf (geocentric latitude) - sin_phi */
    z = sqrtf((1.0f - x) * (1.0f + x));

    NumTerms = ((nMax + 1) * (nMax + 2) / 2);
    schmidtQuasiNorm = (float *) malloc((NumTerms + 1) * sizeof ( float));

    if(schmidtQuasiNorm == NULL)
    {
        return FALSE;
    }

    /*	 First,	Compute the Gauss-normalized associated Legendre  functions*/
    for(n = 1; n <= nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            if(n == m)
            {
                index1 = (n - 1) * n / 2 + m - 1;
                Pcup [index] = z * Pcup[index1];
                dPcup[index] = z * dPcup[index1] + x * Pcup[index1];
            } else if(n == 1 && m == 0)
            {
                index1 = (n - 1) * n / 2 + m;
                Pcup[index] = x * Pcup[index1];
                dPcup[index] = x * dPcup[index1] - z * Pcup[index1];
            } else if(n > 1 && n != m)
            {
                index1 = (n - 2) * (n - 1) / 2 + m;
                index2 = (n - 1) * n / 2 + m;
                if(m > n - 2)
                {
                    Pcup[index] = x * Pcup[index2];
                    dPcup[index] = x * dPcup[index2] - z * Pcup[index2];
                } else
                {
                    k = (float) (((n - 1) * (n - 1)) - (m * m)) / (float) ((2 * n - 1) * (2 * n - 3));
                    Pcup[index] = x * Pcup[index2] - k * Pcup[index1];
                    dPcup[index] = x * dPcup[index2] - z * Pcup[index2] - k * dPcup[index1];
                }
            }
        }
    }
    /* Compute the ration between the the Schmidt quasi-normalized associated Legendre
     * functions and the Gauss-normalized version. */

    schmidtQuasiNorm[0] = 1.0f;
    for(n = 1; n <= nMax; n++)
    {
        index = (n * (n + 1) / 2);
        index1 = (n - 1) * n / 2;
        /* for m = 0 */
        schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * (float) (2 * n - 1) / (float) n;

        for(m = 1; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            index1 = (n * (n + 1) / 2 + m - 1);
            schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * sqrtf((float) ((n - m + 1) * (m == 1 ? 2 : 1)) / (float) (n + m));
        }

    }

    /* Converts the  Gauss-normalized associated Legendre
              functions to the Schmidt quasi-normalized version using pre-computed
              relation stored in the variable schmidtQuasiNorm */

    for(n = 1; n <= nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            Pcup[index] = Pcup[index] * schmidtQuasiNorm[index];
            dPcup[index] = -dPcup[index] * schmidtQuasiNorm[index];
            /* The sign is changed since the new WMM routines use derivative with respect to latitude
            insted of co-latitude */
        }
    }

    if(schmidtQuasiNorm)
        free(schmidtQuasiNorm);
    return TRUE;
} /*MAG_PcupLow */


int MAG_PcupHigh(float *Pcup, float *dPcup, float x, int nMax)

/*	This function evaluates all of the Schmidt-semi normalized associated Legendre
        functions up to degree nMax. The functions are initially scaled by
        10^280 sinf^m in order to minimize the effects of underflow at large m
        near the poles (see Holmes and Featherstone 2002, J. Geodesy, 76, 279-299).
        Note that this function performs the same operation as MAG_PcupLow.
        However this function also can be used for high degree (large nMax) models.

        Calling Parameters:
                INPUT
                        nMax:	 Maximum spherical harmonic degree to compute.
                        x:		cosf(colatitude) or sinf(latitude).

                OUTPUT
                        Pcup:	A vector of all associated Legendgre polynomials evaluated at
                                        x up to nMax. The lenght must by greater or equal to (nMax+1)*(nMax+2)/2.
                  dPcup:   Derivative of Pcup(x) with respect to latitude

                CALLS : none
        Notes:



  Adopted from the FORTRAN code written by Mark Wieczorek September 25, 2005.

  Manoj Nair, Nov, 2009 Manoj.C.Nair@Noaa.Gov

  Change from the previous version
  The prevous version computes the derivatives as
  dP(n,m)(x)/dx, where x = sinf(latitude) (or cosf(colatitude) ).
  However, the WMM Geomagnetic routines requires dP(n,m)(x)/dlatitude.
  Hence the derivatives are multiplied by sinf(latitude).
  Removed the options for CS phase and normalizations.

  Note: In geomagnetism, the derivatives of ALF are usually found with
  respect to the colatitudes. Here the derivatives are found with respect
  to the latitude. The difference is a sign reversal for the derivative of
  the Associated Legendre Functions.

  The derivatives can't be computed for latitude = |90| degrees.
 */
{
    float pm2, pm1, pmm, plm, rescalem, z, scalef;
    float *f1, *f2, *PreSqr;
    int k, kstart, m, n, NumTerms;

    NumTerms = ((nMax + 1) * (nMax + 2) / 2);

    z = sqrtf((1.0f - x)*(1.0f + x));

    if(z == 0)
    {
        return 0;
    }



    if(fabsf(x) == 1.0f)
    {
        printf("Error in PcupHigh: derivative cannot be calculated at poles\n");
        return FALSE;
    }


    f1 = (float *) malloc((NumTerms + 1) * sizeof ( float));
    if(f1 == NULL)
    {
        return FALSE;
    }


    PreSqr = (float *) malloc((NumTerms + 1) * sizeof ( float));

    if(PreSqr == NULL)
    {
        return FALSE;
    }

    f2 = (float *) malloc((NumTerms + 1) * sizeof ( float));

    if(f2 == NULL)
    {
        return FALSE;
    }

    scalef = 1.0e-280;

    for(n = 0; n <= 2 * nMax + 1; ++n)
    {
        PreSqr[n] = sqrtf((float) (n));
    }

    k = 2;

    for(n = 2; n <= nMax; n++)
    {
        k = k + 1;
        f1[k] = (float) (2 * n - 1) / (float) (n);
        f2[k] = (float) (n - 1) / (float) (n);
        for(m = 1; m <= n - 2; m++)
        {
            k = k + 1;
            f1[k] = (float) (2 * n - 1) / PreSqr[n + m] / PreSqr[n - m];
            f2[k] = PreSqr[n - m - 1] * PreSqr[n + m - 1] / PreSqr[n + m] / PreSqr[n - m];
        }
        k = k + 2;
    }

    /*z = sinf (geocentric latitude) */

    pm2 = 1.0f;
    Pcup[0] = 1.0f;
    dPcup[0] = 0.0f;
    if(nMax == 0)
        return FALSE;
    pm1 = x;
    Pcup[1] = pm1;
    dPcup[1] = z;
    k = 1;

    for(n = 2; n <= nMax; n++)
    {
        k = k + n;
        plm = f1[k] * x * pm1 - f2[k] * pm2;
        Pcup[k] = plm;
        dPcup[k] = (float) (n) * (pm1 - x * plm) / z;
        pm2 = pm1;
        pm1 = plm;
    }

    pmm = PreSqr[2] * scalef;
    rescalem = 1.0f / scalef;
    kstart = 0;

    for(m = 1; m <= nMax - 1; ++m)
    {
        rescalem = rescalem*z;

        /* Calculate Pcup(m,m)*/
        kstart = kstart + m + 1;
        pmm = pmm * PreSqr[2 * m + 1] / PreSqr[2 * m];
        Pcup[kstart] = pmm * rescalem / PreSqr[2 * m + 1];
        dPcup[kstart] = -((float) (m) * x * Pcup[kstart] / z);
        pm2 = pmm / PreSqr[2 * m + 1];
        /* Calculate Pcup(m+1,m)*/
        k = kstart + m + 1;
        pm1 = x * PreSqr[2 * m + 1] * pm2;
        Pcup[k] = pm1*rescalem;
        dPcup[k] = ((pm2 * rescalem) * PreSqr[2 * m + 1] - x * (float) (m + 1) * Pcup[k]) / z;
        /* Calculate Pcup(n,m)*/
        for(n = m + 2; n <= nMax; ++n)
        {
            k = k + n;
            plm = x * f1[k] * pm1 - f2[k] * pm2;
            Pcup[k] = plm*rescalem;
            dPcup[k] = (PreSqr[n + m] * PreSqr[n - m] * (pm1 * rescalem) - (float) (n) * x * Pcup[k]) / z;
            pm2 = pm1;
            pm1 = plm;
        }
    }

    /* Calculate Pcup(nMax,nMax)*/
    rescalem = rescalem*z;
    kstart = kstart + m + 1;
    pmm = pmm / PreSqr[2 * nMax];
    Pcup[kstart] = pmm * rescalem;
    dPcup[kstart] = -(float) (nMax) * x * Pcup[kstart] / z;
    free(f1);
    free(PreSqr);
    free(f2);

    return TRUE;
} /* MAG_PcupHigh */


int MAG_AssociatedLegendreFunction(MAGtype_CoordSpherical CoordSpherical, int nMax, MAGtype_LegendreFunction *LegendreFunction)

/* Computes  all of the Schmidt-semi normalized associated Legendre
functions up to degree nMax. If nMax <= 16, function MAG_PcupLow is used.
Otherwise MAG_PcupHigh is called.
INPUT  CoordSpherical 	A data structure with the following elements
                                                float lambda; ( longitude)
                                                float phig; ( geocentric latitude )
                                                float r;  	  ( distance from the center of the ellipsoid)
                nMax        	integer 	 ( Maxumum degree of spherical harmonic secular model)
                LegendreFunction Pointer to data structure with the following elements
                                                float *Pcup;  (  pointer to store Legendre Function  )
                                                float *dPcup; ( pointer to store  Derivative of Lagendre function )

OUTPUT  LegendreFunction  Calculated Legendre variables in the data structure

 */
{
    float sin_phi;
    int FLAG = 1;

    sin_phi = sinf(DEG2RAD(CoordSpherical.phig)); /* sinf  (geocentric latitude) */

    if(nMax <= 16 || (1 - fabsf(sin_phi)) < 1.0e-10) /* If nMax is less tha 16 or at the poles */
        FLAG = MAG_PcupLow(LegendreFunction->Pcup, LegendreFunction->dPcup, sin_phi, nMax);
    else FLAG = MAG_PcupHigh(LegendreFunction->Pcup, LegendreFunction->dPcup, sin_phi, nMax);
    if(FLAG == 0) /* Error while computing  Legendre variables*/
        return FALSE;


    return TRUE;
} /*MAG_AssociatedLegendreFunction */


void MAG_GradYSummation(MAGtype_LegendreFunction *LegendreFunction, MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *GradY)
{
    int m, n, index;
    float cos_phi;
    GradY->Bz = 0.0f;
    GradY->By = 0.0f;
    GradY->Bx = 0.0f;
    for(n = 1; n <= MagneticModel->nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);

            GradY->Bz -= SphVariables.RelativeRadiusPower[n] *
                    (-1 * MagneticModel->Main_Field_Coeff_G[index] * SphVariables.sin_mlambda[m] +
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables.cos_mlambda[m])
                    * (float) (n + 1) * (float) (m) * LegendreFunction-> Pcup[index] * (1/CoordSpherical.r);
            GradY->By += SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Main_Field_Coeff_G[index] * SphVariables.cos_mlambda[m] +
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables.sin_mlambda[m])
                    * (float) (m * m) * LegendreFunction-> Pcup[index] * (1/CoordSpherical.r);
            GradY->Bx -= SphVariables.RelativeRadiusPower[n] *
                    (-1 * MagneticModel->Main_Field_Coeff_G[index] * SphVariables.sin_mlambda[m] +
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables.cos_mlambda[m])
                    * (float) (m) * LegendreFunction-> dPcup[index] * (1/CoordSpherical.r);



        }
    }

    cos_phi = cosf(DEG2RAD(CoordSpherical.phig));
    if(fabsf(cos_phi) > 1.0e-10)
    {
        GradY->By = GradY->By / (cos_phi * cos_phi);
        GradY->Bx = GradY->Bx / (cos_phi);
        GradY->Bz = GradY->Bz / (cos_phi);
    } else
        /* Special calculation for component - By - at Geographic poles.
         * If the user wants to avoid using this function,  please make sure that
         * the latitude is not exactly +/-90. An option is to make use the function
         * MAG_CheckGeographicPoles.
         */
    {
       /* MAG_SummationSpecial(MagneticModel, SphVariables, CoordSpherical, GradY); */
    }
}


int MAG_RotateMagneticVector(MAGtype_CoordSpherical CoordSpherical, MAGtype_CoordGeodetic CoordGeodetic, MAGtype_MagneticResults MagneticResultsSph, MAGtype_MagneticResults *MagneticResultsGeo)
/* Rotate the Magnetic Vectors to Geodetic Coordinates
Manoj Nair, June, 2009 Manoj.C.Nair@Noaa.Gov
Equation 16, WMM Technical report

INPUT : CoordSpherical : Data structure MAGtype_CoordSpherical with the following elements
                        float lambda; ( longitude)
                        float phig; ( geocentric latitude )
                        float r;  	  ( distance from the center of the ellipsoid)

                CoordGeodetic : Data structure MAGtype_CoordGeodetic with the following elements
                        float lambda; (longitude)
                        float phi; ( geodetic latitude)
                        float HeightAboveEllipsoid; (height above the ellipsoid (HaE) )
                        float HeightAboveGeoid;(height above the Geoid )

                MagneticResultsSph : Data structure MAGtype_MagneticResults with the following elements
                        float Bx;      North
                        float By;      East
                        float Bz;      Down

OUTPUT: MagneticResultsGeo Pointer to the data structure MAGtype_MagneticResults, with the following elements
                        float Bx;      North
                        float By;      East
                        float Bz;      Down

CALLS : none

 */
{
    float Psi;
    /* Difference between the spherical and Geodetic latitudes */
    Psi = (M_PI / 180) * (CoordSpherical.phig - CoordGeodetic.phi);

    /* Rotate spherical field components to the Geodetic system */
    MagneticResultsGeo->Bz = MagneticResultsSph.Bx * sinf(Psi) + MagneticResultsSph.Bz * cosf(Psi);
    MagneticResultsGeo->Bx = MagneticResultsSph.Bx * cosf(Psi) - MagneticResultsSph.Bz * sinf(Psi);
    MagneticResultsGeo->By = MagneticResultsSph.By;
    return TRUE;
} /*MAG_RotateMagneticVector*/

void MAG_CalculateGradientElements(MAGtype_MagneticResults GradResults, MAGtype_GeoMagneticElements MagneticElements, MAGtype_GeoMagneticElements *GradElements)
{
    GradElements->X = GradResults.Bx;
    GradElements->Y = GradResults.By;
    GradElements->Z = GradResults.Bz;
    
    GradElements->H = (GradElements->X * MagneticElements.X + GradElements->Y * MagneticElements.Y) / MagneticElements.H;
    GradElements->F = (GradElements->X * MagneticElements.X + GradElements->Y * MagneticElements.Y + GradElements->Z * MagneticElements.Z) / MagneticElements.F;
    GradElements->Decl = 180.0f / M_PI * (MagneticElements.X * GradElements->Y - MagneticElements.Y * GradElements->X) / (MagneticElements.H * MagneticElements.H);
    GradElements->Incl = 180.0f / M_PI * (MagneticElements.H * GradElements->Z - MagneticElements.Z * GradElements->H) / (MagneticElements.F * MagneticElements.F);
    GradElements->GV = GradElements->Decl;
}

int MAG_FreeLegendreMemory(MAGtype_LegendreFunction *LegendreFunction)

/* Free the Legendre Coefficients memory used by the WMM functions.
INPUT : LegendreFunction Pointer to data structure with the following elements
                                                float *Pcup;  (  pointer to store Legendre Function  )
                                                float *dPcup; ( pointer to store  Derivative of Lagendre function )

OUTPUT: none
CALLS : none

 */
{
    if(LegendreFunction->Pcup)
    {
        free(LegendreFunction->Pcup);
        LegendreFunction->Pcup = NULL;
    }
    if(LegendreFunction->dPcup)
    {
        free(LegendreFunction->dPcup);
        LegendreFunction->dPcup = NULL;
    }
    if(LegendreFunction)
    {
        free(LegendreFunction);
        LegendreFunction = NULL;
    }

    return TRUE;
} /*MAG_FreeLegendreMemory */

int MAG_FreeSphVarMemory(MAGtype_SphericalHarmonicVariables *SphVar)

/* Free the Spherical Harmonic Variable memory used by the WMM functions.
INPUT : LegendreFunction Pointer to data structure with the following elements
                                                float *RelativeRadiusPower
                                                float *cos_mlambda
                                                float *sin_mlambda
 OUTPUT: none
 CALLS : none
 */
{
    if(SphVar->RelativeRadiusPower)
    {
        free(SphVar->RelativeRadiusPower);
        SphVar->RelativeRadiusPower = NULL;
    }
    if(SphVar->cos_mlambda)
    {
        free(SphVar->cos_mlambda);
        SphVar->cos_mlambda = NULL;
    }
    if(SphVar->sin_mlambda)
    {
        free(SphVar->sin_mlambda);
        SphVar->sin_mlambda = NULL;
    }
    if(SphVar)
    {
        free(SphVar);
        SphVar = NULL;
    }

    return TRUE;
} /*MAG_FreeSphVarMemory*/


int MAG_SummationSpecial(MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *MagneticResults)
/* Special calculation for the component By at Geographic poles.
Manoj Nair, June, 2009 manoj.c.nair@noaa.gov
INPUT: MagneticModel
           SphVariables
           CoordSpherical
OUTPUT: MagneticResults
CALLS : none
See Section 1.4, "SINGULARITIES AT THE GEOGRAPHIC POLES", WMM Technical report

 */
{
    int n, index;
    float k, sin_phi, *PcupS, schmidtQuasiNorm1, schmidtQuasiNorm2, schmidtQuasiNorm3;

    PcupS = (float *) malloc((MagneticModel->nMax + 1) * sizeof (float));
    if(PcupS == 0)
    {
        return FALSE;
    }

    PcupS[0] = 1;
    schmidtQuasiNorm1 = 1.0f;

    MagneticResults->By = 0.0f;
    sin_phi = sinf(DEG2RAD(CoordSpherical.phig));

    for(n = 1; n <= MagneticModel->nMax; n++)
    {

        /*Compute the ration between the Gauss-normalized associated Legendre
  functions and the Schmidt quasi-normalized version. This is equivalent to
  sqrtf((m==0?1:2)*(n-m)!/(n+m!))*(2n-1)!!/(n-m)!  */

        index = (n * (n + 1) / 2 + 1);
        schmidtQuasiNorm2 = schmidtQuasiNorm1 * (float) (2 * n - 1) / (float) n;
        schmidtQuasiNorm3 = schmidtQuasiNorm2 * sqrtf((float) (n * 2) / (float) (n + 1));
        schmidtQuasiNorm1 = schmidtQuasiNorm2;
        if(n == 1)
        {
            PcupS[n] = PcupS[n - 1];
        } else
        {
            k = (float) (((n - 1) * (n - 1)) - 1) / (float) ((2 * n - 1) * (2 * n - 3));
            PcupS[n] = sin_phi * PcupS[n - 1] - k * PcupS[n - 2];
        }

        /*		  1 nMax  (n+2)    n     m            m           m
                By =    SUM (a/r) (m)  SUM  [g cosf(m p) + h sinf(m p)] dP (sinf(phi))
                           n=1             m=0   n            n           n  */
        /* Equation 11 in the WMM Technical report. Derivative with respect to longitude, divided by radius. */

        MagneticResults->By += SphVariables.RelativeRadiusPower[n] *
                (MagneticModel->Main_Field_Coeff_G[index] * SphVariables.sin_mlambda[1] -
                MagneticModel->Main_Field_Coeff_H[index] * SphVariables.cos_mlambda[1])
                * PcupS[n] * schmidtQuasiNorm3;
    }

    if(PcupS)
        free(PcupS);
    return TRUE;
}/*MAG_SummationSpecial */


int MAG_GetUTMParameters(float Latitude,
        float Longitude,
        int *Zone,
        char *Hemisphere,
        float *CentralMeridian)
{
    /*
     * The function MAG_GetUTMParameters converts geodetic (latitude and
     * longitude) coordinates to UTM projection parameters (zone, hemisphere and central meridian)
     * If any errors occur, the error code(s) are returned
     * by the function, otherwise TRUE is returned.
     *
     *    Latitude          : Latitude in radians                 (input)
     *    Longitude         : Longitude in radians                (input)
     *    Zone              : UTM zone                            (output)
     *    Hemisphere        : North or South hemisphere           (output)
     *    CentralMeridian	: Central Meridian of the UTM Zone in radians	   (output)
     */

    long Lat_Degrees;
    long Long_Degrees;
    long temp_zone;
    int Error_Code = 0;



    if((Latitude < DEG2RAD(MAG_UTM_MIN_LAT_DEGREE)) || (Latitude > DEG2RAD(MAG_UTM_MAX_LAT_DEGREE)))
    { /* Latitude out of range */
        Error_Code = 1;
    }
    if((Longitude < -M_PI) || (Longitude > (2 * M_PI)))
    { /* Longitude out of range */
        Error_Code = 1;
    }
    if(!Error_Code)
    { /* no errors */
        if(Longitude < 0)
            Longitude += (2 * M_PI) + 1.0e-10;
        Lat_Degrees = (long) (Latitude * 180.0f / M_PI);
        Long_Degrees = (long) (Longitude * 180.0f / M_PI);

        if(Longitude < M_PI)
            temp_zone = (long) (31 + ((Longitude * 180.0f / M_PI) / 6.0f));
        else
            temp_zone = (long) (((Longitude * 180.0f / M_PI) / 6.0f) - 29);
        if(temp_zone > 60)
            temp_zone = 1;
        /* UTM special cases */
        if((Lat_Degrees > 55) && (Lat_Degrees < 64) && (Long_Degrees > -1)
                && (Long_Degrees < 3))
            temp_zone = 31;
        if((Lat_Degrees > 55) && (Lat_Degrees < 64) && (Long_Degrees > 2)
                && (Long_Degrees < 12))
            temp_zone = 32;
        if((Lat_Degrees > 71) && (Long_Degrees > -1) && (Long_Degrees < 9))
            temp_zone = 31;
        if((Lat_Degrees > 71) && (Long_Degrees > 8) && (Long_Degrees < 21))
            temp_zone = 33;
        if((Lat_Degrees > 71) && (Long_Degrees > 20) && (Long_Degrees < 33))
            temp_zone = 35;
        if((Lat_Degrees > 71) && (Long_Degrees > 32) && (Long_Degrees < 42))
            temp_zone = 37;

        if(!Error_Code)
        {
            if(temp_zone >= 31)
                *CentralMeridian = (6 * temp_zone - 183) * M_PI / 180.0f;
            else
                *CentralMeridian = (6 * temp_zone + 177) * M_PI / 180.0f;
            *Zone = temp_zone;
            if(Latitude < 0) *Hemisphere = 'S';
            else *Hemisphere = 'N';
        }
    } /* END OF if (!Error_Code) */
    return (Error_Code);
} /* MAG_GetUTMParameters */


void MAG_TMfwd4(float Eps, float Epssq, float K0R4, float K0R4oa,
        float Acoeff[], float Lam0, float K0, float falseE,
        float falseN, int XYonly, float Lambda, float Phi,
        float *X, float *Y, float *pscale, float *CoM)
{

    /*  Transverse Mercator forward equations including point-scale and CoM
            =--------- =------- =--=--= ---------

       Algorithm developed by: C. Rollins   August 7, 2006
       C software written by:  K. Robins


            Constants fixed by choice of ellipsoid and choice of projection parameters
            ---------------

              Eps          Eccentricity (epsilon) of the ellipsoid
              Epssq        Eccentricity squared
            ( R4           Meridional isoperimetric radius   )
            ( K0           Central scale factor              )
              K0R4         K0 times R4
              K0R4oa       K0 times Ratio of R4 over semi-major axis
              Acoeff       Trig series coefficients, omega as a function of chi
              Lam0         Longitude of the central meridian in radians
              K0           Central scale factor, for example, 0.9996 for UTM
              falseE       False easting, for example, 500000 for UTM
              falseN       False northing

       Processing option
       ---------- ------

              XYonly       If one (1), then only X and Y will be properly
                                       computed.  Values returned for point-scale
                                       and CoM will merely be the trivial values for
                                       points on the central meridian

       Input items that identify the point to be converted
       ----- -----

              Lambda       Longitude (from Greenwich) in radians
              Phi          Latitude in radians

       Output items
       ------ -----

              X            X coordinate (Easting) in meters
              Y            Y coordinate (Northing) in meters
              pscale       point-scale (dimensionless)
          CoM          Convergence-of-meridians in radians
     */

    float Lam, CLam, SLam, CPhi, SPhi;
    float P, part1, part2, denom, CChi, SChi;
    float U, V;
    float T, Tsq, denom2;
    float c2u, s2u, c4u, s4u, c6u, s6u, c8u, s8u;
    float c2v, s2v, c4v, s4v, c6v, s6v, c8v, s8v;
    float Xstar, Ystar;
    float sig1, sig2, comroo;

    /*
       Ellipsoid to sphere
       --------- -- ------

       Convert longitude (Greenwhich) to longitude from the central meridian
       It is unnecessary to find the (-Pi, Pi] equivalent of the result.
       Compute its cosine and sine.
     */

    Lam = Lambda - Lam0;
    CLam = cosf(Lam);
    SLam = sinf(Lam);

    /*   Latitude  */

    CPhi = cosf(Phi);
    SPhi = sinf(Phi);

    /*   Convert geodetic latitude, Phi, to conformal latitude, Chi
         Only the cosine and sine of Chi are actually needed.        */

    P = expf(Eps * ATanH(Eps * SPhi));
    part1 = (1 + SPhi) / P;
    part2 = (1 - SPhi) * P;
    denom = 1 / (part1 + part2);
    CChi = 2 * CPhi * denom;
    SChi = (part1 - part2) * denom;

    /*
       Sphere to first plane
       ------ -- ----- -----

       Apply spherical theory of transverse Mercator to get (u,v) coordinates
       Note the order of the arguments in Fortran's version of ArcTan, i.e.
                 atan2f(y, x) = ATan(y/x)
       The two argument form of ArcTan is needed here.
     */

    T = CChi * SLam;
    U = ATanH(T);
    V = atan2f(SChi, CChi * CLam);

    /*
       Trigonometric multiple angles
       ------------- -------- ------

       Compute Cosh of even multiples of U
       Compute Sinh of even multiples of U
       Compute Cos  of even multiples of V
       Compute Sin  of even multiples of V
     */

    Tsq = T * T;
    denom2 = 1 / (1 - Tsq);
    c2u = (1 + Tsq) * denom2;
    s2u = 2 * T * denom2;
    c2v = (-1 + CChi * CChi * (1 + CLam * CLam)) * denom2;
    s2v = 2 * CLam * CChi * SChi * denom2;

    c4u = 1 + 2 * s2u * s2u;
    s4u = 2 * c2u * s2u;
    c4v = 1 - 2 * s2v * s2v;
    s4v = 2 * c2v * s2v;

    c6u = c4u * c2u + s4u * s2u;
    s6u = s4u * c2u + c4u * s2u;
    c6v = c4v * c2v - s4v * s2v;
    s6v = s4v * c2v + c4v * s2v;

    c8u = 1 + 2 * s4u * s4u;
    s8u = 2 * c4u * s4u;
    c8v = 1 - 2 * s4v * s4v;
    s8v = 2 * c4v * s4v;


    /*   First plane to second plane
         ----- ----- -- ------ -----

         Accumulate terms for X and Y
     */

    Xstar = Acoeff[3] * s8u * c8v;
    Xstar = Xstar + Acoeff[2] * s6u * c6v;
    Xstar = Xstar + Acoeff[1] * s4u * c4v;
    Xstar = Xstar + Acoeff[0] * s2u * c2v;
    Xstar = Xstar + U;

    Ystar = Acoeff[3] * c8u * s8v;
    Ystar = Ystar + Acoeff[2] * c6u * s6v;
    Ystar = Ystar + Acoeff[1] * c4u * s4v;
    Ystar = Ystar + Acoeff[0] * c2u * s2v;
    Ystar = Ystar + V;

    /*   Apply isoperimetric radius, scale adjustment, and offsets  */

    *X = K0R4 * Xstar + falseE;
    *Y = K0R4 * Ystar + falseN;


    /*  Point-scale and CoM
        ----- ----- --- ---  */

    if(XYonly == 1)
    {
        *pscale = K0;
        *CoM = 0;
    } else
    {
        sig1 = 8 * Acoeff[3] * c8u * c8v;
        sig1 = sig1 + 6 * Acoeff[2] * c6u * c6v;
        sig1 = sig1 + 4 * Acoeff[1] * c4u * c4v;
        sig1 = sig1 + 2 * Acoeff[0] * c2u * c2v;
        sig1 = sig1 + 1;

        sig2 = 8 * Acoeff[3] * s8u * s8v;
        sig2 = sig2 + 6 * Acoeff[2] * s6u * s6v;
        sig2 = sig2 + 4 * Acoeff[1] * s4u * s4v;
        sig2 = sig2 + 2 * Acoeff[0] * s2u * s2v;

        /*    Combined square roots  */
        comroo = sqrtf((1 - Epssq * SPhi * SPhi) * denom2 *
                (sig1 * sig1 + sig2 * sig2));

        *pscale = K0R4oa * 2 * denom * comroo;
        *CoM = atan2f(SChi * SLam, CLam) + atan2f(sig2, sig1);
    }
} /*MAG_TMfwd4*/


int MAG_GetTransverseMercator(MAGtype_CoordGeodetic CoordGeodetic, MAGtype_UTMParameters *UTMParameters)
/* Gets the UTM Parameters for a given Latitude and Longitude.

INPUT: CoordGeodetic : Data structure MAGtype_CoordGeodetic.
OUTPUT : UTMParameters : Pointer to data structure MAGtype_UTMParameters with the following elements
                     float Easting;  (X) in meters
                     float Northing; (Y) in meters
                     int Zone; UTM Zone
                     char HemiSphere ;
                     float CentralMeridian; Longitude of the Central Meridian of the UTM Zone
                     float ConvergenceOfMeridians;  Convergence of Meridians
                     float PointScale;
 */
{

    float Eps, Epssq;
    float Acoeff[8];
    float Lam0, K0, falseE, falseN;
    float K0R4, K0R4oa;
    float Lambda, Phi;
    int XYonly;
    float X, Y, pscale, CoM;
    int Zone;
    char Hemisphere;



    /*   Get the map projection  parameters */

    Lambda = DEG2RAD(CoordGeodetic.lambda);
    Phi = DEG2RAD(CoordGeodetic.phi);

    MAG_GetUTMParameters(Phi, Lambda, &Zone, &Hemisphere, &Lam0);
    K0 = 0.9996;



    if(Hemisphere == 'n' || Hemisphere == 'N')
    {
        falseN = 0;
    }
    if(Hemisphere == 's' || Hemisphere == 'S')
    {
        falseN = 10000000;
    }

    falseE = 500000;


    /* WGS84 ellipsoid */

    Eps = 0.081819190842621494335;
    Epssq = 0.0066943799901413169961;
    K0R4 = 6367449.1458234153093*K0;
    K0R4oa = K0R4/6378137;


    Acoeff[0] = 8.37731820624469723600E-04;
    Acoeff[1] = 7.60852777357248641400E-07;
    Acoeff[2] = 1.19764550324249124400E-09;
    Acoeff[3] = 2.42917068039708917100E-12;
    Acoeff[4] = 5.71181837042801392800E-15;
    Acoeff[5] = 1.47999793137966169400E-17;
    Acoeff[6] = 4.10762410937071532000E-20;
    Acoeff[7] = 1.21078503892257704200E-22;

    /* WGS84 ellipsoid */


    /*   Execution of the forward T.M. algorithm  */

    XYonly = 0;

    MAG_TMfwd4(Eps, Epssq, K0R4, K0R4oa, Acoeff,
            Lam0, K0, falseE, falseN,
            XYonly,
            Lambda, Phi,
            &X, &Y, &pscale, &CoM);

    /*   Report results  */

    UTMParameters->Easting = X; /* UTM Easting (X) in meters*/
    UTMParameters->Northing = Y; /* UTM Northing (Y) in meters */
    UTMParameters->Zone = Zone; /*UTM Zone*/
    UTMParameters->HemiSphere = Hemisphere;
    UTMParameters->CentralMeridian = RAD2DEG(Lam0); /* Central Meridian of the UTM Zone */
    UTMParameters->ConvergenceOfMeridians = RAD2DEG(CoM); /* Convergence of meridians of the UTM Zone and location */
    UTMParameters->PointScale = pscale;

    return 0;
} /*MAG_GetTransverseMercator*/


int MAG_CalculateGridVariation(MAGtype_CoordGeodetic location, MAGtype_GeoMagneticElements *elements)

/*Computes the grid variation for |latitudes| > MAG_MAX_LAT_DEGREE

Grivation (or grid variation) is the angle between grid north and
magnetic north. This routine calculates Grivation for the Polar Stereographic
projection for polar locations (Latitude => |55| deg). Otherwise, it computes the grid
variation in UTM projection system. However, the UTM projection codes may be used to compute
the grid variation at any latitudes.

INPUT    location    Data structure with the following elements
                float lambda; (longitude)
                float phi; ( geodetic latitude)
                float HeightAboveEllipsoid; (height above the ellipsoid (HaE) )
                float HeightAboveGeoid;(height above the Geoid )
OUTPUT  elements Data  structure with the following elements updated
                float GV; ( The Grid Variation )
CALLS : MAG_GetTransverseMercator

 */
{
    MAGtype_UTMParameters UTMParameters;
    if(location.phi >= MAG_PS_MAX_LAT_DEGREE)
    {
        elements->GV = elements->Decl - location.lambda;
        return 1;
    } else if(location.phi <= MAG_PS_MIN_LAT_DEGREE)
    {
        elements->GV = elements->Decl + location.lambda;
        return 1;
    } else
    {
        MAG_GetTransverseMercator(location, &UTMParameters);
        elements->GV = elements->Decl - UTMParameters.ConvergenceOfMeridians;
    }
    return 0;
} /*MAG_CalculateGridVariation*/


int MAG_Summation(MAGtype_LegendreFunction *LegendreFunction, MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *MagneticResults)
{
    /* Computes Geomagnetic Field Elements X, Y and Z in Spherical coordinate system using
    spherical harmonic summation.


    The vector Magnetic field is given by -grad V, where V is Geomagnetic scalar potential
    The gradient in spherical coordinates is given by:

                     dV ^     1 dV ^        1     dV ^
    grad V = -- r  +  - -- t  +  -------- -- p
                     dr       r dt       r sinf(t) dp


    INPUT :  LegendreFunction
                    MagneticModel
                    SphVariables
                    CoordSpherical
    OUTPUT : MagneticResults

    CALLS : MAG_SummationSpecial



    Manoj Nair, June, 2009 Manoj.C.Nair@Noaa.Gov
     */
    int m, n, index;
    float cos_phi;
    MagneticResults->Bz = 0.0f;
    MagneticResults->By = 0.0f;
    MagneticResults->Bx = 0.0f;
    for(n = 1; n <= MagneticModel->nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);

            /*		    nMax  	(n+2) 	  n     m            m           m
                    Bz =   -SUM (a/r)   (n+1) SUM  [g cosf(m p) + h sinf(m p)] P (sinf(phi))
                                    n=1      	      m=0   n            n           n  */
            /* Equation 12 in the WMM Technical report.  Derivative with respect to radius.*/
            MagneticResults->Bz -= SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Main_Field_Coeff_G[index] * SphVariables.cos_mlambda[m] +
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables.sin_mlambda[m])
                    * (float) (n + 1) * LegendreFunction-> Pcup[index];

            /*		  1 nMax  (n+2)    n     m            m           m
                    By =    SUM (a/r) (m)  SUM  [g cosf(m p) + h sinf(m p)] dP (sinf(phi))
                               n=1             m=0   n            n           n  */
            /* Equation 11 in the WMM Technical report. Derivative with respect to longitude, divided by radius. */
            MagneticResults->By += SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Main_Field_Coeff_G[index] * SphVariables.sin_mlambda[m] -
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables.cos_mlambda[m])
                    * (float) (m) * LegendreFunction-> Pcup[index];
            /*		   nMax  (n+2) n     m            m           m
                    Bx = - SUM (a/r)   SUM  [g cosf(m p) + h sinf(m p)] dP (sinf(phi))
                               n=1         m=0   n            n           n  */
            /* Equation 10  in the WMM Technical report. Derivative with respect to latitude, divided by radius. */

            MagneticResults->Bx -= SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Main_Field_Coeff_G[index] * SphVariables.cos_mlambda[m] +
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables.sin_mlambda[m])
                    * LegendreFunction-> dPcup[index];



        }
    }

    cos_phi = cosf(DEG2RAD(CoordSpherical.phig));
    if(fabsf(cos_phi) > 1.0e-10)
    {
        MagneticResults->By = MagneticResults->By / cos_phi;
    } else
        /* Special calculation for component - By - at Geographic poles.
         * If the user wants to avoid using this function,  please make sure that
         * the latitude is not exactly +/-90. An option is to make use the function
         * MAG_CheckGeographicPoles.
         */
    {
        MAG_SummationSpecial(MagneticModel, SphVariables, CoordSpherical, MagneticResults);
    }
    return TRUE;
}/*MAG_Summation */



int MAG_SecVarSummationSpecial(MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *MagneticResults)
{
    /*Special calculation for the secular variation summation at the poles.


    INPUT: MagneticModel
               SphVariables
               CoordSpherical
    OUTPUT: MagneticResults
    CALLS : none


     */
    int n, index;
    float k, sin_phi, *PcupS, schmidtQuasiNorm1, schmidtQuasiNorm2, schmidtQuasiNorm3;

    PcupS = (float *) malloc((MagneticModel->nMaxSecVar + 1) * sizeof (float));

    if(PcupS == NULL)
    {
        return FALSE;
    }

    PcupS[0] = 1;
    schmidtQuasiNorm1 = 1.0f;

    MagneticResults->By = 0.0f;
    sin_phi = sinf(DEG2RAD(CoordSpherical.phig));

    for(n = 1; n <= MagneticModel->nMaxSecVar; n++)
    {
        index = (n * (n + 1) / 2 + 1);
        schmidtQuasiNorm2 = schmidtQuasiNorm1 * (float) (2 * n - 1) / (float) n;
        schmidtQuasiNorm3 = schmidtQuasiNorm2 * sqrtf((float) (n * 2) / (float) (n + 1));
        schmidtQuasiNorm1 = schmidtQuasiNorm2;
        if(n == 1)
        {
            PcupS[n] = PcupS[n - 1];
        } else
        {
            k = (float) (((n - 1) * (n - 1)) - 1) / (float) ((2 * n - 1) * (2 * n - 3));
            PcupS[n] = sin_phi * PcupS[n - 1] - k * PcupS[n - 2];
        }

        /*		  1 nMax  (n+2)    n     m            m           m
                By =    SUM (a/r) (m)  SUM  [g cosf(m p) + h sinf(m p)] dP (sinf(phi))
                           n=1             m=0   n            n           n  */
        /* Derivative with respect to longitude, divided by radius. */

        MagneticResults->By += SphVariables.RelativeRadiusPower[n] *
                (MagneticModel->Secular_Var_Coeff_G[index] * SphVariables.sin_mlambda[1] -
                MagneticModel->Secular_Var_Coeff_H[index] * SphVariables.cos_mlambda[1])
                * PcupS[n] * schmidtQuasiNorm3;
    }

    if(PcupS)
        free(PcupS);
    return TRUE;
}/*SecVarSummationSpecial*/


int MAG_SecVarSummation(MAGtype_LegendreFunction *LegendreFunction, MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *MagneticResults)
{
    /*This Function sums the secular variation coefficients to get the secular variation of the Magnetic vector.
    INPUT :  LegendreFunction
                    MagneticModel
                    SphVariables
                    CoordSpherical
    OUTPUT : MagneticResults

    CALLS : MAG_SecVarSummationSpecial

     */
    int m, n, index;
    float cos_phi;
    MagneticModel->SecularVariationUsed = TRUE;
    MagneticResults->Bz = 0.0f;
    MagneticResults->By = 0.0f;
    MagneticResults->Bx = 0.0f;
    for(n = 1; n <= MagneticModel->nMaxSecVar; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);

            /*		    nMax  	(n+2) 	  n     m            m           m
                    Bz =   -SUM (a/r)   (n+1) SUM  [g cosf(m p) + h sinf(m p)] P (sinf(phi))
                                    n=1      	      m=0   n            n           n  */
            /*  Derivative with respect to radius.*/
            MagneticResults->Bz -= SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Secular_Var_Coeff_G[index] * SphVariables.cos_mlambda[m] +
                    MagneticModel->Secular_Var_Coeff_H[index] * SphVariables.sin_mlambda[m])
                    * (float) (n + 1) * LegendreFunction-> Pcup[index];

            /*		  1 nMax  (n+2)    n     m            m           m
                    By =    SUM (a/r) (m)  SUM  [g cosf(m p) + h sinf(m p)] dP (sinf(phi))
                               n=1             m=0   n            n           n  */
            /* Derivative with respect to longitude, divided by radius. */
            MagneticResults->By += SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Secular_Var_Coeff_G[index] * SphVariables.sin_mlambda[m] -
                    MagneticModel->Secular_Var_Coeff_H[index] * SphVariables.cos_mlambda[m])
                    * (float) (m) * LegendreFunction-> Pcup[index];
            /*		   nMax  (n+2) n     m            m           m
                    Bx = - SUM (a/r)   SUM  [g cosf(m p) + h sinf(m p)] dP (sinf(phi))
                               n=1         m=0   n            n           n  */
            /* Derivative with respect to latitude, divided by radius. */

            MagneticResults->Bx -= SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Secular_Var_Coeff_G[index] * SphVariables.cos_mlambda[m] +
                    MagneticModel->Secular_Var_Coeff_H[index] * SphVariables.sin_mlambda[m])
                    * LegendreFunction-> dPcup[index];
        }
    }
    cos_phi = cosf(DEG2RAD(CoordSpherical.phig));
    if(fabsf(cos_phi) > 1.0e-10)
    {
        MagneticResults->By = MagneticResults->By / cos_phi;
    } else
        /* Special calculation for component By at Geographic poles */
    {
        MAG_SecVarSummationSpecial(MagneticModel, SphVariables, CoordSpherical, MagneticResults);
    }
    return TRUE;
} /*MAG_SecVarSummation*/


int MAG_CalculateGeoMagneticElements(MAGtype_MagneticResults *MagneticResultsGeo, MAGtype_GeoMagneticElements *GeoMagneticElements)

/* Calculate all the Geomagnetic elements from X,Y and Z components
INPUT     MagneticResultsGeo   Pointer to data structure with the following elements
                        float Bx;    ( North )
                        float By;	  ( East )
                        float Bz;    ( Down )
OUTPUT    GeoMagneticElements    Pointer to data structure with the following elements
                        float Decl; (Angle between the magnetic field vector and true north, positive east)
                        float Incl; Angle between the magnetic field vector and the horizontal plane, positive down
                        float F; Magnetic Field Strength
                        float H; Horizontal Magnetic Field Strength
                        float X; Northern component of the magnetic field vector
                        float Y; Eastern component of the magnetic field vector
                        float Z; Downward component of the magnetic field vector
CALLS : none
 */
{
    GeoMagneticElements->X = MagneticResultsGeo->Bx;
    GeoMagneticElements->Y = MagneticResultsGeo->By;
    GeoMagneticElements->Z = MagneticResultsGeo->Bz;

    GeoMagneticElements->H = sqrtf(MagneticResultsGeo->Bx * MagneticResultsGeo->Bx + MagneticResultsGeo->By * MagneticResultsGeo->By);
    GeoMagneticElements->F = sqrtf(GeoMagneticElements->H * GeoMagneticElements->H + MagneticResultsGeo->Bz * MagneticResultsGeo->Bz);
    GeoMagneticElements->Decl = RAD2DEG(atan2f(GeoMagneticElements->Y, GeoMagneticElements->X));
    GeoMagneticElements->Incl = RAD2DEG(atan2f(GeoMagneticElements->Z, GeoMagneticElements->H));

    return TRUE;
} /*MAG_CalculateGeoMagneticElements */


int MAG_CalculateSecularVariationElements(MAGtype_MagneticResults MagneticVariation, MAGtype_GeoMagneticElements *MagneticElements)
/*This takes the Magnetic Variation in x, y, and z and uses it to calculate the secular variation of each of the Geomagnetic elements.
        INPUT     MagneticVariation   Data structure with the following elements
                                float Bx;    ( North )
                                float By;	  ( East )
                                float Bz;    ( Down )
        OUTPUT   MagneticElements   Pointer to the data  structure with the following elements updated
                        float Decldot; Yearly Rate of change in declination
                        float Incldot; Yearly Rate of change in inclination
                        float Fdot; Yearly rate of change in Magnetic field strength
                        float Hdot; Yearly rate of change in horizontal field strength
                        float Xdot; Yearly rate of change in the northern component
                        float Ydot; Yearly rate of change in the eastern component
                        float Zdot; Yearly rate of change in the downward component
                        float GVdot;Yearly rate of chnage in grid variation
        CALLS : none

 */
{
    MagneticElements->Xdot = MagneticVariation.Bx;
    MagneticElements->Ydot = MagneticVariation.By;
    MagneticElements->Zdot = MagneticVariation.Bz;
    MagneticElements->Hdot = (MagneticElements->X * MagneticElements->Xdot + MagneticElements->Y * MagneticElements->Ydot) / MagneticElements->H; /* See equation 19 in the WMM technical report */
    MagneticElements->Fdot = (MagneticElements->X * MagneticElements->Xdot + MagneticElements->Y * MagneticElements->Ydot + MagneticElements->Z * MagneticElements->Zdot) / MagneticElements->F;
    MagneticElements->Decldot = 180.0f / M_PI * (MagneticElements->X * MagneticElements->Ydot - MagneticElements->Y * MagneticElements->Xdot) / (MagneticElements->H * MagneticElements->H);
    MagneticElements->Incldot = 180.0f / M_PI * (MagneticElements->H * MagneticElements->Zdot - MagneticElements->Z * MagneticElements->Hdot) / (MagneticElements->F * MagneticElements->F);
    MagneticElements->GVdot = MagneticElements->Decldot;
    return TRUE;
} /*MAG_CalculateSecularVariationElements*/


int MAG_Geomag(MAGtype_Ellipsoid Ellip, MAGtype_CoordSpherical CoordSpherical, MAGtype_CoordGeodetic CoordGeodetic,
        MAGtype_MagneticModel *TimedMagneticModel, MAGtype_GeoMagneticElements *GeoMagneticElements)
/*
The main subroutine that calls a sequence of WMM sub-functions to calculate the magnetic field elements for a single point.
The function expects the model coefficients and point coordinates as input and returns the magnetic field elements and
their rate of change. Though, this subroutine can be called successively to calculate a time series, profile or grid
of magnetic field, these are better achieved by the subroutine MAG_Grid.

INPUT: Ellip
              CoordSpherical
              CoordGeodetic
              TimedMagneticModel

OUTPUT : GeoMagneticElements

CALLS:  	MAG_AllocateLegendreFunctionMemory(NumTerms);  ( For storing the ALF functions )
                     MAG_ComputeSphericalHarmonicVariables( Ellip, CoordSpherical, TimedMagneticModel->nMax, &SphVariables); (Compute Spherical Harmonic variables  )
                     MAG_AssociatedLegendreFunction(CoordSpherical, TimedMagneticModel->nMax, LegendreFunction);  	Compute ALF
                     MAG_Summation(LegendreFunction, TimedMagneticModel, SphVariables, CoordSpherical, &MagneticResultsSph);  Accumulate the spherical harmonic coefficients
                     MAG_SecVarSummation(LegendreFunction, TimedMagneticModel, SphVariables, CoordSpherical, &MagneticResultsSphVar); Sum the Secular Variation Coefficients
                     MAG_RotateMagneticVector(CoordSpherical, CoordGeodetic, MagneticResultsSph, &MagneticResultsGeo); Map the computed Magnetic fields to Geodetic coordinates
                     MAG_CalculateGeoMagneticElements(&MagneticResultsGeo, GeoMagneticElements);   Calculate the Geomagnetic elements
                     MAG_CalculateSecularVariationElements(MagneticResultsGeoVar, GeoMagneticElements); Calculate the secular variation of each of the Geomagnetic elements

 */
{
    MAGtype_LegendreFunction *LegendreFunction;
    MAGtype_SphericalHarmonicVariables *SphVariables;
    int NumTerms;
    MAGtype_MagneticResults MagneticResultsSph, MagneticResultsGeo, MagneticResultsSphVar, MagneticResultsGeoVar;

    NumTerms = ((TimedMagneticModel->nMax + 1) * (TimedMagneticModel->nMax + 2) / 2); 
    LegendreFunction = MAG_AllocateLegendreFunctionMemory(NumTerms); /* For storing the ALF functions */
    SphVariables = MAG_AllocateSphVarMemory(TimedMagneticModel->nMax);
    MAG_ComputeSphericalHarmonicVariables(Ellip, CoordSpherical, TimedMagneticModel->nMax, SphVariables); /* Compute Spherical Harmonic variables  */
    MAG_AssociatedLegendreFunction(CoordSpherical, TimedMagneticModel->nMax, LegendreFunction); /* Compute ALF  */
    MAG_Summation(LegendreFunction, TimedMagneticModel, *SphVariables, CoordSpherical, &MagneticResultsSph); /* Accumulate the spherical harmonic coefficients*/
    MAG_SecVarSummation(LegendreFunction, TimedMagneticModel, *SphVariables, CoordSpherical, &MagneticResultsSphVar); /*Sum the Secular Variation Coefficients  */
    MAG_RotateMagneticVector(CoordSpherical, CoordGeodetic, MagneticResultsSph, &MagneticResultsGeo); /* Map the computed Magnetic fields to Geodeitic coordinates  */
    MAG_RotateMagneticVector(CoordSpherical, CoordGeodetic, MagneticResultsSphVar, &MagneticResultsGeoVar); /* Map the secular variation field components to Geodetic coordinates*/
    MAG_CalculateGeoMagneticElements(&MagneticResultsGeo, GeoMagneticElements); /* Calculate the Geomagnetic elements, Equation 19 , WMM Technical report */
    MAG_CalculateSecularVariationElements(MagneticResultsGeoVar, GeoMagneticElements); /*Calculate the secular variation of each of the Geomagnetic elements*/

    MAG_FreeLegendreMemory(LegendreFunction);
    MAG_FreeSphVarMemory(SphVariables);

    return TRUE;
} /*MAG_Geomag*/


void MAG_WMMErrorCalc(float H, MAGtype_GeoMagneticElements *Uncertainty)
{
    float decl_variable, decl_constant;
    Uncertainty->F = WMM_UNCERTAINTY_F;
    Uncertainty->H = WMM_UNCERTAINTY_H;
    Uncertainty->X = WMM_UNCERTAINTY_X;
    Uncertainty->Z = WMM_UNCERTAINTY_Z;
    Uncertainty->Incl = WMM_UNCERTAINTY_I;
    Uncertainty->Y = WMM_UNCERTAINTY_Y;
     decl_variable = (WMM_UNCERTAINTY_D_COEF / H);
     decl_constant = (WMM_UNCERTAINTY_D_OFFSET);
     Uncertainty->Decl = sqrtf(decl_constant*decl_constant + decl_variable*decl_variable);
     if (Uncertainty->Decl > 180) {
         Uncertainty->Decl = 180;
     }
}

/*
 * The function is for point calculation for ewmm_point. The input of hight is already determined whether to
 * covert to Ellipsoid height
 */
void point_calc(MAGtype_Ellipsoid Ellip, MAGtype_CoordGeodetic CoordGeodetic, MAGtype_CoordSpherical* CoordSpherical,
                MAGtype_Date UserDate, MAGtype_MagneticModel* MagneticModel, MAGtype_MagneticModel* TimedMagneticModel,
                MAGtype_GeoMagneticElements* GeoMagneticElements, MAGtype_GeoMagneticElements* Errors){


    MAG_TimelyModifyMagneticModel(UserDate, MagneticModel, TimedMagneticModel); /* Time adjust the coefficients, Equation 19, WMM Technical report */
    MAG_Geomag(Ellip, *CoordSpherical, CoordGeodetic, TimedMagneticModel, GeoMagneticElements); /* Computes the geoMagnetic field elements and their time change*/
    MAG_CalculateGridVariation(CoordGeodetic, GeoMagneticElements);
    #ifdef WMMHR
        MAG_WMMHRErrorCalc(GeoMagneticElements->H, Errors);
    #else
        MAG_WMMErrorCalc(GeoMagneticElements->H, Errors);
    #endif
}


void MAG_DegreeToDMSstring(float DegreesOfArc, int UnitDepth, char *DMSstring)

/*This converts a given decimal degree into a DMS string.
INPUT  DegreesOfArc   decimal degree
           UnitDepth	How many iterations should be printed,
                        1 = Degrees
                        2 = Degrees, Minutes
                        3 = Degrees, Minutes, Seconds
OUPUT  DMSstring 	 pointer to DMSString.  Must be at least 30 characters.
CALLS : none
 */
{
    int DMS[3], i;
    float temp = DegreesOfArc;
    int tmp_size = 36;
    int tmp2_size = 32;
    char *tempstring = malloc(sizeof(char)*tmp_size);
    char *tempstring2 = malloc(sizeof(char)*tmp2_size);
    int DMSstring_size = 100;

    memset(tempstring, '\0', tmp_size);
    memset(tempstring2, '\0', tmp2_size);

    // The DMSstring is passed as pointer and its size if defined outside of the function. The functions used to pass DMSstring to MAG_DegreeToDMSstring() define the size of DMSstring are all 100.
    //MAG_strlcpy_equivalent(DMSstring, "", DMSstring_size);
    for(i = 0; i < UnitDepth; i++)
    {
        DMS[i] = (int) temp;
        switch(i) {
            case 0:
                MAG_strlcpy_equivalent(tempstring2, "Deg", tmp2_size);
                break;
            case 1:
                MAG_strlcpy_equivalent(tempstring2, "Min", tmp2_size);
                break;
            case 2:
                MAG_strlcpy_equivalent(tempstring2, "Sec", tmp2_size);
                break;
        }
        temp = (temp - DMS[i])*60;
        if(i == UnitDepth - 1 && temp >= 30)
            DMS[i]++;
        else if(i == UnitDepth - 1 && temp <= -30)
            DMS[i]--;
        snprintf(tempstring, tmp_size, "%4d%4s", DMS[i], tempstring2);
        strcat(DMSstring, tempstring);

       
        
    }
    
    free(tempstring);
    free(tempstring2);


} /*MAG_DegreeToDMSstring*/

bool wmm_compute_elements_completed = false;

MAGtype_MagneticModel * MagneticModels[1], *TimedMagneticModel;
MAGtype_Ellipsoid Ellip;
MAGtype_CoordSpherical CoordSpherical;
MAGtype_CoordGeodetic CoordGeodetic;
MAGtype_Date UserDate;
MAGtype_GeoMagneticElements GeoMagneticElements, Errors;
MAGtype_Geoid Geoid;

void wmm_init(){
    // Load and parse coefficients from WMM.COF content string
    magnetic_declination_init(&MagneticModels);

    TimedMagneticModel = MAG_AllocateModelMemory(NumTerms); /* For storing the time modified WMM Model parameters */

    if(MagneticModels[0] == NULL || TimedMagneticModel == NULL)
    {
        printf("BAD\n");
    }

    TimedMagneticModel = allocate_coefsArr_memory(NMAX, MagneticModels[0]);

    MAG_SetDefaults(&Ellip, &Geoid); /* Set default values and constants */
    /* Check for Geographic Poles */

    // This shit is when user enters altitude above sea level. If geoid then not needed.
    Geoid.GeoidHeightBuffer = NULL;
    Geoid.Geoid_Initialized = 1;
}

void wmm_compute_elements(float latitude, float longitude, float height_above_geoid_kilometers, int full_year, int month, int day){
    // =========================================== MAG_GetUserInput(MagneticModels[0], &Geoid, &CoordGeodetic, &UserDate);

    char Error_Message[255];
    char ans[20], b;
    CoordGeodetic.phi = latitude;
    CoordGeodetic.lambda = longitude;

    CoordGeodetic.HeightAboveEllipsoid = height_above_geoid_kilometers;
    Geoid.UseGeoid = 0;
    CoordGeodetic.HeightAboveGeoid = CoordGeodetic.HeightAboveEllipsoid;

    UserDate.Day = day;
    UserDate.Month = month;
    UserDate.Year = full_year;

    MAG_DateToYear(&UserDate, Error_Message);

    MAG_GeodeticToSpherical(Ellip, CoordGeodetic, &CoordSpherical);

    point_calc(
        Ellip, 
        CoordGeodetic, 
        &CoordSpherical,
        UserDate, 
        MagneticModels[0], 
        TimedMagneticModel,
        &GeoMagneticElements, 
        &Errors
    );

    wmm_compute_elements_completed = true;
}

float wmm_get_declination_degrees(){
    if(!wmm_compute_elements_completed) return 0.0f;

    return GeoMagneticElements.Decl;
}

void wmm_print_declination(){
    if(!wmm_compute_elements_completed) return;

    int dms_size = 150;
    char *DeclString = malloc(sizeof(char)*dms_size);
    memset(DeclString, '\0', dms_size);

    MAG_DegreeToDMSstring(GeoMagneticElements.Decl, 2, DeclString);

    if(GeoMagneticElements.Decl < 0){
        printf("Decl	=%20s  (WEST) +/-%3.0f Min Ddot = %.1f\tMin/yr\n", DeclString, 60 * Errors.Decl, 60 * GeoMagneticElements.Decldot);
    }else{
        printf("Decl	=%20s  (EAST) +/-%3.0f Min Ddot = %.1f\tMin/yr\n", DeclString, 60 * Errors.Decl, 60 * GeoMagneticElements.Decldot);
    }

    free(DeclString);
}

void wmm_destroy_computed_elements(){
    if(!wmm_compute_elements_completed) return;

    wmm_compute_elements_completed = false;

    // The wmm code already does a great job at freeing everything it used 
    // The rest will be overwritten once a new computation is done.
}

int main() {
    float latitude = 55.6516762; 
    float longitude = 12.5750904; 
    float height_above_geoid_km = 0.03950; // 1.3 m as kilometers
    float decimal_year = 2025;

    int year = 2025;
    int month = 10;
    int day = 15;

    wmm_init();
    wmm_compute_elements(latitude, longitude, height_above_geoid_km, year, month, day);
    float declination_degreees = wmm_get_declination_degrees();
    printf("Declination %f\n", declination_degreees);
    wmm_print_declination();
    wmm_destroy_computed_elements();

    return 0;
}

// RUN THIS
// gcc -o wmm_test wmm_test.c
// ./wmm_test


// sin
// asin
// exp
// cos
// atan2
// sqrt
// fabs
// log