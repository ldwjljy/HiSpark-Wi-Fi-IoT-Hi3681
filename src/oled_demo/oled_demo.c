#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_i2c.h"
#include "iot_watchdog.h"
#include "hi_adc.h"
#include "hi_io.h"
#include "hi_time.h"
#include "hi_types_base.h"
#include "wifi_hotspot.h"
#include "lwip/sockets.h"

#define OLED_I2C_IDX 0
#define OLED_ADDR 0x78
#define OLED_WIDTH 128
#define OLED_PAGES 8
#define OLED_BAUDRATE 400000

#define WIFI_AP_SSID "HMZXYYDS"
#define WIFI_AP_PASSWORD "HMZXYYDS"
#define WIFI_AP_IP "192.168.5.1"
#define HTTP_PORT 80

#define GPIO_FUNC 0
#define MOTOR_LEFT_A 0
#define MOTOR_LEFT_B 1
#define SERVO_GPIO 2
#define DISPLAY_SWITCH_GPIO 5
#define ULTRASONIC_TRIG 7
#define ULTRASONIC_ECHO 8
#define MOTOR_RIGHT_A 9
#define MOTOR_RIGHT_B 10
#define TRACE_LEFT 11
#define TRACE_RIGHT 12
#define OLED_SDA 13
#define OLED_SCL 14

#define KEY_DEBOUNCE_TICKS 30
#define KEY_ADC_CHANNEL HI_ADC_CHANNEL_2
#define KEY_ADC_SAMPLES 20
/* OLED S1/S2 share ADC2. The core-board user key is near 0V and is ignored. */
#define KEY1_MIN_MV 450
#define KEY1_MAX_MV 760
#define KEY2_MIN_MV 850
#define KEY2_MAX_MV 1250

#define DEFAULT_OBSTACLE_DISTANCE_CM 20
#define DEFAULT_COLLISION_DISTANCE_CM 8
#define DEFAULT_REMOTE_TIMEOUT_MS 1000
#define MIN_OBSTACLE_DISTANCE_CM 10
#define MAX_OBSTACLE_DISTANCE_CM 80
#define MIN_COLLISION_DISTANCE_CM 5
#define MAX_COLLISION_DISTANCE_CM 30
#define MIN_REMOTE_TIMEOUT_MS 500
#define MAX_REMOTE_TIMEOUT_MS 5000
#define SERVO_PERIOD_US 20000
#define SERVO_LEFT_US 1000
#define SERVO_CENTER_US 1500
#define SERVO_RIGHT_US 2000
#define SERVO_PULSE_COUNT 10
#define SERVO_SETTLE_TICKS 12
#define DISTANCE_TIMEOUT_US 30000
#define DEFAULT_BATTERY_PERCENT 100
#define RADAR_POINTS 5
#define RADAR_SCAN_INTERVAL_MS 260

typedef enum {
    KEY_NONE = 0,
    KEY_1,
    KEY_2,
} KeyId;

typedef enum {
    MODE_STOP = 0,
    MODE_REMOTE,
    MODE_TRACE,
    MODE_OBSTACLE,
} CarMode;

typedef enum {
    VIEW_CONTROL = 0,
    VIEW_PARAM,
} DisplayView;

typedef enum {
    PARAM_PAGE_CAR = 0,
    PARAM_PAGE_NET,
    PARAM_PAGE_SET,
    PARAM_PAGE_COUNT,
} ParamPage;

typedef enum {
    REMOTE_STOP = 0,
    REMOTE_FORWARD,
    REMOTE_BACKWARD,
    REMOTE_LEFT,
    REMOTE_RIGHT,
} RemoteAction;

static volatile CarMode g_carMode = MODE_STOP;
static volatile DisplayView g_displayView = VIEW_CONTROL;
static volatile ParamPage g_paramPage = PARAM_PAGE_CAR;
static volatile RemoteAction g_remoteAction = REMOTE_STOP;
static volatile unsigned int g_lastKeyTick = 0;
static volatile unsigned char g_displayDirty = 1;
static volatile int g_lastDistanceCm = -1;
static volatile unsigned int g_speedMps100 = 0;
static volatile unsigned char g_batteryPercent = DEFAULT_BATTERY_PERCENT;
static volatile unsigned char g_wifiReady = 0;
static volatile unsigned char g_wifiClients = 0;
static volatile unsigned char g_nfcReady = 0;
static volatile unsigned int g_httpRequests = 0;
static volatile unsigned int g_obstacleDistanceCm = DEFAULT_OBSTACLE_DISTANCE_CM;
static volatile unsigned int g_collisionDistanceCm = DEFAULT_COLLISION_DISTANCE_CM;
static volatile unsigned int g_remoteTimeoutMs = DEFAULT_REMOTE_TIMEOUT_MS;
static volatile unsigned int g_lastRemoteMs = 0;
static volatile unsigned char g_linkLostProtect = 0;
static volatile unsigned char g_collisionProtect = 0;
static volatile unsigned char g_radarEnabled = 0;
static volatile unsigned char g_radarIndex = 0;
static volatile unsigned int g_lastRadarMs = 0;
static volatile int g_radarDistanceCm[RADAR_POINTS] = {-1, -1, -1, -1, -1};
static KeyId g_lastScanKey = KEY_NONE;
static IotGpioValue g_traceLeftBlackLevel = IOT_GPIO_VALUE1;
static IotGpioValue g_traceRightBlackLevel = IOT_GPIO_VALUE1;
static IotGpioValue g_traceWhiteLeft = IOT_GPIO_VALUE0;
static IotGpioValue g_traceWhiteRight = IOT_GPIO_VALUE0;
static IotGpioValue g_traceBlackLeft = IOT_GPIO_VALUE1;
static IotGpioValue g_traceBlackRight = IOT_GPIO_VALUE1;
static volatile unsigned char g_traceWhiteValid = 0;
static volatile unsigned char g_traceBlackValid = 0;

static const unsigned int RADAR_SERVO_US[RADAR_POINTS] = {
    SERVO_LEFT_US, 1250, SERVO_CENTER_US, 1750, SERVO_RIGHT_US
};
static const int RADAR_ANGLE_DEG[RADAR_POINTS] = {-60, -30, 0, 30, 60};

static void OledWriteByte(unsigned char control, unsigned char data)
{
    unsigned char buffer[2] = { control, data };
    IoTI2cWrite(OLED_I2C_IDX, OLED_ADDR, buffer, sizeof(buffer));
}

static void OledWriteCmd(unsigned char cmd)
{
    OledWriteByte(0x00, cmd);
}

static void OledWriteData(unsigned char data)
{
    OledWriteByte(0x40, data);
}

static void OledSetPos(unsigned char x, unsigned char page)
{
    OledWriteCmd(0xB0 + page);
    OledWriteCmd(((x & 0xF0) >> 4) | 0x10);
    OledWriteCmd(x & 0x0F);
}

static void OledClear(void)
{
    for (unsigned char page = 0; page < OLED_PAGES; page++) {
        OledSetPos(0, page);
        for (unsigned char x = 0; x < OLED_WIDTH; x++) {
            OledWriteData(0x00);
        }
    }
}

static void OledInit(void)
{
    OledWriteCmd(0xAE);
    OledWriteCmd(0x20);
    OledWriteCmd(0x10);
    OledWriteCmd(0xB0);
    OledWriteCmd(0xC8);
    OledWriteCmd(0x00);
    OledWriteCmd(0x10);
    OledWriteCmd(0x40);
    OledWriteCmd(0x81);
    OledWriteCmd(0x7F);
    OledWriteCmd(0xA1);
    OledWriteCmd(0xA6);
    OledWriteCmd(0xA8);
    OledWriteCmd(0x3F);
    OledWriteCmd(0xA4);
    OledWriteCmd(0xD3);
    OledWriteCmd(0x00);
    OledWriteCmd(0xD5);
    OledWriteCmd(0x80);
    OledWriteCmd(0xD9);
    OledWriteCmd(0xF1);
    OledWriteCmd(0xDA);
    OledWriteCmd(0x12);
    OledWriteCmd(0xDB);
    OledWriteCmd(0x40);
    OledWriteCmd(0x8D);
    OledWriteCmd(0x14);
    OledWriteCmd(0xAF);
}

static const unsigned char *GetFont5x7(char c)
{
    static const unsigned char SPACE[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const unsigned char EXCL[5] = {0x00, 0x00, 0x5F, 0x00, 0x00};
    static const unsigned char PCT[5] = {0x63, 0x13, 0x08, 0x64, 0x63};
    static const unsigned char DASH[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const unsigned char DOT[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const unsigned char SLASH[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
    static const unsigned char COLON[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const unsigned char NUM0[5] = {0x3E, 0x51, 0x49, 0x45, 0x3E};
    static const unsigned char NUM1[5] = {0x00, 0x42, 0x7F, 0x40, 0x00};
    static const unsigned char NUM2[5] = {0x42, 0x61, 0x51, 0x49, 0x46};
    static const unsigned char NUM3[5] = {0x21, 0x41, 0x45, 0x4B, 0x31};
    static const unsigned char NUM4[5] = {0x18, 0x14, 0x12, 0x7F, 0x10};
    static const unsigned char NUM5[5] = {0x27, 0x45, 0x45, 0x45, 0x39};
    static const unsigned char NUM6[5] = {0x3C, 0x4A, 0x49, 0x49, 0x30};
    static const unsigned char NUM7[5] = {0x01, 0x71, 0x09, 0x05, 0x03};
    static const unsigned char NUM8[5] = {0x36, 0x49, 0x49, 0x49, 0x36};
    static const unsigned char NUM9[5] = {0x06, 0x49, 0x49, 0x29, 0x1E};
    static const unsigned char A[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
    static const unsigned char B[5] = {0x7F, 0x49, 0x49, 0x49, 0x36};
    static const unsigned char C[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
    static const unsigned char D[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
    static const unsigned char E[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
    static const unsigned char F[5] = {0x7F, 0x09, 0x09, 0x09, 0x01};
    static const unsigned char G[5] = {0x3E, 0x41, 0x49, 0x49, 0x7A};
    static const unsigned char H[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
    static const unsigned char I[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
    static const unsigned char J[5] = {0x20, 0x40, 0x41, 0x3F, 0x01};
    static const unsigned char K[5] = {0x7F, 0x08, 0x14, 0x22, 0x41};
    static const unsigned char L[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
    static const unsigned char M[5] = {0x7F, 0x02, 0x0C, 0x02, 0x7F};
    static const unsigned char N[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F};
    static const unsigned char O[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
    static const unsigned char P[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
    static const unsigned char Q[5] = {0x3E, 0x41, 0x51, 0x21, 0x5E};
    static const unsigned char R[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
    static const unsigned char S[5] = {0x46, 0x49, 0x49, 0x49, 0x31};
    static const unsigned char T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
    static const unsigned char U[5] = {0x3F, 0x40, 0x40, 0x40, 0x3F};
    static const unsigned char V[5] = {0x1F, 0x20, 0x40, 0x20, 0x1F};
    static const unsigned char W[5] = {0x7F, 0x20, 0x18, 0x20, 0x7F};
    static const unsigned char X[5] = {0x63, 0x14, 0x08, 0x14, 0x63};
    static const unsigned char Y[5] = {0x07, 0x08, 0x70, 0x08, 0x07};
    static const unsigned char Z[5] = {0x61, 0x51, 0x49, 0x45, 0x43};
    if (c >= 'a' && c <= 'z') {
        c -= 'a' - 'A';
    }
    switch (c) {
        case '!': return EXCL;
        case '%': return PCT;
        case '-': return DASH;
        case '.': return DOT;
        case '/': return SLASH;
        case ':': return COLON;
        case '0': return NUM0;
        case '1': return NUM1;
        case '2': return NUM2;
        case '3': return NUM3;
        case '4': return NUM4;
        case '5': return NUM5;
        case '6': return NUM6;
        case '7': return NUM7;
        case '8': return NUM8;
        case '9': return NUM9;
        case 'A': return A;
        case 'B': return B;
        case 'C': return C;
        case 'D': return D;
        case 'E': return E;
        case 'F': return F;
        case 'G': return G;
        case 'H': return H;
        case 'I': return I;
        case 'J': return J;
        case 'K': return K;
        case 'L': return L;
        case 'M': return M;
        case 'N': return N;
        case 'O': return O;
        case 'P': return P;
        case 'Q': return Q;
        case 'R': return R;
        case 'S': return S;
        case 'T': return T;
        case 'U': return U;
        case 'V': return V;
        case 'W': return W;
        case 'X': return X;
        case 'Y': return Y;
        case 'Z': return Z;
        case ' ': return SPACE;
        default: return SPACE;
    }
}

typedef struct {
    unsigned int code;
    unsigned char data[32];
} Cn16Font;

static const Cn16Font CN16_FONT[] = {
    { 0x667A, { 0x10, 0x9C, 0xD7, 0x75, 0x3C, 0x54, 0xD4, 0x94, 0x84, 0x7C, 0x44, 0x44, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x7F, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00 } },
    { 0x80FD, { 0x10, 0xDC, 0x56, 0x53, 0x54, 0xDC, 0xD8, 0x10, 0xBF, 0x28, 0x24, 0x24, 0x22, 0x30, 0x00, 0x00, 0x00, 0x7F, 0x09, 0x09, 0x29, 0x3F, 0x3F, 0x00, 0x3F, 0x26, 0x22, 0x22, 0x21, 0x30, 0x00, 0x00 } },
    { 0x5C0F, { 0x00, 0x00, 0xE0, 0x30, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x30, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x04, 0x07, 0x01, 0x00, 0x00, 0x20, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00 } },
    { 0x8F66, { 0x00, 0x04, 0xC4, 0xA4, 0x9C, 0x8E, 0x87, 0xF4, 0x84, 0x84, 0x84, 0x84, 0x84, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x7F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00 } },
    { 0x6309, { 0x08, 0x08, 0xFF, 0x88, 0x80, 0x9C, 0x9C, 0x84, 0xF5, 0x97, 0x84, 0x84, 0x84, 0x9C, 0x00, 0x00, 0x03, 0x21, 0x3F, 0x00, 0x20, 0x20, 0x26, 0x27, 0x1C, 0x18, 0x1C, 0x13, 0x30, 0x20, 0x00, 0x00 } },
    { 0x952E, { 0x18, 0xE6, 0xE7, 0x24, 0x46, 0x76, 0xCE, 0x10, 0x52, 0x52, 0xFF, 0x52, 0x7E, 0x10, 0x00, 0x00, 0x01, 0x3F, 0x1F, 0x0D, 0x6A, 0x3E, 0x1F, 0x30, 0x25, 0x25, 0x3F, 0x25, 0x25, 0x24, 0x00, 0x00 } },
    { 0x6A21, { 0x08, 0xC8, 0xFF, 0x48, 0x88, 0x02, 0xF2, 0x57, 0x57, 0x52, 0x57, 0x57, 0xF2, 0x02, 0x00, 0x00, 0x03, 0x01, 0x3F, 0x00, 0x21, 0x24, 0x25, 0x15, 0x1D, 0x07, 0x0D, 0x15, 0x25, 0x64, 0x20, 0x00 } },
    { 0x5F0F, { 0x08, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x1F, 0xFF, 0x08, 0x0A, 0x0E, 0x08, 0x08, 0x00, 0x00, 0x10, 0x30, 0x10, 0x1F, 0x1F, 0x08, 0x08, 0x08, 0x01, 0x0F, 0x18, 0x30, 0x20, 0x38, 0x00, 0x00 } },
    { 0x5FAA, { 0x98, 0xCC, 0xEF, 0x11, 0x00, 0xFE, 0x0A, 0xCA, 0x4A, 0x7E, 0x4E, 0x4B, 0xC9, 0x08, 0x00, 0x00, 0x01, 0x00, 0x3F, 0x20, 0x3C, 0x03, 0x00, 0x3F, 0x15, 0x15, 0x15, 0x15, 0x3F, 0x00, 0x00, 0x00 } },
    { 0x8FF9, { 0x40, 0xCE, 0xCC, 0x00, 0xC8, 0x68, 0x08, 0xF9, 0x0F, 0x08, 0xF8, 0x48, 0xE8, 0x88, 0x00, 0x00, 0x30, 0x1F, 0x1F, 0x11, 0x31, 0x2C, 0x27, 0x21, 0x28, 0x28, 0x2F, 0x20, 0x20, 0x21, 0x00, 0x00 } },
    { 0x907F, { 0x62, 0xEE, 0x00, 0x80, 0xFE, 0x92, 0x92, 0x9E, 0x44, 0x7C, 0xD7, 0xC7, 0x7C, 0x44, 0x00, 0x00, 0x20, 0x1F, 0x14, 0x17, 0x2F, 0x2F, 0x28, 0x2F, 0x22, 0x22, 0x3F, 0x3F, 0x22, 0x22, 0x00, 0x00 } },
    { 0x969C, { 0xFE, 0x02, 0x72, 0xDE, 0x02, 0x10, 0xD4, 0x5C, 0x56, 0x55, 0x5C, 0x5C, 0xD4, 0x14, 0x00, 0x00, 0x3F, 0x00, 0x04, 0x04, 0x13, 0x10, 0x13, 0x15, 0x15, 0x7D, 0x15, 0x15, 0x13, 0x10, 0x00, 0x00 } },
    { 0x53C2, { 0x20, 0x20, 0x28, 0xA8, 0x6C, 0x7A, 0x39, 0xA8, 0x2A, 0x6C, 0xAC, 0x38, 0x20, 0x20, 0x00, 0x00, 0x03, 0x21, 0x21, 0x2A, 0x2A, 0x2D, 0x25, 0x14, 0x13, 0x0A, 0x08, 0x05, 0x03, 0x01, 0x00, 0x00 } },
    { 0x6570, { 0x48, 0x6E, 0x3C, 0xFF, 0x7F, 0x2C, 0x4E, 0xC8, 0xFC, 0x0F, 0x08, 0xC8, 0x78, 0x08, 0x00, 0x00, 0x01, 0x25, 0x27, 0x19, 0x19, 0x0D, 0x13, 0x20, 0x30, 0x17, 0x0C, 0x17, 0x20, 0x20, 0x00, 0x00 } },
    { 0x578B, { 0x30, 0xB2, 0xF2, 0x3E, 0x32, 0x32, 0xFE, 0x32, 0x30, 0x7E, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x21, 0x21, 0x24, 0x24, 0x24, 0x24, 0x3F, 0x3E, 0x24, 0x24, 0x24, 0x25, 0x25, 0x20, 0x00, 0x00 } },
    { 0x53F7, { 0x40, 0x40, 0x40, 0x5E, 0xD2, 0x52, 0x52, 0x52, 0x52, 0x52, 0x5E, 0x5E, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x02, 0x02, 0x62, 0x22, 0x22, 0x3E, 0x0E, 0x00, 0x00, 0x00, 0x00 } },
    { 0x8DDD, { 0x00, 0x9E, 0x12, 0xF2, 0x1E, 0x1E, 0x00, 0xFE, 0x22, 0x22, 0x22, 0x22, 0xE2, 0x02, 0x00, 0x00, 0x30, 0x1F, 0x10, 0x1F, 0x11, 0x19, 0x00, 0x3F, 0x22, 0x22, 0x22, 0x22, 0x23, 0x20, 0x00, 0x00 } },
    { 0x79BB, { 0x02, 0x02, 0x7A, 0x62, 0x6A, 0x6A, 0xDB, 0x52, 0x5A, 0x6E, 0x62, 0x42, 0x72, 0x02, 0x00, 0x00, 0x00, 0x7E, 0x02, 0x0A, 0x1E, 0x16, 0x13, 0x0A, 0x0A, 0x0E, 0x1A, 0x62, 0x3E, 0x3E, 0x00, 0x00 } },
    { 0x901F, { 0x40, 0xC6, 0xCC, 0x00, 0x86, 0xF6, 0x96, 0x96, 0xFF, 0x96, 0x96, 0x96, 0xF6, 0x06, 0x00, 0x00, 0x60, 0x1F, 0x1F, 0x10, 0x28, 0x24, 0x22, 0x21, 0x2F, 0x20, 0x23, 0x22, 0x24, 0x24, 0x00, 0x00 } },
    { 0x5EA6, { 0x00, 0xFC, 0x04, 0x24, 0x24, 0xFC, 0xA4, 0xA7, 0xA4, 0xA4, 0xFC, 0x24, 0x24, 0x00, 0x00, 0x00, 0x30, 0x1F, 0x00, 0x42, 0x22, 0x26, 0x2E, 0x1A, 0x12, 0x2A, 0x26, 0x22, 0x20, 0x60, 0x00, 0x00 } },
    { 0x7535, { 0x00, 0xF8, 0x48, 0x48, 0x48, 0x48, 0xFF, 0x48, 0x48, 0x48, 0x48, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x06, 0x06, 0x06, 0x06, 0x3F, 0x26, 0x26, 0x26, 0x26, 0x27, 0x20, 0x30, 0x00, 0x00 } },
    { 0x91CF, { 0x20, 0x20, 0xBE, 0xB6, 0xB6, 0xB6, 0xB6, 0xB6, 0xB6, 0xB6, 0xB6, 0xBE, 0x20, 0x20, 0x00, 0x00, 0x20, 0x28, 0x2F, 0x2D, 0x2D, 0x2D, 0x3F, 0x3F, 0x2D, 0x2D, 0x2D, 0x2F, 0x20, 0x20, 0x00, 0x00 } },
    { 0x4F4E, { 0xC0, 0x30, 0xFE, 0x03, 0x00, 0xFC, 0x44, 0x44, 0x42, 0xFE, 0x42, 0x42, 0x42, 0x40, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x3F, 0x18, 0x1C, 0x20, 0x03, 0x0E, 0x30, 0x20, 0x38, 0x00, 0x00 } },
    { 0x505C, { 0x60, 0x18, 0xFF, 0x01, 0x82, 0xBA, 0xAA, 0xAA, 0xAB, 0xAB, 0xAA, 0xBA, 0x82, 0x82, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x01, 0x02, 0x02, 0x62, 0x22, 0x3E, 0x02, 0x02, 0x02, 0x01, 0x00, 0x00 } },
    { 0x6B62, { 0x00, 0x00, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0xFF, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x30, 0x30, 0x3F, 0x3F, 0x30, 0x30, 0x30, 0x3F, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00 } },
};

static const unsigned char *FindCn16Font(unsigned int code)
{
    for (unsigned int i = 0; i < sizeof(CN16_FONT) / sizeof(CN16_FONT[0]); i++) {
        if (CN16_FONT[i].code == code) {
            return CN16_FONT[i].data;
        }
    }
    return NULL;
}

static unsigned int Utf8NextCode(const char **str)
{
    const unsigned char *s = (const unsigned char *)*str;
    if (s[0] == '\0') {
        return 0;
    }
    if (s[0] < 0x80) {
        *str += 1;
        return s[0];
    }
    if ((s[0] & 0xE0) == 0xC0 && s[1] != '\0') {
        *str += 2;
        return ((unsigned int)(s[0] & 0x1F) << 6) | (unsigned int)(s[1] & 0x3F);
    }
    if ((s[0] & 0xF0) == 0xE0 && s[1] != '\0' && s[2] != '\0') {
        *str += 3;
        return ((unsigned int)(s[0] & 0x0F) << 12) |
               ((unsigned int)(s[1] & 0x3F) << 6) |
               (unsigned int)(s[2] & 0x3F);
    }
    *str += 1;
    return ' ';
}

static unsigned char TextWidth(const char *str)
{
    unsigned char width = 0;
    while (*str != '\0') {
        width += 6;
        str++;
    }
    return width;
}

static void OledShowChar(unsigned char x, unsigned char page, char c)
{
    const unsigned char *font = GetFont5x7(c);
    OledSetPos(x, page);
    for (int i = 0; i < 5; i++) {
        OledWriteData(font[i]);
    }
    OledWriteData(0x00);
}

static void OledShowString(unsigned char x, unsigned char page, const char *str)
{
    while (*str != '\0' && page < OLED_PAGES) {
        OledShowChar(x, page, *str);
        x += 6;
        str++;
        if (x > 122) {
            x = 0;
            page++;
        }
    }
}

static unsigned int OledMixedTextWidth(const char *str)
{
    unsigned int width = 0;
    while (*str != '\0') {
        const char *next = str;
        unsigned int code = Utf8NextCode(&next);
        width += (code < 0x80) ? 6 : 16;
        str = next;
    }
    return width;
}

static void OledShowCn16(unsigned char x, unsigned char page, const unsigned char *font)
{
    if (font == NULL || page >= OLED_PAGES - 1 || x > OLED_WIDTH - 16) {
        return;
    }

    OledSetPos(x, page);
    for (unsigned char i = 0; i < 16; i++) {
        OledWriteData(font[i]);
    }
    OledSetPos(x, page + 1);
    for (unsigned char i = 16; i < 32; i++) {
        OledWriteData(font[i]);
    }
}

static void OledShowUtf8String(unsigned char x, unsigned char page, const char *str)
{
    while (*str != '\0' && page < OLED_PAGES) {
        unsigned int code = Utf8NextCode(&str);
        unsigned char width = (code < 0x80) ? 6 : 16;
        if (x + width > OLED_WIDTH) {
            x = 0;
            page += (width == 16) ? 2 : 1;
            if (page >= OLED_PAGES) {
                break;
            }
        }

        if (code < 0x80) {
            OledShowChar(x, page, (char)code);
        } else {
            OledShowCn16(x, page, FindCn16Font(code));
        }
        x += width;
    }
}

static void OledShowUtf8Centered(unsigned char page, const char *str)
{
    unsigned int width = OledMixedTextWidth(str);
    unsigned char x = (width >= OLED_WIDTH) ? 0 : (unsigned char)((OLED_WIDTH - width) / 2);
    OledShowUtf8String(x, page, str);
}

static void OledShowCentered(unsigned char page, const char *str)
{
    unsigned char width = TextWidth(str);
    unsigned char x = (width >= OLED_WIDTH) ? 0 : (OLED_WIDTH - width) / 2;
    OledShowString(x, page, str);
}

static void OledDrawProgress(unsigned char page, unsigned char width)
{
    unsigned char start = 24;
    if (width > 80) {
        width = 80;
    }
    OledSetPos(start, page);
    for (unsigned char i = 0; i < 80; i++) {
        OledWriteData((i < width) ? 0x18 : 0x00);
    }
}

static void OledBootAnimation(void)
{
    for (unsigned char i = 0; i <= 80; i += 16) {
        OledClear();
        OledShowCentered(3, "YYDS");
        OledDrawProgress(5, i);
        osDelay(15);
    }
    osDelay(80);
}

static void SetGpioOutput(unsigned int gpio, IotGpioValue value)
{
    hi_io_set_func((hi_io_name)gpio, GPIO_FUNC);
    IoTGpioSetDir(gpio, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(gpio, value);
}

static void SetUltrasonicTrigOutput(void)
{
    hi_io_set_func(HI_IO_NAME_GPIO_7, HI_IO_FUNC_GPIO_7_GPIO);
    IoTGpioSetDir(ULTRASONIC_TRIG, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(ULTRASONIC_TRIG, IOT_GPIO_VALUE0);
}

static void ServoSetPulseUsCount(unsigned int dutyUs, unsigned int count)
{
    if (dutyUs >= SERVO_PERIOD_US) {
        return;
    }

    IoTWatchDogDisable();
    hi_io_set_func(HI_IO_NAME_GPIO_2, HI_IO_FUNC_GPIO_2_GPIO);
    IoTGpioSetDir(SERVO_GPIO, IOT_GPIO_DIR_OUT);

    for (unsigned int i = 0; i < count; i++) {
        IoTGpioSetOutputVal(SERVO_GPIO, IOT_GPIO_VALUE1);
        hi_udelay(dutyUs);
        IoTGpioSetOutputVal(SERVO_GPIO, IOT_GPIO_VALUE0);
        hi_udelay(SERVO_PERIOD_US - dutyUs);
    }
}

static void ServoSetPulseUs(unsigned int dutyUs)
{
    ServoSetPulseUsCount(dutyUs, SERVO_PULSE_COUNT);
}

static void ServoSetPulseUsFast(unsigned int dutyUs)
{
    ServoSetPulseUsCount(dutyUs, 2);
    osDelay(4);
}

static void ServoLeft(void)
{
    ServoSetPulseUs(SERVO_LEFT_US);
    osDelay(SERVO_SETTLE_TICKS);
}

static void ServoRight(void)
{
    ServoSetPulseUs(SERVO_RIGHT_US);
    osDelay(SERVO_SETTLE_TICKS);
}

static void ServoCenter(void)
{
    ServoSetPulseUs(SERVO_CENTER_US);
    osDelay(SERVO_SETTLE_TICKS);
}

static void CarStop(void)
{
    SetGpioOutput(MOTOR_LEFT_A, IOT_GPIO_VALUE1);
    SetGpioOutput(MOTOR_LEFT_B, IOT_GPIO_VALUE1);
    SetGpioOutput(MOTOR_RIGHT_A, IOT_GPIO_VALUE1);
    SetGpioOutput(MOTOR_RIGHT_B, IOT_GPIO_VALUE1);
    g_speedMps100 = 0;
}

static void CarForward(void)
{
    SetGpioOutput(MOTOR_LEFT_A, IOT_GPIO_VALUE1);
    SetGpioOutput(MOTOR_LEFT_B, IOT_GPIO_VALUE0);
    SetGpioOutput(MOTOR_RIGHT_A, IOT_GPIO_VALUE1);
    SetGpioOutput(MOTOR_RIGHT_B, IOT_GPIO_VALUE0);
    g_speedMps100 = 30;
}

static void CarBackward(void)
{
    SetGpioOutput(MOTOR_LEFT_A, IOT_GPIO_VALUE0);
    SetGpioOutput(MOTOR_LEFT_B, IOT_GPIO_VALUE1);
    SetGpioOutput(MOTOR_RIGHT_A, IOT_GPIO_VALUE0);
    SetGpioOutput(MOTOR_RIGHT_B, IOT_GPIO_VALUE1);
    g_speedMps100 = 20;
}

static void CarLeft(void)
{
    SetGpioOutput(MOTOR_LEFT_A, IOT_GPIO_VALUE0);
    SetGpioOutput(MOTOR_LEFT_B, IOT_GPIO_VALUE0);
    SetGpioOutput(MOTOR_RIGHT_A, IOT_GPIO_VALUE1);
    SetGpioOutput(MOTOR_RIGHT_B, IOT_GPIO_VALUE0);
    g_speedMps100 = 20;
}

static void CarRight(void)
{
    SetGpioOutput(MOTOR_LEFT_A, IOT_GPIO_VALUE1);
    SetGpioOutput(MOTOR_LEFT_B, IOT_GPIO_VALUE0);
    SetGpioOutput(MOTOR_RIGHT_A, IOT_GPIO_VALUE0);
    SetGpioOutput(MOTOR_RIGHT_B, IOT_GPIO_VALUE0);
    g_speedMps100 = 20;
}

static int IsCollisionDistance(void)
{
    return (g_lastDistanceCm > 0 && g_lastDistanceCm <= (int)g_collisionDistanceCm);
}

static void RaiseCollisionProtect(void)
{
    g_collisionProtect = 1;
    g_displayDirty = 1;
    CarStop();
}

static void ClearCollisionProtectIfSafe(int distance)
{
    if (distance > (int)(g_collisionDistanceCm + 2) && g_collisionProtect != 0) {
        g_collisionProtect = 0;
        g_displayDirty = 1;
    }
}

static void RemoteStep(void)
{
    unsigned int nowMs = hi_get_milli_seconds();
    if (g_remoteAction != REMOTE_STOP && nowMs - g_lastRemoteMs > g_remoteTimeoutMs) {
        g_linkLostProtect = 1;
        g_remoteAction = REMOTE_STOP;
        g_carMode = MODE_STOP;
        g_displayDirty = 1;
        CarStop();
        return;
    }

    if (g_remoteAction == REMOTE_FORWARD && IsCollisionDistance()) {
        g_remoteAction = REMOTE_STOP;
        RaiseCollisionProtect();
        return;
    }

    if (g_remoteAction == REMOTE_FORWARD) {
        CarForward();
    } else if (g_remoteAction == REMOTE_BACKWARD) {
        CarBackward();
    } else if (g_remoteAction == REMOTE_LEFT) {
        CarLeft();
    } else if (g_remoteAction == REMOTE_RIGHT) {
        CarRight();
    } else {
        CarStop();
    }
}

static void CarGpioInit(void)
{
    const unsigned int outPins[] = {
        MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_RIGHT_A, MOTOR_RIGHT_B, SERVO_GPIO
    };
    for (unsigned int i = 0; i < sizeof(outPins) / sizeof(outPins[0]); i++) {
        IoTGpioInit(outPins[i]);
        hi_io_set_func((hi_io_name)outPins[i], GPIO_FUNC);
        IoTGpioSetDir(outPins[i], IOT_GPIO_DIR_OUT);
    }

    IoTGpioInit(ULTRASONIC_ECHO);
    hi_io_set_func((hi_io_name)ULTRASONIC_ECHO, GPIO_FUNC);
    IoTGpioSetDir(ULTRASONIC_ECHO, IOT_GPIO_DIR_IN);

    IoTGpioInit(TRACE_LEFT);
    IoTGpioInit(TRACE_RIGHT);
    hi_io_set_func((hi_io_name)TRACE_LEFT, GPIO_FUNC);
    hi_io_set_func((hi_io_name)TRACE_RIGHT, GPIO_FUNC);
    IoTGpioSetDir(TRACE_LEFT, IOT_GPIO_DIR_IN);
    IoTGpioSetDir(TRACE_RIGHT, IOT_GPIO_DIR_IN);

    CarStop();
}

static void ReadTraceSensors(IotGpioValue *left, IotGpioValue *right)
{
    *left = IOT_GPIO_VALUE1;
    *right = IOT_GPIO_VALUE1;
    IoTGpioGetInputVal(TRACE_LEFT, left);
    IoTGpioGetInputVal(TRACE_RIGHT, right);
}

static void GetTraceView(IotGpioValue *left, IotGpioValue *right, int *seeLeft, int *seeRight)
{
    ReadTraceSensors(left, right);
    *seeLeft = (*left == g_traceLeftBlackLevel);
    *seeRight = (*right == g_traceRightBlackLevel);
}

static const char *TraceActionName(int seeLeft, int seeRight)
{
    if (seeLeft && seeRight) {
        return "FORWARD";
    }
    if (seeLeft && !seeRight) {
        return "LEFT";
    }
    if (!seeLeft && seeRight) {
        return "RIGHT";
    }
    return "STOP";
}

static void TraceStep(void)
{
    IotGpioValue left = IOT_GPIO_VALUE1;
    IotGpioValue right = IOT_GPIO_VALUE1;
    int seeLeft = 0;
    int seeRight = 0;

    GetTraceView(&left, &right, &seeLeft, &seeRight);

    if (IsCollisionDistance()) {
        RaiseCollisionProtect();
        return;
    }

    if (seeLeft && seeRight) {
        CarForward();
    } else if (seeLeft && !seeRight) {
        CarLeft();
    } else if (!seeLeft && seeRight) {
        CarRight();
    } else {
        CarStop();
    }
}

static int GetDistanceCm(void)
{
    IotGpioValue value = IOT_GPIO_VALUE0;
    unsigned long startTime = 0;
    unsigned long beginWait = hi_get_us();

    SetUltrasonicTrigOutput();
    IoTGpioSetOutputVal(ULTRASONIC_TRIG, IOT_GPIO_VALUE1);
    hi_udelay(20);
    IoTGpioSetOutputVal(ULTRASONIC_TRIG, IOT_GPIO_VALUE0);

    do {
        IoTGpioGetInputVal(ULTRASONIC_ECHO, &value);
        if (hi_get_us() - beginWait > DISTANCE_TIMEOUT_US) {
            return -1;
        }
    } while (value == IOT_GPIO_VALUE0);

    startTime = hi_get_us();
    do {
        IoTGpioGetInputVal(ULTRASONIC_ECHO, &value);
        if (hi_get_us() - startTime > DISTANCE_TIMEOUT_US) {
            return -1;
        }
    } while (value == IOT_GPIO_VALUE1);

    return (int)((hi_get_us() - startTime) * 34 / 2000);
}

static void ObstacleStep(void)
{
    int distance = GetDistanceCm();
    if (distance > 0) {
        g_lastDistanceCm = distance;
        g_radarDistanceCm[2] = distance;
        ClearCollisionProtectIfSafe(distance);
    }

    if (distance > 0 && distance <= (int)g_collisionDistanceCm) {
        g_collisionProtect = 1;
        g_displayDirty = 1;
    }

    if (distance > 0 && distance < (int)g_obstacleDistanceCm) {
        int leftDistance = -1;
        int rightDistance = -1;

        CarStop();
        osDelay(25);
        CarBackward();
        osDelay(45);
        CarStop();

        ServoLeft();
        leftDistance = GetDistanceCm();
        if (leftDistance > 0) {
            g_lastDistanceCm = leftDistance;
            g_radarDistanceCm[0] = leftDistance;
            g_radarDistanceCm[1] = leftDistance;
        }

        ServoRight();
        rightDistance = GetDistanceCm();
        if (rightDistance > 0) {
            g_lastDistanceCm = rightDistance;
            g_radarDistanceCm[3] = rightDistance;
            g_radarDistanceCm[4] = rightDistance;
        }

        ServoCenter();
        if (leftDistance > rightDistance) {
            CarLeft();
        } else {
            CarRight();
        }
        osDelay(45);
        CarForward();
    } else {
        CarForward();
    }
}

static void UpdateDistance(void)
{
    int distance = GetDistanceCm();
    if (distance > 0) {
        g_lastDistanceCm = distance;
        g_radarDistanceCm[2] = distance;
        ClearCollisionProtectIfSafe(distance);
    }
}

static void RadarScanStep(void)
{
    unsigned int nowMs = hi_get_milli_seconds();
    unsigned char index;
    int distance;

    if (g_radarEnabled == 0 || g_carMode == MODE_OBSTACLE) {
        return;
    }
    if (nowMs - g_lastRadarMs < RADAR_SCAN_INTERVAL_MS) {
        return;
    }
    g_lastRadarMs = nowMs;

    index = g_radarIndex;
    if (index >= RADAR_POINTS) {
        index = 0;
    }

    ServoSetPulseUsFast(RADAR_SERVO_US[index]);
    distance = GetDistanceCm();
    if (distance > 0) {
        g_radarDistanceCm[index] = distance;
        if (index == 2) {
            g_lastDistanceCm = distance;
            ClearCollisionProtectIfSafe(distance);
        }
    }

    index++;
    if (index >= RADAR_POINTS) {
        index = 0;
    }
    g_radarIndex = index;
}

static void ButtonInit(void)
{
    IoTGpioInit(DISPLAY_SWITCH_GPIO);
    hi_io_set_func(HI_IO_NAME_GPIO_5, HI_IO_FUNC_GPIO_5_GPIO);
    hi_io_set_pull(HI_IO_NAME_GPIO_5, HI_IO_PULL_UP);
    IoTGpioSetDir(DISPLAY_SWITCH_GPIO, IOT_GPIO_DIR_IN);
}

static unsigned int ReadDisplaySwitchMv(void)
{
    hi_u16 data = 0;
    unsigned int sumMv = 0;
    unsigned int count = 0;

    for (unsigned int i = 0; i < KEY_ADC_SAMPLES; i++) {
        if (hi_adc_read(KEY_ADC_CHANNEL, &data, HI_ADC_EQU_MODEL_4, HI_ADC_CUR_BAIS_DEFAULT, 0xF0) == 0) {
            unsigned int mv = (unsigned int)data * 7200 / 4096;
            sumMv += mv;
            count++;
        }
    }

    if (count == 0) {
        return 0xFFFFFFFF;
    }
    return sumMv / count;
}

static KeyId ScanDisplayKey(void)
{
    unsigned int mv = ReadDisplaySwitchMv();

    if (mv >= KEY1_MIN_MV && mv <= KEY1_MAX_MV) {
        return KEY_1;
    }
    if (mv >= KEY2_MIN_MV && mv <= KEY2_MAX_MV) {
        return KEY_2;
    }
    return KEY_NONE;
}

static CarMode NextCarMode(CarMode mode)
{
    if (mode == MODE_STOP) {
        return MODE_TRACE;
    }
    if (mode == MODE_TRACE) {
        return MODE_OBSTACLE;
    }
    return MODE_STOP;
}

static const char *ModeName(CarMode mode)
{
    if (mode == MODE_REMOTE) {
        return "REMOTE";
    }
    if (mode == MODE_TRACE) {
        return "TRACE";
    }
    if (mode == MODE_OBSTACLE) {
        return "OBSTACLE";
    }
    return "STOP";
}

static void SetCarMode(CarMode mode)
{
    g_carMode = mode;
    g_linkLostProtect = 0;
    if (mode != MODE_STOP) {
        g_collisionProtect = 0;
    }
    if (mode != MODE_REMOTE) {
        g_remoteAction = REMOTE_STOP;
    }
    if (mode == MODE_STOP) {
        CarStop();
    } else if (mode == MODE_OBSTACLE) {
        ServoCenter();
    }
    g_displayDirty = 1;
}

static void SetRemoteAction(RemoteAction action)
{
    g_remoteAction = action;
    g_lastRemoteMs = hi_get_milli_seconds();
    g_linkLostProtect = 0;
    if (action != REMOTE_FORWARD) {
        g_collisionProtect = 0;
    }
    if (action == REMOTE_STOP) {
        SetCarMode(MODE_STOP);
    } else {
        g_carMode = MODE_REMOTE;
        g_displayDirty = 1;
    }
}

static unsigned int ClampUnsigned(unsigned int value, unsigned int minValue, unsigned int maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static void ResetRuntimeParams(void)
{
    g_obstacleDistanceCm = DEFAULT_OBSTACLE_DISTANCE_CM;
    g_collisionDistanceCm = DEFAULT_COLLISION_DISTANCE_CM;
    g_remoteTimeoutMs = DEFAULT_REMOTE_TIMEOUT_MS;
    g_linkLostProtect = 0;
    g_collisionProtect = 0;
    g_displayDirty = 1;
}

static void HandleKey(KeyId key)
{
    if (key == KEY_2) {
        if (g_displayView == VIEW_PARAM) {
            g_paramPage = (ParamPage)((g_paramPage + 1) % PARAM_PAGE_COUNT);
        } else {
            g_paramPage = PARAM_PAGE_CAR;
        }
        g_displayView = VIEW_PARAM;
        g_displayDirty = 1;
        return;
    }

    if (key != KEY_1) {
        return;
    }

    SetCarMode(NextCarMode(g_carMode));
    g_displayView = VIEW_CONTROL;
    g_displayDirty = 1;
}

static void ProcessPendingKey(void)
{
    KeyId key = ScanDisplayKey();
    unsigned int now = hi_get_tick();

    if (key == KEY_NONE) {
        g_lastScanKey = KEY_NONE;
        return;
    }

    if (g_lastScanKey == KEY_NONE && now - g_lastKeyTick >= KEY_DEBOUNCE_TICKS) {
        g_lastKeyTick = now;
        g_lastScanKey = key;
        HandleKey(key);
    }
}

static unsigned char ReadBatteryPercent(void)
{
    return DEFAULT_BATTERY_PERCENT;
}

static void DrawControlScreen(void)
{
    char line[24];
    unsigned int mps100 = g_speedMps100;
    int distance = g_lastDistanceCm;

    g_batteryPercent = ReadBatteryPercent();

    OledClear();
    OledShowUtf8Centered(0, "智能小车");

    snprintf(line, sizeof(line), "MODE:%s", ModeName(g_carMode));
    OledShowString(0, 1, line);

    if (distance > 0) {
        snprintf(line, sizeof(line), "DIST:%03dCM", distance);
    } else {
        snprintf(line, sizeof(line), "DIST:--CM");
    }
    OledShowString(0, 2, line);

    snprintf(line, sizeof(line), "SPD:%u.%02uM/S", mps100 / 100, mps100 % 100);
    OledShowString(0, 3, line);

    snprintf(line, sizeof(line), "BAT:%u%% WIFI:%s", g_batteryPercent, g_wifiReady ? "ON" : "OFF");
    OledShowString(0, 4, line);

    snprintf(line, sizeof(line), "SET O%u C%u", g_obstacleDistanceCm, g_collisionDistanceCm);
    OledShowString(0, 5, line);

    if (g_linkLostProtect != 0) {
        OledShowString(0, 6, "SAFE:LINK LOST");
    } else if (g_collisionProtect != 0) {
        OledShowString(0, 6, "SAFE:COLLISION");
    } else if (g_batteryPercent < 20) {
        OledShowUtf8String(0, 6, "低电量!");
    } else {
        OledShowString(0, 6, "SAFE:OK");
    }

    OledShowString(0, 7, "S1 MODE  S2 PAGE");
}

static void DrawCarParamScreen(void)
{
    char line[24];
    unsigned int mps100 = g_speedMps100;
    unsigned int kmh100 = mps100 * 36 / 10;
    int distance = g_lastDistanceCm;

    g_batteryPercent = ReadBatteryPercent();

    OledClear();
    OledShowString(0, 0, "CAR 1/3");

    if (distance > 0) {
        snprintf(line, sizeof(line), "%03dCM", distance);
    } else {
        snprintf(line, sizeof(line), "--CM");
    }
    OledShowUtf8String(0, 2, "距离");
    OledShowString(36, 2, line);

    OledShowUtf8String(0, 4, "速度");
    snprintf(line, sizeof(line), "%u.%02uM/S", mps100 / 100, mps100 % 100);
    OledShowString(36, 4, line);
    snprintf(line, sizeof(line), "%u.%02uKM/H", kmh100 / 100, kmh100 % 100);
    OledShowString(36, 5, line);

    OledShowUtf8String(0, 6, "电量");
    snprintf(line, sizeof(line), "%u%%", g_batteryPercent);
    OledShowString(36, 6, line);
    if (g_batteryPercent < 20) {
        OledShowUtf8String(66, 6, "低电量!");
    } else {
        snprintf(line, sizeof(line), "MODE:%s", ModeName(g_carMode));
        OledShowString(0, 7, line);
    }
}

static void DrawNetParamScreen(void)
{
    char line[24];
    OledClear();
    OledShowString(0, 0, "NET 2/3");
    OledShowString(0, 2, g_wifiReady ? "WIFI:ON" : "WIFI:OFF");
    OledShowString(0, 3, "SSID:HMZXYYDS");
    OledShowString(0, 4, "PWD:HMZXYYDS");
    OledShowString(0, 5, "URL:" WIFI_AP_IP);
    snprintf(line, sizeof(line), "MODE:%s", ModeName(g_carMode));
    OledShowString(0, 6, line);
    OledShowString(0, 7, "S1 MODE S2 NEXT");
}

static void DrawSetParamScreen(void)
{
    char line[24];
    OledClear();
    OledShowString(0, 0, "SET 3/3");
    snprintf(line, sizeof(line), "OBS:%uCM", g_obstacleDistanceCm);
    OledShowString(0, 2, line);
    snprintf(line, sizeof(line), "COLL:%uCM", g_collisionDistanceCm);
    OledShowString(0, 3, line);
    snprintf(line, sizeof(line), "LOST:%uMS", g_remoteTimeoutMs);
    OledShowString(0, 4, line);
    snprintf(line, sizeof(line), "MODE:%s", ModeName(g_carMode));
    OledShowString(0, 5, line);
    OledShowString(0, 6, g_linkLostProtect ? "SAFE:LINK" : (g_collisionProtect ? "SAFE:COLL" : "SAFE:OK"));
    OledShowString(0, 7, "S1 MODE S2 NEXT");
}

static void DrawParamScreen(void)
{
    if (g_paramPage == PARAM_PAGE_SET) {
        DrawSetParamScreen();
    } else if (g_paramPage == PARAM_PAGE_NET) {
        DrawNetParamScreen();
    } else {
        DrawCarParamScreen();
    }
}

static void RefreshDisplay(void)
{
    if (g_displayView == VIEW_PARAM) {
        DrawParamScreen();
    } else {
        DrawControlScreen();
    }
    g_displayDirty = 0;
}

#if 0
static const char CONTROL_PAGE[] =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'>"
"<title>HMZXYYDS</title><style>"
"body{margin:0;background:#111;color:#f7f7f7;font-family:Arial,'Microsoft YaHei',sans-serif}"
".wrap{max-width:520px;margin:auto;padding:16px}.top{display:flex;justify-content:space-between;align-items:center}"
	".pill{border:1px solid #555;border-radius:999px;padding:6px 10px;color:#bdf}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:16px}"
	"button{height:58px;border:0;border-radius:10px;background:#2563eb;color:white;font-size:18px;font-weight:700}"
	"button.stop{background:#dc2626}.mode button{background:#374151;height:46px;font-size:15px}.card{background:#1f2937;border-radius:10px;padding:12px;margin-top:14px}"
	".kv{display:grid;grid-template-columns:1fr 1fr;gap:8px}.kv div{background:#111827;border-radius:8px;padding:10px}.v{font-size:20px;color:#7dd3fc}"
	".toggle{width:100%;height:46px;margin-top:14px;background:#0f766e}.joyctl{display:none;margin-top:16px;text-align:center}"
	".pad{width:210px;height:210px;border-radius:50%;background:#0f172a;border:2px solid #334155;position:relative;touch-action:none;margin:auto}"
	".stick{width:74px;height:74px;border-radius:50%;background:#38bdf8;position:absolute;left:68px;top:68px;box-shadow:0 8px 24px #0008}.hint{color:#9ca3af;margin-top:10px;font-size:14px}"
	".cfg{display:grid;grid-template-columns:1fr 1fr;gap:10px}.cfg label{font-size:13px;color:#cbd5e1}.cfg input{width:100%;box-sizing:border-box;margin-top:5px;background:#111827;color:white;border:1px solid #334155;border-radius:8px;padding:9px;font-size:16px}"
	".cfg button{height:42px;font-size:15px}.gray{background:#475569}.tiltctl{display:none;margin-top:14px;text-align:center}"
"</style></head><body><div class='wrap'>"
"<div class='top'><h2>HMZXYYDS 小车</h2><span class='pill' id='wifi'>连接中</span></div>"
"<div class='card kv'><div>模式<br><span class='v' id='mode'>--</span></div><div>延迟<br><span class='v' id='lat'>--</span></div>"
"<div>距离<br><span class='v' id='dist'>--</span></div><div>碰撞距离<br><span class='v' id='col'>--</span></div>"
"<div>速度 m/s<br><span class='v' id='mps'>--</span></div><div>速度 km/h<br><span class='v' id='kmh'>--</span></div>"
"<div>电量<br><span class='v' id='bat'>--</span></div><div>NFC<br><span class='v' id='nfc'>--</span></div></div>"
"<div class='card cfg'><label>避障距离 cm<input id='po' type='number' min='10' max='80'></label><label>碰撞距离 cm<input id='pc' type='number' min='5' max='30'></label><label>失联保护 ms<input id='pt' type='number' min='500' max='5000'></label><label>保护状态<br><span class='v' id='safe'>--</span></label><button onclick='saveParams()'>保存参数</button><button class='gray' onclick='resetParams()'>默认参数</button></div>"
"<button class='toggle' id='ctlBtn' onclick='toggleCtl()'>切换摇杆控制</button>"
"<div class='grid' id='btnCtl'>"
"<span></span><button data-m='forward'>前进</button><span></span>"
"<button data-m='left'>左转</button><button class='stop' data-m='stop'>停止</button><button data-m='right'>右转</button>"
"<span></span><button data-m='backward'>后退</button><span></span></div>"
"<div class='joyctl' id='joyCtl'><div class='pad' id='pad'><div class='stick' id='stick'></div></div><div class='hint' id='joyHint'>摇杆：松手停止</div></div>"
"<div class='tiltctl' id='tiltCtl'><button onclick='enableTilt()'>开启体感</button><div class='hint' id='tiltHint'>体感：前后左右倾斜手机，放平停止</div></div>"
"<div class='grid mode'><button onclick=\"setMode('stop')\">停止模式</button><button onclick=\"setMode('trace')\">循迹模式</button><button onclick=\"setMode('obstacle')\">避障模式</button></div>"
"<div class='card'>SSID: HMZXYYDS<br>Password: HMZXYYDS<br>控制页: http://" WIFI_AP_IP "</div>"
"</div><script>"
"var E=function(id){return document.getElementById(id)};"
"var wifi=E('wifi'),lat=E('lat'),modeText=E('mode'),dist=E('dist'),col=E('col'),mps=E('mps'),kmh=E('kmh'),bat=E('bat'),nfc=E('nfc'),safe=E('safe'),po=E('po'),pc=E('pc'),pt=E('pt');"
"var btnCtl=E('btnCtl'),joyCtl=E('joyCtl'),tiltCtl=E('tiltCtl'),ctlBtn=E('ctlBtn'),pad=E('pad'),stick=E('stick'),joyHint=E('joyHint'),tiltHint=E('tiltHint'),ctl='button',activeCmd='stop',joyDown=false,tiltCmd='stop';"
"function req(path,cb){var x=new XMLHttpRequest(),t=Date.now();x.timeout=900;x.onreadystatechange=function(){if(x.readyState===4){if(x.status===200){if(cb)cb(x.responseText,Date.now()-t)}else wifi.textContent='连接断开'}};x.ontimeout=x.onerror=function(){wifi.textContent='连接断开'};x.open('GET',path+(path.indexOf('?')>=0?'&':'?')+'t='+Date.now(),true);x.send();}"
"function go(m){req('/cmd?move='+m)}"
"function setDrive(m){activeCmd=m;go(m)}"
"function setMode(m){req('/mode?set='+m,function(){stat()})}"
"function saveParams(){req('/param?obstacle='+po.value+'&collision='+pc.value+'&timeout='+pt.value,function(){stat()})}"
"function resetParams(){req('/param?reset=1',function(){stat()})}"
"function showCtl(m){ctl=m;tiltCmd='stop';btnCtl.style.display=m==='button'?'grid':'none';joyCtl.style.display=m==='joy'?'block':'none';tiltCtl.style.display=m==='tilt'?'block':'none';ctlBtn.textContent=m==='button'?'切换摇杆控制':(m==='joy'?'切换体感控制':'切换按键控制');setDrive('stop')}"
"function toggleCtl(){showCtl(ctl==='button'?'joy':(ctl==='joy'?'tilt':'button'))}"
"document.querySelectorAll('[data-m]').forEach(function(b){var m=b.getAttribute('data-m');var down=function(e){e.preventDefault();setDrive(m)};var up=function(e){e.preventDefault();setDrive('stop')};b.addEventListener('touchstart',down);b.addEventListener('touchend',up);b.addEventListener('mousedown',down);b.addEventListener('mouseup',up);b.addEventListener('mouseleave',up);b.onclick=function(e){e.preventDefault()}});"
"function jp(e){var t=e.touches?e.touches[0]:e,r=pad.getBoundingClientRect();return{x:t.clientX-r.left-r.width/2,y:t.clientY-r.top-r.height/2}}"
"function joyMove(e){if(!joyDown)return;e.preventDefault();var p=jp(e),mx=68,d=Math.sqrt(p.x*p.x+p.y*p.y);if(d>mx){p.x=p.x*mx/d;p.y=p.y*mx/d;d=mx}stick.style.left=(68+p.x)+'px';stick.style.top=(68+p.y)+'px';var c='stop';if(d>26){c=Math.abs(p.x)>Math.abs(p.y)?(p.x>0?'right':'left'):(p.y>0?'backward':'forward')}if(c!==activeCmd)setDrive(c);joyHint.textContent='摇杆:'+c}"
"function joyStart(e){joyDown=true;joyMove(e)}"
"function joyEnd(e){if(!joyDown)return;joyDown=false;stick.style.left='68px';stick.style.top='68px';joyHint.textContent='摇杆：松手停止';setDrive('stop')}"
"pad.addEventListener('touchstart',joyStart);pad.addEventListener('touchmove',joyMove);pad.addEventListener('touchend',joyEnd);pad.addEventListener('mousedown',joyStart);document.addEventListener('mousemove',joyMove);document.addEventListener('mouseup',joyEnd);"
"function tiltPick(b,g){var c='stop';if(Math.abs(g)>18||Math.abs(b)>18)c=Math.abs(g)>Math.abs(b)?(g>0?'right':'left'):(b>0?'backward':'forward');return c}"
"function tiltRun(e){if(ctl!=='tilt')return;var c=tiltPick(e.beta||0,e.gamma||0);if(c!==tiltCmd){tiltCmd=c;setDrive(c)}tiltHint.textContent='体感:'+c}"
"function enableTilt(){if(window.DeviceOrientationEvent&&DeviceOrientationEvent.requestPermission){DeviceOrientationEvent.requestPermission().then(function(r){if(r==='granted')window.addEventListener('deviceorientation',tiltRun)})}else{window.addEventListener('deviceorientation',tiltRun)}tiltHint.textContent='体感已开启'}"
"function fill(id,v){if(document.activeElement!==id)id.value=v}"
"function stat(){req('/api/status',function(s,dt){var d=JSON.parse(s);lat.textContent=dt+'ms';wifi.textContent=d.wifi?'WiFi已开':'WiFi未开';modeText.textContent=d.mode;dist.textContent=d.distance>0?d.distance+'cm':'--';col.textContent=d.collision+'cm';mps.textContent=d.mps;kmh.textContent=d.kmh;bat.textContent=d.battery+'%';nfc.textContent=d.nfc?'OK':'WAIT';safe.textContent=d.safe;fill(po,d.obstacle);fill(pc,d.collision);fill(pt,d.timeout);})}"
"setInterval(stat,800);setInterval(function(){if(activeCmd!=='stop')go(activeCmd)},350);stat();</script></body></html>";

#endif

static const char CONTROL_PAGE_V205[] =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'>"
"<title>HMZXYYDS</title><style>"
"*{box-sizing:border-box}body{margin:0;background:#080c12;color:#eef6f5;font-family:'Trebuchet MS','Microsoft YaHei',sans-serif}"
".wrap{max-width:480px;margin:0 auto;padding:14px 12px 22px}.top{display:flex;justify-content:space-between;gap:10px;align-items:flex-start;margin-bottom:10px}"
".brand{font-size:22px;line-height:1.1;font-weight:800}.sub{margin-top:5px;color:#8ea0ad;font-size:12px}.pill{border:1px solid #2d4a5c;border-radius:999px;color:#8ee6d2;padding:6px 9px;font-size:12px;white-space:nowrap}"
".metrics{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}.metric,.card{background:#121a22;border:1px solid #263645;border-radius:8px;padding:11px;box-shadow:0 8px 18px #0005}"
".metric b{display:block;color:#90a4b3;font-size:12px;font-weight:500}.v{display:block;margin-top:4px;color:#72e6c9;font-size:21px;font-weight:800}.danger{color:#fb7185}.warn{color:#f59e0b}"
".card{margin-top:10px}.section{color:#cfe8e1;font-size:14px;font-weight:800;margin-bottom:8px}.dirgrid{display:grid;grid-template-columns:repeat(5,1fr);gap:6px}.dirgrid div{background:#0b1118;border:1px solid #243342;border-radius:8px;padding:7px 3px;text-align:center;font-size:12px;color:#9fb3c0}.dirgrid span{display:block;color:#fbbf24;font-size:15px;font-weight:800;margin-top:2px}"
"button{height:54px;border:0;border-radius:8px;background:#1d4ed8;color:white;font-size:17px;font-weight:800;touch-action:manipulation}.toolbar,.modebar{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:10px}.tool{height:42px;background:#0f766e;font-size:14px}.tool.amber{background:#a16207}.modebar button{height:42px;background:#1f2937;border:1px solid #334155;font-size:14px}.drive{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:12px}.drive .stop{background:#dc2626}.drive .blank{height:54px}.drive small{display:block;color:#dbeafe;font-size:11px;font-weight:500}"
".joyctl{display:none;margin-top:12px;text-align:center}.pad{width:220px;height:220px;border-radius:50%;background:#0b1118;border:2px solid #315064;position:relative;touch-action:none;margin:0 auto}.pad:before,.pad:after{content:'';position:absolute;background:#243342}.pad:before{left:109px;top:16px;width:2px;height:188px}.pad:after{left:16px;top:109px;width:188px;height:2px}.stick{width:76px;height:76px;border-radius:50%;background:#2dd4bf;position:absolute;left:72px;top:72px;box-shadow:0 8px 20px #0009}.hint{color:#90a4b3;margin-top:9px;font-size:13px}"
    ".panel{display:none}.cfg{grid-template-columns:1fr 1fr;gap:10px}.cfg label{font-size:13px;color:#cbd5e1}.cfg input{width:100%;margin-top:5px;background:#0b1118;color:white;border:1px solid #334155;border-radius:8px;padding:9px;font-size:16px}.cfg button{height:42px;font-size:14px}.gray{background:#475569}.radarBox canvas{width:100%;height:auto;background:#06100d;border:1px solid #294358;border-radius:8px}"
    ".traceTop{display:grid;grid-template-columns:1fr 1fr;gap:8px}.eye{background:#0b1118;border:1px solid #243342;border-radius:8px;padding:10px;text-align:center}.eye span{display:block;font-size:26px;font-weight:900;margin-top:3px}.eye.black{border-color:#72e6c9}.eye.black span{color:#72e6c9}.eye.white span{color:#e5e7eb}.track{height:86px;margin-top:10px;border-radius:8px;background:linear-gradient(90deg,#e5e7eb 0 18%,#111 18% 82%,#e5e7eb 82%);position:relative;border:1px solid #334155}.probe{position:absolute;top:18px;width:30px;height:50px;border-radius:8px;background:#f59e0b;border:2px solid #fde68a}.probe.l{left:33%}.probe.r{right:33%}.probe.hit{background:#14b8a6;border-color:#99f6e4}.steps{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:10px}.steps button{height:44px;font-size:13px;background:#334155}.steps button.ok{background:#0f766e}.traceLine{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:9px;color:#90a4b3;font-size:12px}.foot{color:#718392;font-size:12px;margin-top:10px;text-align:center}"
"</style></head><body><div class='wrap'>"
"<div class='top'><div><div class='brand'>HMZXYYDS 智能小车</div><div class='sub'>AP: HMZXYYDS | http://" WIFI_AP_IP "</div></div><span class='pill' id='wifi'>连接中</span></div>"
"<div class='metrics'><div class='metric'><b>当前模式</b><span class='v' id='mode'>--</span></div><div class='metric'><b>连接延迟</b><span class='v' id='lat'>--</span></div>"
"<div class='metric'><b>前方距离</b><span class='v' id='dist'>--</span></div><div class='metric'><b>电量 / 输入</b><span class='v' id='bat'>--</span></div>"
"<div class='metric'><b>速度 m/s</b><span class='v' id='mps'>--</span></div><div class='metric'><b>速度 km/h</b><span class='v' id='kmh'>--</span></div></div>"
"<div class='card'><div class='section'>方向距离 cm</div><div class='dirgrid'><div>左60<span id='rd0'>--</span></div><div>左30<span id='rd1'>--</span></div><div>前方<span id='rd2'>--</span></div><div>右30<span id='rd3'>--</span></div><div>右60<span id='rd4'>--</span></div></div></div>"
    "<div class='toolbar'><button class='tool' id='ctlBtn' onclick='toggleCtl()'>切换摇杆</button><button class='tool' id='paramBtn' onclick='toggleParams()'>打开参数</button><button class='tool amber' id='radarBtn' onclick='toggleRadar()'>打开雷达</button><button class='tool' id='traceBtn' onclick='toggleTrace()'>循迹校准</button></div>"
"<div class='modebar'><button onclick=\"setMode('stop')\">停止模式</button><button onclick=\"setMode('trace')\">循迹模式</button><button onclick=\"setMode('obstacle')\">避障模式</button></div>"
"<div class='drive' id='btnCtl'><span class='blank'></span><button data-m='forward'>↑<small>前进</small></button><span class='blank'></span>"
"<button data-m='left'>←<small>左转</small></button><button class='stop' data-m='stop'>■<small>停止</small></button><button data-m='right'>→<small>右转</small></button>"
"<span class='blank'></span><button data-m='backward'>↓<small>后退</small></button><span class='blank'></span></div>"
    "<div class='joyctl' id='joyCtl'><div class='pad' id='pad'><div class='stick' id='stick'></div></div><div class='hint' id='joyHint'>摇杆：松手停止</div></div>"
    "<div class='card panel radarBox' id='radarPanel'><div class='section'>超声波雷达扫描</div><canvas id='radar' width='320' height='210'></canvas><div class='hint' id='radarHint'>雷达打开后会左右扫描，距离越近颜色越危险</div></div>"
    "<div class='card panel' id='tracePanel'><div class='section'>循迹模块视角</div><div class='traceTop'><div class='eye' id='tlBox'>左探头<span id='tlSee'>--</span><small id='tlRaw'>GPIO --</small></div><div class='eye' id='trBox'>右探头<span id='trSee'>--</span><small id='trRaw'>GPIO --</small></div></div><div class='track'><i class='probe l' id='pl'></i><i class='probe r' id='pr'></i></div><div class='traceLine'><div>建议动作 <b id='ta'>--</b></div><div>黑线电平 L/R <b id='tbl'>--</b>/<b id='tbr'>--</b></div><div>白地样本 <b id='tw'>未记录</b></div><div>黑线样本 <b id='tb'>未记录</b></div></div><div class='steps'><button id='whiteBtn' onclick='traceMark(\"white\")'>1 白地</button><button id='blackBtn' onclick='traceMark(\"black\")'>2 黑线</button><button id='applyBtn' onclick='traceMark(\"apply\")'>3 应用</button></div></div>"
    "<div class='card panel cfg' id='paramPanel'><label>避障距离 cm<input id='po' type='number' min='10' max='80'></label><label>碰撞距离 cm<input id='pc' type='number' min='5' max='30'></label><label>失联保护 ms<input id='pt' type='number' min='500' max='5000'></label><label>保护状态<br><span class='v' id='safe'>--</span></label><button onclick='saveParams()'>保存参数</button><button class='gray' onclick='resetParams()'>默认参数</button></div>"
    "<div class='foot'>SSID HMZXYYDS / PASSWORD HMZXYYDS / V2.08</div>"
"</div><script>"
"var E=function(id){return document.getElementById(id)};"
    "var wifi=E('wifi'),lat=E('lat'),modeText=E('mode'),dist=E('dist'),mps=E('mps'),kmh=E('kmh'),bat=E('bat'),safe=E('safe'),po=E('po'),pc=E('pc'),pt=E('pt');"
    "var btnCtl=E('btnCtl'),joyCtl=E('joyCtl'),ctlBtn=E('ctlBtn'),paramBtn=E('paramBtn'),paramPanel=E('paramPanel'),radarBtn=E('radarBtn'),radarPanel=E('radarPanel'),traceBtn=E('traceBtn'),tracePanel=E('tracePanel'),pad=E('pad'),stick=E('stick'),joyHint=E('joyHint'),radar=E('radar'),ctx=radar.getContext('2d');"
    "var ctl='button',activeCmd='stop',joyDown=false,radarOpen=false,traceOpen=false,lastRadar=[],lastCollision=8;"
"function req(path,cb){var x=new XMLHttpRequest(),t=Date.now();x.timeout=900;x.onreadystatechange=function(){if(x.readyState===4){if(x.status===200){if(cb)cb(x.responseText,Date.now()-t)}else wifi.textContent='连接断开'}};x.ontimeout=x.onerror=function(){wifi.textContent='连接断开'};x.open('GET',path+(path.indexOf('?')>=0?'&':'?')+'t='+Date.now(),true);x.send();}"
"function go(m){req('/cmd?move='+m)}function setDrive(m){activeCmd=m;go(m)}function setMode(m){req('/mode?set='+m,function(){stat()})}"
"function saveParams(){req('/param?obstacle='+po.value+'&collision='+pc.value+'&timeout='+pt.value,function(){stat()})}function resetParams(){req('/param?reset=1',function(){stat()})}"
"function showCtl(m){ctl=m;btnCtl.style.display=m==='button'?'grid':'none';joyCtl.style.display=m==='joy'?'block':'none';ctlBtn.textContent=m==='button'?'切换摇杆':'切换按键';setDrive('stop')}function toggleCtl(){showCtl(ctl==='button'?'joy':'button')}"
    "function toggleParams(){var open=paramPanel.style.display==='grid';paramPanel.style.display=open?'none':'grid';paramBtn.textContent=open?'打开参数':'关闭参数'}"
    "function toggleRadar(){radarOpen=!radarOpen;radarPanel.style.display=radarOpen?'block':'none';radarBtn.textContent=radarOpen?'关闭雷达':'打开雷达';req('/radar?on='+(radarOpen?1:0),function(){stat()});if(radarOpen)drawRadar(lastRadar,true)}"
    "function toggleTrace(){traceOpen=!traceOpen;tracePanel.style.display=traceOpen?'block':'none';traceBtn.textContent=traceOpen?'关闭校准':'循迹校准';if(traceOpen)stat()}"
    "function traceMark(m){var q=m==='white'?'white=1':(m==='black'?'black=1':'apply=1');req('/trace?'+q,function(){stat()})}"
"document.querySelectorAll('[data-m]').forEach(function(b){var m=b.getAttribute('data-m');var down=function(e){e.preventDefault();setDrive(m)};var up=function(e){e.preventDefault();setDrive('stop')};b.addEventListener('touchstart',down);b.addEventListener('touchend',up);b.addEventListener('touchcancel',up);b.addEventListener('mousedown',down);b.addEventListener('mouseup',up);b.addEventListener('mouseleave',up);b.onclick=function(e){e.preventDefault()}});"
"function jp(e){var t=e.touches?e.touches[0]:e,r=pad.getBoundingClientRect();return{x:t.clientX-r.left-r.width/2,y:t.clientY-r.top-r.height/2}}"
"function joyMove(e){if(!joyDown)return;e.preventDefault();var p=jp(e),mx=72,d=Math.sqrt(p.x*p.x+p.y*p.y);if(d>mx){p.x=p.x*mx/d;p.y=p.y*mx/d;d=mx}stick.style.left=(72+p.x)+'px';stick.style.top=(72+p.y)+'px';var c='stop';if(d>26){c=Math.abs(p.x)>Math.abs(p.y)?(p.x>0?'right':'left'):(p.y>0?'backward':'forward')}if(c!==activeCmd)setDrive(c);joyHint.textContent='摇杆:'+c}"
"function joyStart(e){joyDown=true;joyMove(e)}function joyEnd(e){if(!joyDown)return;joyDown=false;stick.style.left='72px';stick.style.top='72px';joyHint.textContent='摇杆：松手停止';setDrive('stop')}"
"pad.addEventListener('touchstart',joyStart);pad.addEventListener('touchmove',joyMove);pad.addEventListener('touchend',joyEnd);pad.addEventListener('touchcancel',joyEnd);pad.addEventListener('mousedown',joyStart);document.addEventListener('mousemove',joyMove);document.addEventListener('mouseup',joyEnd);"
    "function fill(id,v){if(document.activeElement!==id)id.value=v}function dt(v){return v>0?v+'cm':'--'}function putDirs(rs){for(var i=0;i<5;i++){E('rd'+i).textContent=dt(rs&&rs[i]?rs[i].d:-1)}}"
    "function putEye(box,txt,raw,hit){box.className='eye '+(hit?'black':'white');box.querySelector('span').textContent=txt;box.querySelector('small').textContent='GPIO '+raw}"
    "function putTrace(t){if(!t)return;putEye(E('tlBox'),t.seeLeft?'黑':'白',t.left,t.seeLeft);putEye(E('trBox'),t.seeRight?'黑':'白',t.right,t.seeRight);E('pl').className='probe l '+(t.seeLeft?'hit':'');E('pr').className='probe r '+(t.seeRight?'hit':'');E('ta').textContent=t.action;E('tbl').textContent=t.leftBlack;E('tbr').textContent=t.rightBlack;E('tw').textContent=t.whiteValid?('L'+t.whiteLeft+' R'+t.whiteRight):'未记录';E('tb').textContent=t.blackValid?('L'+t.blackLeft+' R'+t.blackRight):'未记录';E('whiteBtn').className=t.whiteValid?'ok':'';E('blackBtn').className=t.blackValid?'ok':'';E('applyBtn').className=(t.whiteValid&&t.blackValid)?'ok':''}"
"function drawRadar(rs,on){var w=radar.width,h=radar.height,cx=w/2,cy=h-14,r=148;ctx.clearRect(0,0,w,h);ctx.fillStyle='#06100d';ctx.fillRect(0,0,w,h);ctx.strokeStyle='#214034';ctx.lineWidth=1;for(var a=30;a<=150;a+=30){var rad=a*Math.PI/180;ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx+Math.cos(rad)*r,cy-Math.sin(rad)*r);ctx.stroke()}for(var rr=40;rr<=r;rr+=36){ctx.beginPath();ctx.arc(cx,cy,rr,Math.PI,0);ctx.stroke()}if(on){var sw=((Date.now()/18)%120-60)*Math.PI/180;ctx.strokeStyle='rgba(114,230,201,.85)';ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx+Math.sin(sw)*r,cy-Math.cos(sw)*r);ctx.stroke()}if(rs){for(var i=0;i<rs.length;i++){var d=rs[i].d;if(d<=0)continue;var rr=Math.min(d,120)/120*r,a=rs[i].a*Math.PI/180,x=cx+Math.sin(a)*rr,y=cy-Math.cos(a)*rr;ctx.fillStyle=d<=lastCollision?'#fb7185':(d<30?'#fbbf24':'#72e6c9');ctx.beginPath();ctx.arc(x,y,5,0,Math.PI*2);ctx.fill();ctx.fillText(d+'cm',x+6,y-6)}}ctx.fillStyle='#8ea0ad';ctx.font='12px Trebuchet MS';ctx.fillText('0cm',cx-13,cy-3);ctx.fillText('120cm',cx+r-43,cy-3)}"
    "function stat(){req('/api/status',function(s,dtm){var d=JSON.parse(s);lastCollision=d.collision;lat.textContent=dtm+'ms';wifi.textContent=d.wifi?'热点已开 '+d.clients+'人':'热点未开';modeText.textContent=d.mode;dist.textContent=d.distance>0?d.distance+'cm':'--';mps.textContent=d.mps;kmh.textContent=d.kmh;bat.textContent=d.battery+'% '+(d.bat||'');bat.className=d.battery<20?'v danger':'v';safe.textContent=d.safe;fill(po,d.obstacle);fill(pc,d.collision);fill(pt,d.timeout);lastRadar=d.radar||[];putDirs(lastRadar);putTrace(d.trace);if(radarOpen)drawRadar(lastRadar,d.radarOn)})}"
    "setInterval(stat,800);setInterval(function(){if(activeCmd!=='stop')go(activeCmd);if(radarOpen)drawRadar(lastRadar,true);if(traceOpen)stat()},350);stat();</script></body></html>";

static const char CONTROL_PAGE_DEMO[] =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'>"
"<title>HiSpark Smart Car Demo</title><style>"
"*{box-sizing:border-box}body{margin:0;background:#10151a;color:#f4f7f6;font-family:'Trebuchet MS','Microsoft YaHei',sans-serif}"
".wrap{max-width:460px;margin:0 auto;padding:14px 12px 20px}.top{display:flex;justify-content:space-between;gap:10px;align-items:flex-start}"
".brand{font-size:24px;line-height:1.05;font-weight:900}.sub{margin-top:5px;color:#93a4af;font-size:12px}.pill{border:1px solid #36536a;border-radius:999px;color:#7ee3c4;padding:6px 10px;font-size:12px;white-space:nowrap}"
".hero{margin-top:12px;padding:13px;border:1px solid #2c3f4c;border-radius:8px;background:#151d24}.hero b{display:block;color:#94aeba;font-size:12px;font-weight:600}.mode{font-size:32px;line-height:1.1;font-weight:900;color:#7ee3c4;margin-top:4px}"
".metrics{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;margin-top:8px}.metric,.card{background:#151d24;border:1px solid #293a45;border-radius:8px;padding:10px}.metric b{display:block;color:#93a4af;font-size:12px;font-weight:600}.v{display:block;margin-top:4px;color:#f4c95d;font-size:20px;font-weight:900}"
".dirs{display:grid;grid-template-columns:repeat(5,1fr);gap:6px}.dirs div{background:#0c1116;border:1px solid #263642;border-radius:8px;text-align:center;padding:7px 3px;color:#9fb0ba;font-size:12px}.dirs span{display:block;margin-top:3px;color:#7ee3c4;font-size:15px;font-weight:900}"
".section{font-size:14px;font-weight:900;margin-bottom:8px;color:#dbe9e5}.modebar,.drive{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:10px}button{height:50px;border:0;border-radius:8px;background:#1d4ed8;color:white;font-size:16px;font-weight:900;touch-action:manipulation}.modebar button{height:42px;background:#26313b;border:1px solid #3d4c59;font-size:14px}.drive .blank{height:50px}.drive small{display:block;color:#dbeafe;font-size:11px;font-weight:500}.stop{background:#dc2626}.conn{display:grid;grid-template-columns:1fr;gap:5px;color:#cbd5df;font-size:14px}.conn code{color:#7ee3c4;font-weight:900}.foot{text-align:center;color:#758693;font-size:12px;margin-top:10px}"
"</style></head><body><div class='wrap'>"
"<div class='top'><div><div class='brand'>HiSpark 智能小车</div><div class='sub'>Wi-Fi IoT Demo / 黑线循迹 / 超声波避障</div></div><span class='pill' id='wifi'>连接中</span></div>"
"<div class='hero'><b>当前模式</b><div class='mode' id='mode'>--</div></div>"
"<div class='metrics'><div class='metric'><b>前方距离</b><span class='v' id='dist'>--</span></div><div class='metric'><b>连接延迟</b><span class='v' id='lat'>--</span></div><div class='metric'><b>速度 m/s</b><span class='v' id='mps'>--</span></div><div class='metric'><b>保护状态</b><span class='v' id='safe'>--</span></div></div>"
"<div class='card'><div class='section'>方向距离 cm</div><div class='dirs'><div>左60<span id='rd0'>--</span></div><div>左30<span id='rd1'>--</span></div><div>前方<span id='rd2'>--</span></div><div>右30<span id='rd3'>--</span></div><div>右60<span id='rd4'>--</span></div></div></div>"
"<div class='modebar'><button onclick=\"setMode('stop')\">停止</button><button onclick=\"setMode('trace')\">循迹</button><button onclick=\"setMode('obstacle')\">避障</button></div>"
"<div class='drive'><span class='blank'></span><button data-m='forward'>↑<small>前进</small></button><span class='blank'></span><button data-m='left'>←<small>左转</small></button><button class='stop' data-m='stop'>■<small>停止</small></button><button data-m='right'>→<small>右转</small></button><span class='blank'></span><button data-m='backward'>↓<small>后退</small></button><span class='blank'></span></div>"
"<div class='card conn'><div>SSID: <code>HMZXYYDS</code></div><div>Password: <code>HMZXYYDS</code></div><div>Website: <code>http://" WIFI_AP_IP "</code></div></div>"
"<div class='foot'>Demo Version / Hi3861 Wi-Fi IoT Smart Car</div></div><script>"
"var E=function(id){return document.getElementById(id)},activeCmd='stop';"
"function req(path,cb){var x=new XMLHttpRequest(),t=Date.now();x.timeout=900;x.onreadystatechange=function(){if(x.readyState===4){if(x.status===200){if(cb)cb(x.responseText,Date.now()-t)}else E('wifi').textContent='连接断开'}};x.ontimeout=x.onerror=function(){E('wifi').textContent='连接断开'};x.open('GET',path+(path.indexOf('?')>=0?'&':'?')+'t='+Date.now(),true);x.send()}"
"function go(m){req('/cmd?move='+m)}function setDrive(m){activeCmd=m;go(m)}function setMode(m){req('/mode?set='+m,function(){stat()})}"
"document.querySelectorAll('[data-m]').forEach(function(b){var m=b.getAttribute('data-m'),down=function(e){e.preventDefault();setDrive(m)},up=function(e){e.preventDefault();setDrive('stop')};b.addEventListener('touchstart',down);b.addEventListener('touchend',up);b.addEventListener('touchcancel',up);b.addEventListener('mousedown',down);b.addEventListener('mouseup',up);b.addEventListener('mouseleave',up);b.onclick=function(e){e.preventDefault()}});"
"function dt(v){return v>0?v+'cm':'--'}function dirs(rs){for(var i=0;i<5;i++)E('rd'+i).textContent=dt(rs&&rs[i]?rs[i].d:-1)}"
"function stat(){req('/api/status',function(s,ms){var d=JSON.parse(s);E('lat').textContent=ms+'ms';E('wifi').textContent=d.wifi?'热点已开 '+d.clients+'人':'热点未开';E('mode').textContent=d.mode;E('dist').textContent=dt(d.distance);E('mps').textContent=d.mps;E('safe').textContent=d.safe;dirs(d.radar||[])})}"
"setInterval(stat,800);setInterval(function(){if(activeCmd!=='stop')go(activeCmd)},350);stat();</script></body></html>";

static void WifiStateChanged(int state)
{
    g_wifiReady = (state == WIFI_HOTSPOT_ACTIVE) ? 1 : 0;
}

static void WifiStaJoin(StationInfo *info)
{
    (void)info;
    if (g_wifiClients < WIFI_MAX_STA_NUM) {
        g_wifiClients++;
    }
}

static void WifiStaLeave(StationInfo *info)
{
    (void)info;
    if (g_wifiClients > 0) {
        g_wifiClients--;
    }
}

static WifiEvent g_wifiEvent = {
    .OnHotspotStaJoin = WifiStaJoin,
    .OnHotspotStaLeave = WifiStaLeave,
    .OnHotspotStateChanged = WifiStateChanged,
};

static int StartWifiAp(void)
{
    HotspotConfig config = {0};
    WifiErrorCode err;

    strncpy(config.ssid, WIFI_AP_SSID, sizeof(config.ssid) - 1);
    strncpy(config.preSharedKey, WIFI_AP_PASSWORD, sizeof(config.preSharedKey) - 1);
    config.securityType = WIFI_SEC_TYPE_PSK;
    config.band = HOTSPOT_BAND_TYPE_2G;
    config.channelNum = HOTSPOT_DEFAULT_CHANNEL;

    RegisterWifiEvent(&g_wifiEvent);
    err = SetHotspotConfig(&config);
    if (err != WIFI_SUCCESS) {
        return -1;
    }

    g_wifiReady = 0;
    err = EnableHotspot();
    if (err != WIFI_SUCCESS) {
        return -1;
    }

    /* EnableHotspot configures ap0 as 192.168.5.1 and starts DHCP. */
    g_wifiReady = 1;
    g_displayDirty = 1;
    return 0;
}

static void SendAll(int fd, const char *data)
{
    size_t left = strlen(data);
    const char *p = data;
    while (left > 0) {
        ssize_t sent = send(fd, p, left, 0);
        if (sent <= 0) {
            return;
        }
        p += sent;
        left -= (size_t)sent;
    }
}

static void SendHttp(int fd, const char *type, const char *body)
{
    char header[128];
    snprintf(header, sizeof(header),
        "HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
        type, (unsigned int)strlen(body));
    SendAll(fd, header);
    SendAll(fd, body);
}

static void SendStatusJson(int fd)
{
    char body[1500];
    unsigned int mps100 = g_speedMps100;
    unsigned int kmh100 = mps100 * 36 / 10;
    const char *safe = "OK";
    IotGpioValue traceLeft = IOT_GPIO_VALUE1;
    IotGpioValue traceRight = IOT_GPIO_VALUE1;
    int traceSeeLeft = 0;
    int traceSeeRight = 0;
    if (g_linkLostProtect != 0) {
        safe = "LINK";
    } else if (g_collisionProtect != 0) {
        safe = "COLL";
    }

    GetTraceView(&traceLeft, &traceRight, &traceSeeLeft, &traceSeeRight);

    g_httpRequests++;
    snprintf(body, sizeof(body),
        "{\"mode\":\"%s\",\"distance\":%d,\"collision\":%u,\"mps\":\"%u.%02u\","
        "\"kmh\":\"%u.%02u\",\"battery\":%u,\"wifi\":%u,\"nfc\":%u,\"clients\":%u,\"requests\":%u,"
        "\"obstacle\":%u,\"timeout\":%u,\"safe\":\"%s\",\"bat\":\"BAT3-9V\",\"radarOn\":%u,"
        "\"radar\":[{\"a\":%d,\"d\":%d},{\"a\":%d,\"d\":%d},{\"a\":%d,\"d\":%d},"
        "{\"a\":%d,\"d\":%d},{\"a\":%d,\"d\":%d}],"
        "\"trace\":{\"left\":%u,\"right\":%u,\"seeLeft\":%d,\"seeRight\":%d,"
        "\"leftBlack\":%u,\"rightBlack\":%u,\"whiteValid\":%u,\"blackValid\":%u,"
        "\"whiteLeft\":%u,\"whiteRight\":%u,\"blackLeft\":%u,\"blackRight\":%u,"
        "\"action\":\"%s\"}}",
        ModeName(g_carMode), g_lastDistanceCm, g_collisionDistanceCm,
        mps100 / 100, mps100 % 100, kmh100 / 100, kmh100 % 100,
        g_batteryPercent, g_wifiReady, g_nfcReady, g_wifiClients, g_httpRequests,
        g_obstacleDistanceCm, g_remoteTimeoutMs, safe, g_radarEnabled,
        RADAR_ANGLE_DEG[0], g_radarDistanceCm[0],
        RADAR_ANGLE_DEG[1], g_radarDistanceCm[1],
        RADAR_ANGLE_DEG[2], g_radarDistanceCm[2],
        RADAR_ANGLE_DEG[3], g_radarDistanceCm[3],
        RADAR_ANGLE_DEG[4], g_radarDistanceCm[4],
        traceLeft, traceRight, traceSeeLeft, traceSeeRight,
        g_traceLeftBlackLevel, g_traceRightBlackLevel,
        g_traceWhiteValid, g_traceBlackValid,
        g_traceWhiteLeft, g_traceWhiteRight, g_traceBlackLeft, g_traceBlackRight,
        TraceActionName(traceSeeLeft, traceSeeRight));
    SendHttp(fd, "application/json", body);
}

static int GetQueryNumber(const char *request, const char *name, unsigned int *value)
{
    const char *p = strstr(request, name);
    if (p == NULL) {
        return 0;
    }

    p += strlen(name);
    if (*p < '0' || *p > '9') {
        return 0;
    }
    *value = (unsigned int)atoi(p);
    return 1;
}

static void HandleMoveRequest(const char *request)
{
    if (strstr(request, "move=forward") != NULL) {
        SetRemoteAction(REMOTE_FORWARD);
    } else if (strstr(request, "move=backward") != NULL) {
        SetRemoteAction(REMOTE_BACKWARD);
    } else if (strstr(request, "move=left") != NULL) {
        SetRemoteAction(REMOTE_LEFT);
    } else if (strstr(request, "move=right") != NULL) {
        SetRemoteAction(REMOTE_RIGHT);
    } else {
        SetRemoteAction(REMOTE_STOP);
    }
}

static void HandleParamRequest(const char *request)
{
    unsigned int value = 0;

    if (strstr(request, "reset=1") != NULL) {
        ResetRuntimeParams();
        return;
    }

    if (GetQueryNumber(request, "obstacle=", &value)) {
        g_obstacleDistanceCm = ClampUnsigned(value, MIN_OBSTACLE_DISTANCE_CM, MAX_OBSTACLE_DISTANCE_CM);
    }
    if (GetQueryNumber(request, "collision=", &value)) {
        g_collisionDistanceCm = ClampUnsigned(value, MIN_COLLISION_DISTANCE_CM, MAX_COLLISION_DISTANCE_CM);
    }
    if (GetQueryNumber(request, "timeout=", &value)) {
        g_remoteTimeoutMs = ClampUnsigned(value, MIN_REMOTE_TIMEOUT_MS, MAX_REMOTE_TIMEOUT_MS);
    }
    if (g_obstacleDistanceCm <= g_collisionDistanceCm) {
        g_obstacleDistanceCm = ClampUnsigned(g_collisionDistanceCm + 5,
            MIN_OBSTACLE_DISTANCE_CM, MAX_OBSTACLE_DISTANCE_CM);
    }
    g_displayDirty = 1;
}

static void HandleRadarRequest(const char *request)
{
    if (strstr(request, "on=1") != NULL) {
        g_radarEnabled = 1;
        g_radarIndex = 0;
        g_lastRadarMs = 0;
    } else {
        g_radarEnabled = 0;
        ServoCenter();
    }
    g_displayDirty = 1;
}

static void HandleTraceRequest(const char *request)
{
    IotGpioValue left = IOT_GPIO_VALUE1;
    IotGpioValue right = IOT_GPIO_VALUE1;

    ReadTraceSensors(&left, &right);
    if (strstr(request, "white=1") != NULL) {
        g_traceWhiteLeft = left;
        g_traceWhiteRight = right;
        g_traceWhiteValid = 1;
    }
    if (strstr(request, "black=1") != NULL) {
        g_traceBlackLeft = left;
        g_traceBlackRight = right;
        g_traceBlackValid = 1;
    }
    if (strstr(request, "apply=1") != NULL) {
        if (g_traceBlackValid != 0) {
            g_traceLeftBlackLevel = g_traceBlackLeft;
            g_traceRightBlackLevel = g_traceBlackRight;
        }
    }
    g_displayDirty = 1;
}

static void HandleModeRequest(const char *request)
{
    if (strstr(request, "set=trace") != NULL) {
        SetCarMode(MODE_TRACE);
    } else if (strstr(request, "set=obstacle") != NULL) {
        SetCarMode(MODE_OBSTACLE);
    } else if (strstr(request, "set=remote") != NULL) {
        g_carMode = MODE_REMOTE;
        g_lastRemoteMs = hi_get_milli_seconds();
        g_linkLostProtect = 0;
        g_displayDirty = 1;
    } else {
        SetCarMode(MODE_STOP);
    }
}

static void HandleHttpRequest(int fd, const char *request)
{
    if (strncmp(request, "GET /api/status", 15) == 0) {
        SendStatusJson(fd);
    } else if (strncmp(request, "GET /cmd?", 9) == 0) {
        HandleMoveRequest(request);
        SendStatusJson(fd);
    } else if (strncmp(request, "GET /mode?", 10) == 0) {
        HandleModeRequest(request);
        SendStatusJson(fd);
    } else if (strncmp(request, "GET /param?", 11) == 0) {
        HandleParamRequest(request);
        SendStatusJson(fd);
    } else if (strncmp(request, "GET /radar?", 11) == 0) {
        HandleRadarRequest(request);
        SendStatusJson(fd);
    } else if (strncmp(request, "GET /trace?", 11) == 0) {
        HandleTraceRequest(request);
        SendStatusJson(fd);
    } else if (strncmp(request, "GET / ", 6) == 0 || strncmp(request, "GET /?", 6) == 0) {
        SendHttp(fd, "text/html; charset=utf-8", CONTROL_PAGE_DEMO);
    } else {
        SendHttp(fd, "text/plain", "HMZXYYDS");
    }
}

static void SetSocketTimeout(int fd, long sec)
{
    struct timeval timeout;
    timeout.tv_sec = sec;
    timeout.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
}

static void HttpServerTask(void *arg)
{
    (void)arg;
    int serverFd;
    struct sockaddr_in serverAddr = {0};

    if (StartWifiAp() != 0) {
        g_wifiReady = 0;
        g_displayDirty = 1;
        return;
    }

    serverFd = socket(AF_INET, SOCK_STREAM, 0);
    if (serverFd < 0) {
        return;
    }

    int reuse = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(HTTP_PORT);
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(serverFd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        lwip_close(serverFd);
        return;
    }
    listen(serverFd, 4);

    while (1) {
        char request[512] = {0};
        struct sockaddr_in clientAddr = {0};
        socklen_t clientLen = sizeof(clientAddr);
        int clientFd = accept(serverFd, (struct sockaddr *)&clientAddr, &clientLen);
        if (clientFd < 0) {
            osDelay(10);
            continue;
        }
        SetSocketTimeout(clientFd, 1);
        if (recv(clientFd, request, sizeof(request) - 1, 0) > 0) {
            HandleHttpRequest(clientFd, request);
        }
        lwip_close(clientFd);
        IoTWatchDogDisable();
    }
}

static void ProbeNfcModule(void)
{
    unsigned char cmd[2] = {0xFF, 0xFD};
    unsigned char data = 0;
    if (IoTI2cWrite(OLED_I2C_IDX, 0xAE, cmd, sizeof(cmd)) == 0 &&
        IoTI2cRead(OLED_I2C_IDX, 0xAF, &data, 1) == 0) {
        g_nfcReady = 1;
    } else {
        g_nfcReady = 0;
    }
    g_displayDirty = 1;
}

static void HardwareInit(void)
{
    hi_io_set_func(HI_IO_NAME_GPIO_13, HI_IO_FUNC_GPIO_13_I2C0_SDA);
    hi_io_set_func(HI_IO_NAME_GPIO_14, HI_IO_FUNC_GPIO_14_I2C0_SCL);
    hi_io_set_pull(HI_IO_NAME_GPIO_13, HI_IO_PULL_UP);
    hi_io_set_pull(HI_IO_NAME_GPIO_14, HI_IO_PULL_UP);
    IoTI2cInit(OLED_I2C_IDX, OLED_BAUDRATE);
    osDelay(20);
    OledInit();

    CarGpioInit();
    ButtonInit();
    ProbeNfcModule();
    IoTWatchDogDisable();
    ServoCenter();
}

static void SmartCarTask(void *arg)
{
    (void)arg;
    unsigned int displayTicks = 0;
    unsigned int distanceTicks = 0;

    HardwareInit();
    OledBootAnimation();
    g_carMode = MODE_STOP;
    g_displayView = VIEW_CONTROL;
    CarStop();
    RefreshDisplay();

    while (1) {
        ProcessPendingKey();
        RadarScanStep();

        if (g_radarEnabled == 0 && g_carMode != MODE_OBSTACLE && distanceTicks >= 100) {
            UpdateDistance();
            distanceTicks = 0;
        }

        if (g_carMode == MODE_STOP) {
            CarStop();
        } else if (g_carMode == MODE_REMOTE) {
            RemoteStep();
        } else if (g_carMode == MODE_TRACE) {
            TraceStep();
        } else {
            ObstacleStep();
        }

        if (g_displayDirty != 0 || displayTicks >= 100) {
            RefreshDisplay();
            displayTicks = 0;
        }

        IoTWatchDogDisable();
        osDelay(2);
        displayTicks++;
        distanceTicks++;
    }
}

static void SmartCarEntry(void)
{
    osThreadAttr_t attr;

    attr.name = "SmartCarTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 8192;
    attr.priority = osPriorityNormal;

    if (osThreadNew(SmartCarTask, NULL, &attr) == NULL) {
        printf("Failed to create SmartCarTask\r\n");
    }

    attr.name = "HttpServerTask";
    attr.stack_size = 12288;
    attr.priority = osPriorityBelowNormal;
    if (osThreadNew(HttpServerTask, NULL, &attr) == NULL) {
        printf("Failed to create HttpServerTask\r\n");
    }
}

APP_FEATURE_INIT(SmartCarEntry);
