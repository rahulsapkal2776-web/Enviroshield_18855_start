#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

// =======================
// PIC16F18855 - BASIC STARTER CODE
// XC8 / MPLAB X
// =======================

// NOTE:
// 1) Add/adjust configuration bits from MPLAB Code Configurator (MCC)
//    based on your hardware clock and watchdog requirements.
// 2) This starter focuses on pin direction + basic manual Modbus coil mapping.

#define _XTAL_FREQ 16000000UL

// -----------------------
// Pin mapping (as provided)
// -----------------------
//  1 MCLR/VPP  : Reset
//  2 RA0       : Temp Sensor 1 (analog)
//  3 RA1       : Temp Sensor 2 (analog)
//  4 RA2       : Humidity Sensor Analog (analog)
//  5 RA3       : Spare
//  6 RA4       : Phase Preventer Input
//  7 RA5       : Spare
//  8 VSS       : GND
//  9 OSC1/CLKIN
// 10 OSC2/CLKOUT
// 11 RC0       : Main Motor Forward
// 12 RC1       : Main Motor Reverse
// 13 RC2       : Blower
// 14 RC3       : Heater
// 15 RC4       : Crusher Forward
// 16 RC5       : Crusher Reverse
// 17 RC6       : RS485 TX
// 18 RC7       : RS485 RX
// 19 VSS       : GND
// 20 VDD       : +5V
// 21 RB0       : Door 1
// 22 RB1       : Door 2
// 23 RB2       : Humidity Digital Threshold
// 24 RB3       : Emergency Stop
// 25 RB4       : Extra Relay Output 1
// 26 RB5       : Not used for RS485 DE/RE (auto RX/TX module)
// 27 RB6       : Spare
// 28 RB7       : Spare

// -----------------------
// HMI/Modbus coil mapping (0x)
// -----------------------
#define COIL_MOTOR_FWD_START   0u
#define COIL_MOTOR_FWD_STOP    1u
#define COIL_MOTOR_REV_START   2u
#define COIL_MOTOR_REV_STOP    3u
#define COIL_BLOWER_START      4u
#define COIL_BLOWER_STOP       5u
#define COIL_HEATER_START      6u
#define COIL_HEATER_STOP       7u
#define COIL_MANUAL_MODE       10u
#define COIL_AUTO_MODE         11u

// -----------------------
// Output controls
// -----------------------
#define MAIN_MOTOR_FWD_LAT     LATCbits.LATC0
#define MAIN_MOTOR_REV_LAT     LATCbits.LATC1
#define BLOWER_LAT             LATCbits.LATC2
#define HEATER_LAT             LATCbits.LATC3
#define CRUSHER_FWD_LAT        LATCbits.LATC4
#define CRUSHER_REV_LAT        LATCbits.LATC5
#define EXTRA_RELAY_LAT        LATBbits.LATB4

// -----------------------
// Input reads
// -----------------------
#define DOOR1_PORT             PORTBbits.RB0
#define DOOR2_PORT             PORTBbits.RB1
#define HUM_DIGITAL_PORT       PORTBbits.RB2
#define ESTOP_PORT             PORTBbits.RB3
#define PHASE_PREVENT_PORT     PORTAbits.RA4

typedef struct
{
    bool manual_mode;
    bool auto_mode;
} system_mode_t;

static volatile system_mode_t g_mode = { .manual_mode = true, .auto_mode = false };

static void gpio_init(void)
{
    // --- Analog configuration ---
    // RA0/RA1/RA2 are analog sensors.
    ANSELAbits.ANSA0 = 1;
    ANSELAbits.ANSA1 = 1;
    ANSELAbits.ANSA2 = 1;

    // Other A pins digital.
    ANSELAbits.ANSA3 = 0;
    ANSELAbits.ANSA4 = 0;
    ANSELAbits.ANSA5 = 0;

    // Port B/C as digital for now.
    ANSELB = 0x00;
    ANSELC = 0x00;

    // --- TRIS (1=input, 0=output) ---
    TRISA = 0x3F; // RA0..RA5 inputs (sensors/spares)

    TRISBbits.TRISB0 = 1; // Door1
    TRISBbits.TRISB1 = 1; // Door2
    TRISBbits.TRISB2 = 1; // Humidity digital
    TRISBbits.TRISB3 = 1; // E-Stop
    TRISBbits.TRISB4 = 0; // Extra relay output
    TRISBbits.TRISB5 = 1; // Unused for DE/RE (keep input)
    TRISBbits.TRISB6 = 1; // Spare
    TRISBbits.TRISB7 = 1; // Spare

    TRISCbits.TRISC0 = 0; // Motor FWD
    TRISCbits.TRISC1 = 0; // Motor REV
    TRISCbits.TRISC2 = 0; // Blower
    TRISCbits.TRISC3 = 0; // Heater
    TRISCbits.TRISC4 = 0; // Crusher FWD
    TRISCbits.TRISC5 = 0; // Crusher REV
    TRISCbits.TRISC6 = 0; // UART TX (RS485)
    TRISCbits.TRISC7 = 1; // UART RX (RS485)

    // --- Safe startup states ---
    LATC = 0x00;
    LATBbits.LATB4 = 0;
}

static void outputs_all_stop(void)
{
    MAIN_MOTOR_FWD_LAT = 0;
    MAIN_MOTOR_REV_LAT = 0;
    BLOWER_LAT = 0;
    HEATER_LAT = 0;
    CRUSHER_FWD_LAT = 0;
    CRUSHER_REV_LAT = 0;
}

static void set_manual_mode(bool enable)
{
    g_mode.manual_mode = enable;
    g_mode.auto_mode = !enable;

    // Optional safety behavior on mode change
    outputs_all_stop();
}

// Call this from your Modbus stack whenever a coil write is received.
void modbus_handle_coil_write(uint16_t coil_address, bool value)
{
    (void)value; // start/stop coils are edge-command style in this starter.

    switch (coil_address)
    {
        case COIL_MANUAL_MODE:
            set_manual_mode(true);
            break;

        case COIL_AUTO_MODE:
            set_manual_mode(false);
            break;

        // Manual mode controls (from HMI buttons)
        case COIL_MOTOR_FWD_START:
            if (g_mode.manual_mode)
            {
                MAIN_MOTOR_REV_LAT = 0; // interlock
                MAIN_MOTOR_FWD_LAT = 1;
            }
            break;

        case COIL_MOTOR_FWD_STOP:
            MAIN_MOTOR_FWD_LAT = 0;
            break;

        case COIL_MOTOR_REV_START:
            if (g_mode.manual_mode)
            {
                MAIN_MOTOR_FWD_LAT = 0; // interlock
                MAIN_MOTOR_REV_LAT = 1;
            }
            break;

        case COIL_MOTOR_REV_STOP:
            MAIN_MOTOR_REV_LAT = 0;
            break;

        case COIL_BLOWER_START:
            if (g_mode.manual_mode)
            {
                BLOWER_LAT = 1;
            }
            break;

        case COIL_BLOWER_STOP:
            BLOWER_LAT = 0;
            break;

        case COIL_HEATER_START:
            if (g_mode.manual_mode)
            {
                HEATER_LAT = 1;
            }
            break;

        case COIL_HEATER_STOP:
            HEATER_LAT = 0;
            break;

        default:
            // Unknown/unused coil.
            break;
    }
}

static void app_init(void)
{
    gpio_init();

    // TODO: UART init for RS485 Modbus RTU (RC6 TX, RC7 RX)
    // TODO: Modbus slave init (set node ID, baud rate, parity)
    // NOTE: RB5 DE/RE control is intentionally NOT used because your RS485
    // module handles RX/TX auto direction internally.

    outputs_all_stop();
}

void main(void)
{
    app_init();

    while (1)
    {
        // TODO: Poll Modbus stack here.
        // Example:
        // modbus_poll();

        // TODO: Add auto mode logic step-by-step in next iterations.
    }
}
