#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

// =====================================================
// PIC16F18855 + Flexem HMI + RS485 (Auto RX/TX) Starter
// - Internal oscillator: 32 MHz
// - Modbus RTU slave basic communication check
// - HMI coil control for Motor/Blower/Heater relay outputs
// =====================================================

#define _XTAL_FREQ 32000000UL

// -----------------------
// User settings
// -----------------------
#define MODBUS_SLAVE_ID                1u
#define MODBUS_MAX_FRAME               64u
#define MODBUS_COMM_TIMEOUT_LOOPS      50000UL

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
#define COIL_COUNT             16u

// -----------------------
// Output controls
// -----------------------
#define MAIN_MOTOR_FWD_LAT     LATCbits.LATC0
#define MAIN_MOTOR_REV_LAT     LATCbits.LATC1
#define BLOWER_LAT             LATCbits.LATC2
#define HEATER_LAT             LATCbits.LATC3
#define CRUSHER_FWD_LAT        LATCbits.LATC4
#define CRUSHER_REV_LAT        LATCbits.LATC5
#define COMM_OK_RELAY_LAT      LATBbits.LATB4

// -----------------------
// Inputs
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

static volatile system_mode_t g_mode = { true, false };
static volatile bool g_coils[COIL_COUNT];
static volatile uint32_t g_comm_watchdog = 0;

// =====================================================
// Low-level init
// =====================================================
static void clock_init_32mhz_internal(void)
{
    // HFINTOSC selected, divider 1:1
    OSCCON1bits.NDIV = 0b0000;
    OSCCON1bits.NOSC = 0b110;

    // HFINTOSC frequency = 32 MHz
    OSCFRQbits.HFFRQ = 0b101;
}

static void gpio_init(void)
{
    // RA0/RA1/RA2 analog sensors
    ANSELAbits.ANSA0 = 1;
    ANSELAbits.ANSA1 = 1;
    ANSELAbits.ANSA2 = 1;

    // RA3/RA4/RA5 digital
    ANSELAbits.ANSA3 = 0;
    ANSELAbits.ANSA4 = 0;
    ANSELAbits.ANSA5 = 0;

    ANSELB = 0x00;
    ANSELC = 0x00;

    // Port directions
    TRISA = 0x3F; // RA0..RA5 input

    TRISBbits.TRISB0 = 1; // Door 1
    TRISBbits.TRISB1 = 1; // Door 2
    TRISBbits.TRISB2 = 1; // Humidity digital threshold
    TRISBbits.TRISB3 = 1; // Emergency stop
    TRISBbits.TRISB4 = 0; // Extra relay / COMM OK indication
    TRISBbits.TRISB5 = 1; // NOT used for RS485 DE/RE (auto direction module)
    TRISBbits.TRISB6 = 1; // Spare
    TRISBbits.TRISB7 = 1; // Spare

    TRISCbits.TRISC0 = 0; // Main motor forward relay
    TRISCbits.TRISC1 = 0; // Main motor reverse relay
    TRISCbits.TRISC2 = 0; // Blower relay
    TRISCbits.TRISC3 = 0; // Heater relay
    TRISCbits.TRISC4 = 0; // Crusher forward relay
    TRISCbits.TRISC5 = 0; // Crusher reverse relay
    TRISCbits.TRISC6 = 0; // RS485 TX (EUSART TX)
    TRISCbits.TRISC7 = 1; // RS485 RX (EUSART RX)

    // Safe state
    LATC = 0x00;
    COMM_OK_RELAY_LAT = 0;
}

static void uart_init_9600_8n1(void)
{
    // PPS mapping for EUSART1
    RC6PPS = 0x14;   // RC6 -> TX1
    RX1PPS = 0x17;   // RC7 -> RX1

    BAUD1CONbits.BRG16 = 1;
    TX1STAbits.BRGH = 1;

    // Fosc=32MHz, baud=9600, BRGH=1, BRG16=1 => SP1BRG ~= 832
    SP1BRGH = 0x03;
    SP1BRGL = 0x40;

    RC1STAbits.SPEN = 1;
    RC1STAbits.CREN = 1;
    TX1STAbits.SYNC = 0;
    TX1STAbits.TXEN = 1;
}

static inline void uart_write_byte(uint8_t b)
{
    while (!PIR3bits.TX1IF)
    {
    }
    TX1REG = b;
}

static inline bool uart_read_byte_nonblocking(uint8_t *out)
{
    if (PIR3bits.RC1IF)
    {
        if (RC1STAbits.OERR)
        {
            RC1STAbits.CREN = 0;
            RC1STAbits.CREN = 1;
        }
        *out = RC1REG;
        return true;
    }
    return false;
}

// =====================================================
// Application control
// =====================================================
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
    outputs_all_stop();
}

static void execute_manual_commands(uint16_t coil_address)
{
    switch (coil_address)
    {
        case COIL_MOTOR_FWD_START:
            if (g_mode.manual_mode)
            {
                MAIN_MOTOR_REV_LAT = 0;
                MAIN_MOTOR_FWD_LAT = 1;
            }
            break;

        case COIL_MOTOR_FWD_STOP:
            MAIN_MOTOR_FWD_LAT = 0;
            break;

        case COIL_MOTOR_REV_START:
            if (g_mode.manual_mode)
            {
                MAIN_MOTOR_FWD_LAT = 0;
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

        case COIL_MANUAL_MODE:
            set_manual_mode(true);
            break;

        case COIL_AUTO_MODE:
            set_manual_mode(false);
            break;

        default:
            break;
    }
}

// =====================================================
// Modbus RTU minimal stack
// Supported FC:
//  - 0x01 Read Coils (for HMI communication check)
//  - 0x05 Write Single Coil (for HMI control buttons)
// =====================================================
static uint16_t modbus_crc16(const uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    uint8_t i;
    uint8_t j;

    for (i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static void modbus_send_response(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++)
    {
        uart_write_byte(data[i]);
    }
}

static void modbus_send_exception(uint8_t function, uint8_t ex_code)
{
    uint8_t resp[5];
    uint16_t crc;

    resp[0] = MODBUS_SLAVE_ID;
    resp[1] = (uint8_t)(function | 0x80u);
    resp[2] = ex_code;

    crc = modbus_crc16(resp, 3);
    resp[3] = (uint8_t)(crc & 0xFFu);
    resp[4] = (uint8_t)(crc >> 8);

    modbus_send_response(resp, 5);
}

static void modbus_process_fc01_read_coils(const uint8_t *req)
{
    uint16_t start = (uint16_t)req[2] << 8 | req[3];
    uint16_t qty = (uint16_t)req[4] << 8 | req[5];
    uint8_t resp[32];
    uint8_t byte_count;
    uint16_t crc;
    uint16_t i;

    if ((qty == 0u) || (qty > COIL_COUNT) || ((start + qty) > COIL_COUNT))
    {
        modbus_send_exception(0x01, 0x02); // illegal data address
        return;
    }

    byte_count = (uint8_t)((qty + 7u) / 8u);
    resp[0] = MODBUS_SLAVE_ID;
    resp[1] = 0x01;
    resp[2] = byte_count;

    for (i = 0; i < byte_count; i++)
    {
        resp[3 + i] = 0;
    }

    for (i = 0; i < qty; i++)
    {
        if (g_coils[start + i])
        {
            resp[3 + (i / 8u)] |= (uint8_t)(1u << (i % 8u));
        }
    }

    crc = modbus_crc16(resp, (uint8_t)(3u + byte_count));
    resp[3 + byte_count] = (uint8_t)(crc & 0xFFu);
    resp[4 + byte_count] = (uint8_t)(crc >> 8);

    modbus_send_response(resp, (uint8_t)(5u + byte_count));
}

static void modbus_process_fc05_write_single_coil(const uint8_t *req)
{
    uint16_t addr = (uint16_t)req[2] << 8 | req[3];
    uint16_t val = (uint16_t)req[4] << 8 | req[5];

    if (addr >= COIL_COUNT)
    {
        modbus_send_exception(0x05, 0x02);
        return;
    }

    if ((val != 0xFF00u) && (val != 0x0000u))
    {
        modbus_send_exception(0x05, 0x03); // illegal data value
        return;
    }

    g_coils[addr] = (val == 0xFF00u);

    // Execute action on coil write
    if (g_coils[addr])
    {
        execute_manual_commands(addr);

        // command coils are momentary; clear after action
        if (addr <= COIL_HEATER_STOP)
        {
            g_coils[addr] = false;
        }
    }

    // FC05 response is exact echo of request
    modbus_send_response(req, 8);
}

static void modbus_process_frame(uint8_t *frame, uint8_t len)
{
    uint16_t rx_crc;
    uint16_t calc_crc;

    if (len < 8u)
    {
        return;
    }

    if ((frame[0] != MODBUS_SLAVE_ID) && (frame[0] != 0u))
    {
        return;
    }

    rx_crc = (uint16_t)frame[len - 1] << 8 | frame[len - 2];
    calc_crc = modbus_crc16(frame, (uint8_t)(len - 2u));
    if (rx_crc != calc_crc)
    {
        return;
    }

    // Any valid frame = communication OK
    g_comm_watchdog = 0;
    COMM_OK_RELAY_LAT = 1;

    switch (frame[1])
    {
        case 0x01:
            modbus_process_fc01_read_coils(frame);
            break;

        case 0x05:
            modbus_process_fc05_write_single_coil(frame);
            break;

        default:
            if (frame[0] != 0u) // no response to broadcast
            {
                modbus_send_exception(frame[1], 0x01); // illegal function
            }
            break;
    }
}

static void modbus_poll(void)
{
    static uint8_t frame[MODBUS_MAX_FRAME];
    static uint8_t idx = 0;
    static uint16_t silent_gap = 0;
    uint8_t b;

    if (uart_read_byte_nonblocking(&b))
    {
        if (idx < MODBUS_MAX_FRAME)
        {
            frame[idx++] = b;
        }
        silent_gap = 0;
    }
    else
    {
        if (idx > 0u)
        {
            silent_gap++;
            if (silent_gap > 1000u) // crude inter-frame timeout
            {
                modbus_process_frame(frame, idx);
                idx = 0;
                silent_gap = 0;
            }
        }
    }
}

static void app_init(void)
{
    uint8_t i;

    clock_init_32mhz_internal();
    gpio_init();
    uart_init_9600_8n1();

    for (i = 0; i < COIL_COUNT; i++)
    {
        g_coils[i] = false;
    }

    set_manual_mode(true);
}

void main(void)
{
    app_init();

    while (1)
    {
        modbus_poll();

        // Communication watchdog
        g_comm_watchdog++;
        if (g_comm_watchdog > MODBUS_COMM_TIMEOUT_LOOPS)
        {
            COMM_OK_RELAY_LAT = 0; // communication lost with HMI
        }

        // Basic hard safety hooks (can be expanded)
        if (ESTOP_PORT == 0u)
        {
            outputs_all_stop();
        }
    }
}
