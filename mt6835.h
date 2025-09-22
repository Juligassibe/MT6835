#include "driver/spi_master.h"

typedef enum MT6835_CMD_t {
    READ        = 0b0011,
    WRITE       = 0b0110,
    PROG_EEPROM = 0b1100,
    SET_ZERO    = 0b0101,
    BURST_READ  = 0b1010
};

typedef enum MT6835_ADDR_t{
    USER_ID      = 0x001,
    ANGLE3       = 0x003,
    ANGLE2       = 0x004,
    ANGLE1       = 0x005,   // 0:2 STATUS - 3:7 ANGLE1
    CRC          = 0x006,
    ABZ_RES2     = 0x007,
    ABZ_RES1     = 0x008,   // 0 Swap AB - 1 ABZ Off - 2:7 ABZ_RES1
    ZERO2        = 0x009,
    ZERO1        = 0x00A,   // 0:2 Z pulse width - 3 Z edge - 4:7 ZERO1
    UVW_CONF     = 0x00B,   // 0:3 UVW_RES - 4 UVW_OFF - 5 UVW_MUX - 6:7 Z_PHASE
    PWM_CONF     = 0x00C,   // 0:2 PWM_SEL - 3 PWM_POL - 4 PWM_FQ - 5 NLC_EN
    HYST         = 0x00D,   // 0:2 HYST - 3 ROT_DIR
    AUTOCAL_FREQ = 0x00E,   // 4:6 AUTOCAL_FREQ - 7 GPIO_DS
    BW           = 0x0011,  // 0:2 BW
    NLC_START    = 0x013,
    NLC_END      = 0x0D2
};



