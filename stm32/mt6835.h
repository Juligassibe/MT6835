#ifndef MT6835_H
#define MT6835_H

#include <stdint.h>
#include "spi.h"

typedef enum {
	MT6835_READ        = 0b0011,
	MT6835_WRITE       = 0b0110,
	MT6835_PROG_EEPROM = 0b1100,
	MT6835_SET_ZERO    = 0b0101,
	MT6835_BURST_READ  = 0b1010
} MT6835_CMD_e;

typedef enum {
    MT6835_USER_ID      = 0x001,
	MT6835_ANGLE_HIGH   = 0x003,
	MT6835_ANGLE_MID    = 0x004,
	MT6835_ANGLE_LOW    = 0x005,   // 0:2 STATUS - 3:7 ANGLE1
	MT6835_CRC          = 0x006,
	MT6835_ABZ_RES_HIGH = 0x007,
	MT6835_ABZ_RES_LOW  = 0x008,   // 0 Swap AB - 1 ABZ Off - 2:7 ABZ_RES1
	MT6835_ZERO_HIGH    = 0x009,
	MT6835_ZERO_LOW     = 0x00A,   // 0:2 Z pulse width - 3 Z edge - 4:7 ZERO1
	MT6835_Z_PHASE      = 0x00B,   // 0:3 UVW_RES - 4 UVW_OFF - 5 UVW_MUX - 6:7 Z_PHASE
	MT6835_PWM_CONF     = 0x00C,   // 0:2 PWM_SEL - 3 PWM_POL - 4 PWM_FQ - 5 NLC_EN
	MT6835_ROT_DIR      = 0x00D,   // 0:2 HYST - 3 ROT_DIR
	MT6835_AUTOCAL_FREQ = 0x00E,   // 4:6 AUTOCAL_FREQ - 7 GPIO_DS
	MT6835_BW           = 0x011,   // 0:2 BW
	MT6835_NLC_START    = 0x013,
	MT6835_NLC_END      = 0x0D2
} MT6835_ADDR_e;

typedef enum {
	MT6835_OK,
	MT6835_INV_RES,
	MT6835_ZERO_FAIL,
	MT6835_INV_WIDTH,
	MT6835_INV_PHASE,
	MT6835_SPI_ERROR,
	MT6835_CHECK_ERROR
} MT6835_RETURNS_e;

typedef enum {
	MT6835_ABZ_ON = 0,
	MT6835_ABZ_OFF
} MT6835_ABZ_OFF_e;

typedef enum {
	MT6835_ABZ_NO_SWAP = 0,
	MT6835_ABZ_SWAP
} MT6835_ABZ_SWAP_e;

typedef enum {
	MT6835_ZRE = 0,	// Flanco de subida de Z alineado a 0
	MT6835_ZFE		// Flanco de bajada de Z alineado a 0
} MT6835_Z_EDGE_e;

typedef enum {
	MT6835_Z_WIDTH_1LSB = 0,
	MT6835_Z_WIDTH_2LSB,
	MT6835_Z_WIDTH_4LSB,
	MT6835_Z_WIDTH_8LSB,
	MT6835_Z_WIDTH_16LSB,
	MT6835_Z_WIDTH_60DEG,
	MT6835_Z_WIDTH_120DEG,
	MT6835_Z_WIDTH_180DEG
} MT6835_Z_WIDTH_e;

typedef enum {
	MT6835_Z_AFE = 0,
	MT6835_Z_BRE,
	MT6835_Z_ARE,
	MT6835_Z_BFE
} MT6835_Z_PHASE_e;

typedef enum {
    MT6835_CCW_BA = 0,
	MT6835_CCW_AB = 1
} MT6835_ROT_DIR_e;

typedef enum {
	MT6835_NOT_INIT = 0,
	MT6835_INIT
} MT6835_STATE_e;

// Para acceder en tiempo de ejecucion a la config del encoder
typedef struct {
	uint16_t abz_res;
	uint8_t abz_off;
	uint8_t abz_swap;
	uint8_t z_edge;
	uint8_t z_width;
	uint8_t z_phase;
	uint8_t rot_dir;
	uint8_t init;
} mt6835_settings_t;

typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *CS_port;
	uint16_t CS_pin;
	mt6835_settings_t settings;
} mt6835_t;

HAL_StatusTypeDef mt6835_spi_txrx(mt6835_t *hmt6835, uint8_t *tx, uint8_t *rx, uint8_t len);		/* LISTO */
HAL_StatusTypeDef mt6835_spi_tx(mt6835_t *hmt6835, uint8_t *tx, uint8_t len);						/* LISTO */
HAL_StatusTypeDef mt6835_spi_txrx_dma(mt6835_t *hmt6835, uint8_t *tx, uint8_t *rx, uint8_t len);	/* LISTO */
MT6835_RETURNS_e mt6835_get_position(mt6835_t *hmt6835, uint32_t *angulo, uint8_t *crc);							/* LISTO */
MT6835_RETURNS_e mt6835_get_abz_res(mt6835_t *hmt6835, uint16_t *abz_res);							/* LISTO */
MT6835_RETURNS_e mt6835_set_abz_res(mt6835_t *hmt6835, uint16_t abz_res);							/* LISTO */
MT6835_RETURNS_e mt6835_get_abz_off(mt6835_t *hmt6835, uint8_t *abz_off);							/* LISTO */
MT6835_RETURNS_e mt6835_set_abz_off(mt6835_t *hmt6835, uint8_t abz_off);							/* LISTO */
MT6835_RETURNS_e mt6835_get_abz_swap(mt6835_t *hmt6835, uint8_t *abz_swap);							/* LISTO */
MT6835_RETURNS_e mt6835_set_abz_swap(mt6835_t *hmt6835, uint8_t abz_swap);							/* LISTO */
MT6835_RETURNS_e mt6835_set_zero(mt6835_t *hmt6835);												/* LISTO */
MT6835_RETURNS_e mt6835_get_z_edge(mt6835_t *hmt6835, uint8_t *z_edge);								/* LISTO */
MT6835_RETURNS_e mt6835_set_z_edge(mt6835_t *hmt6835, uint8_t z_edge);								/* LISTO */
MT6835_RETURNS_e mt6835_get_z_width(mt6835_t *hmt6835, uint8_t *z_width);							/* LISTO */
MT6835_RETURNS_e mt6835_set_z_width(mt6835_t *hmt6835, uint8_t z_width);							/* LISTO */
MT6835_RETURNS_e mt6835_get_z_phase(mt6835_t *hmt6835, uint8_t *z_phase);							/* LISTO */
MT6835_RETURNS_e mt6835_set_z_phase(mt6835_t *hmt6835, uint8_t z_phase);							/* LISTO */
MT6835_RETURNS_e mt6835_get_rot_dir(mt6835_t *hmt6835, uint8_t *rot_dir);							/* LISTO */
MT6835_RETURNS_e mt6835_set_rot_dir(mt6835_t *hmt6835, uint8_t rot_dir);							/* LISTO */
MT6835_RETURNS_e mt6835_init(mt6835_t *hmt6835, mt6835_settings_t *settings);						/* LISTO */

#endif
