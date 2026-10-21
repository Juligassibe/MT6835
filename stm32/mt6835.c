#include "mt6835.h"

static HAL_StatusTypeDef mt6835_spi_txrx(mt6835_t *hmt6835, uint8_t *tx, uint8_t *rx, uint8_t len) {
	HAL_GPIO_WritePin(hmt6835->CS_port, hmt6835->CS_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hmt6835->hspi, tx, rx, len, 10);
	HAL_GPIO_WritePin(hmt6835->CS_port, hmt6835->CS_pin, GPIO_PIN_SET);

	return status;
}

static HAL_StatusTypeDef mt6835_spi_tx(mt6835_t *hmt6835, uint8_t *tx, uint8_t len) {
	HAL_GPIO_WritePin(hmt6835->CS_port, hmt6835->CS_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(hmt6835->hspi, tx, len, 10);
	HAL_GPIO_WritePin(hmt6835->CS_port, hmt6835->CS_pin, GPIO_PIN_SET);

	return status;
}

MT6835_RETURNS_e mt6835_get_position(mt6835_t *hmt6835, uint32_t *angulo, uint8_t *crc) {
	uint8_t tx_data[6] = {0};
	uint8_t rx_data[6] = {0};

	tx_data[0] = MT6835_BURST_READ << 4;
	tx_data[1] = MT6835_ANGLE_HIGH;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 6) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	*angulo = ((uint32_t)rx_data[2] << 16) |
			  ((uint32_t)rx_data[3] << 8)  |
			  ((uint32_t)rx_data[4]);

	*crc = rx_data[5];

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_get_abz_res(mt6835_t *hmt6835, uint16_t *abz_res) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};
	uint16_t temp_res = 0;

	// Lectura del registro HIGH
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ABZ_RES_HIGH;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	temp_res = (uint16_t)rx_data[2] << 8;

	// Lectura del registro LOW
	tx_data[1] = MT6835_ABZ_RES_LOW;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	temp_res |= rx_data[2];

	temp_res >>= 2;

	*abz_res = temp_res + 1;

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_set_abz_res(mt6835_t *hmt6835, uint16_t abz_res) {
	if (abz_res > 16384 || abz_res == 0) {
		return MT6835_INV_RES;
	}

	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};
	uint8_t abz_res_low = 0;
	uint16_t abz_res_check = UINT16_MAX;

	// Primero leo para no pisar bits de ABZ_OFF y ABZ_SWAP
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ABZ_RES_LOW;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	abz_res_low = rx_data[2] & 0b00000011;

	// Escritura del registro HIGH
	tx_data[0] = MT6835_WRITE << 4;
	tx_data[1] = MT6835_ABZ_RES_HIGH;
	tx_data[2] = ((abz_res - 1) >> 6) & 0x00FF;

	if (mt6835_spi_tx(hmt6835, tx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	// Escritura del registro LOW
	tx_data[1] = MT6835_ABZ_RES_LOW;
	tx_data[2] = abz_res_low | (((abz_res - 1) << 2) & 0x00FF);

	if (mt6835_spi_tx(hmt6835, tx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	// Leo configuracion luego de escribirla para verificar que este bien escrita
	if (mt6835_get_abz_res(hmt6835, &abz_res_check) != MT6835_OK) {
		return MT6835_SPI_ERROR;
	}

	if (abz_res_check != abz_res) {
		return MT6835_CHECK_ERROR;
	}

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_get_abz_off(mt6835_t *hmt6835, MT6835_ABZ_OFF_e *abz_off) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ABZ_RES_LOW;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	*abz_off = (rx_data[2] >> 1) & 0x01;

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_set_abz_off(mt6835_t *hmt6835, MT6835_ABZ_OFF_e abz_off) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};
	uint8_t abz_off_check = UINT8_MAX;

	// Primero leo para no pisar bits de ABZ_RES y ABZ_SWAP
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ABZ_RES_LOW;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	uint8_t temp = rx_data[2];

	// Si abz_off > 0 pongo el bit en 1
	temp = (abz_off > 0) ? (temp | 0x02) : (temp & 0xFD);

	// Ahora escribo registro completo con nuevo valor de ABZ_OFF
	tx_data[0] = MT6835_WRITE << 4;
	tx_data[2] = temp;

	if (mt6835_spi_tx(hmt6835, tx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	// Leo configuracion luego de escribirla para verificar que este bien escrita
	if (mt6835_get_abz_off(hmt6835, &abz_off_check) != MT6835_OK) {
		return MT6835_SPI_ERROR;
	}

	if (abz_off_check != abz_off) {
		return MT6835_CHECK_ERROR;
	}

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_get_abz_swap(mt6835_t *hmt6835, MT6835_ABZ_SWAP_e *abz_swap) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ABZ_RES_LOW;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	*abz_swap = rx_data[2] & 0x01;

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_set_abz_swap(mt6835_t *hmt6835, MT6835_ABZ_SWAP_e abz_swap) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};
	uint8_t abz_swap_check = UINT8_MAX;

	// Primero leo para no pisar bits de ABZ_RES y ABZ_OFF
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ABZ_RES_LOW;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	uint8_t temp = rx_data[2];

	// Si abz_swap > 0 pongo el bit en 1
	temp = (abz_swap > 0) ? (temp | 0x01) : (temp & 0xFE);

	// Ahora escribo registro completo con nuevo valor de ABZ_SWAP
	tx_data[0] = MT6835_WRITE << 4;
	tx_data[2] = temp;

	if (mt6835_spi_tx(hmt6835, tx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	// Leo configuracion luego de escribirla para verificar que este bien escrita
	if (mt6835_get_abz_swap(hmt6835, &abz_swap_check) != MT6835_OK) {
		return MT6835_SPI_ERROR;
	}

	if (abz_swap_check != abz_swap) {
		return MT6835_CHECK_ERROR;
	}

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_set_zero(mt6835_t *hmt6835) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = (MT6835_SET_ZERO << 4);

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	if (rx_data[2] != 0x55) {
		return MT6835_ZERO_FAIL;
	}

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_get_z_edge(mt6835_t *hmt6835, MT6835_Z_EDGE_e *z_edge) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ZERO_LOW;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	*z_edge = (rx_data[2] >> 3) & 0x01;

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_set_z_edge(mt6835_t *hmt6835, MT6835_Z_EDGE_e z_edge) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};
	uint8_t z_edge_check = UINT8_MAX;

	// Primero leo para no pisar zero_low y z_width
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ZERO_LOW;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	uint8_t temp = rx_data[2];

	// Pongo el bit en 1 cuando z_edge > 0
	temp = (z_edge > 0) ? (temp | 0x08) : (temp & 0xF7);

	tx_data[0] = MT6835_WRITE << 4;
	tx_data[2] = temp;

	if (mt6835_spi_tx(hmt6835, tx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	// Leo configuracion luego de escribirla para verificar que este bien escrita
	if (mt6835_get_z_edge(hmt6835, &z_edge_check) != MT6835_OK) {
		return MT6835_SPI_ERROR;
	}

	if (z_edge_check != z_edge) {
		return MT6835_CHECK_ERROR;
	}

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_get_z_width(mt6835_t *hmt6835, MT6835_Z_WIDTH_e *z_width) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ZERO_LOW;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	*z_width = rx_data[2] & 0x07;

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_set_z_width(mt6835_t *hmt6835, MT6835_Z_WIDTH_e z_width) {
	if (z_width > MT6835_Z_WIDTH_180DEG) {
		return MT6835_INV_WIDTH;
	}

	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};
	uint8_t z_width_check = UINT8_MAX;

	// Primero leo para no pisar zero_low y z_edge
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ZERO_LOW;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	// Limpio bits de Z_WIDTH 0xF8 = 0b11111000
	uint8_t temp = rx_data[2] & 0xF8;

	// Escribo bits de Z_WIDTH deseado
	temp |= z_width;

	tx_data[0] = MT6835_WRITE << 4;
	tx_data[2] = temp;

	if (mt6835_spi_tx(hmt6835, tx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	// Leo configuracion luego de escribirla para verificar que este bien escrita
	if (mt6835_get_z_width(hmt6835, &z_width_check) != MT6835_OK) {
		return MT6835_SPI_ERROR;
	}

	if (z_width_check != z_width) {
		return MT6835_CHECK_ERROR;
	}

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_get_z_phase(mt6835_t *hmt6835, MT6835_Z_PHASE_e *z_phase) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_Z_PHASE;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	*z_phase = (rx_data[2] >> 6) & 0x03;

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_set_z_phase(mt6835_t *hmt6835, MT6835_Z_PHASE_e z_phase) {
	if (z_phase > MT6835_Z_BFE) {
		return MT6835_INV_PHASE;
	}

	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};
	uint8_t z_phase_check = UINT8_MAX;

	// Primero leo para no pisar UVW config
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_Z_PHASE;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	// Limpio bits de Z_PHASE 0x3F = 0b00111111
	uint8_t temp = rx_data[2] & 0x3F;

	// Escribo Z_PHASE deseado
	temp |= (z_phase << 6);

	tx_data[0] = MT6835_WRITE << 4;
	tx_data[2] = temp;

	if (mt6835_spi_tx(hmt6835, tx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	// Leo configuracion luego de escribirla para verificar que este bien escrita
	if (mt6835_get_z_phase(hmt6835, &z_phase_check) != MT6835_OK) {
		return MT6835_SPI_ERROR;
	}

	if (z_phase_check != z_phase) {
		return MT6835_CHECK_ERROR;
	}

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_get_rot_dir(mt6835_t *hmt6835, MT6835_ROT_DIR_e *rot_dir) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ROT_DIR;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	*rot_dir = (rx_data[2] >> 3) & 0x01;

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_set_rot_dir(mt6835_t *hmt6835, MT6835_ROT_DIR_e rot_dir) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};
	uint8_t rot_dir_check = UINT8_MAX;

	// Primero leo para no pisar bits del registro
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ROT_DIR;

	if (mt6835_spi_txrx(hmt6835, tx_data, rx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	// Limpio bits de ROT_DIR 0xF7 = 0b11110111
	uint8_t temp = rx_data[2] & 0xF7;

	// Si rot_dir > 0 pongo registro en 1: A leads B in CCW rotation
	temp = (rot_dir > 0) ? (temp | 0x08) : (temp & 0xF7);

	tx_data[0] = MT6835_WRITE << 4;
	tx_data[2] = temp;

	if (mt6835_spi_tx(hmt6835, tx_data, 3) != HAL_OK) {
		return MT6835_SPI_ERROR;
	}

	// Leo configuracion luego de escribirla para verificar que este bien escrita
	if (mt6835_get_rot_dir(hmt6835, &rot_dir_check) != MT6835_OK) {
		return MT6835_SPI_ERROR;
	}

	if (rot_dir_check != rot_dir) {
		return MT6835_CHECK_ERROR;
	}

	return MT6835_OK;
}

MT6835_RETURNS_e mt6835_init(mt6835_t *hmt6835, mt6835_settings_t *settings) {
	// En caso que el CS no este configurado para estar en HIGH al iniciar el micro
	HAL_GPIO_WritePin(hmt6835->CS_port, hmt6835->CS_pin, GPIO_PIN_SET);

	MT6835_RETURNS_e err;

	hmt6835->settings.init = MT6835_NOT_INIT;

	// Escribo resolucion modo ABZ
	err = mt6835_set_abz_res(hmt6835, settings->abz_res);

	if (err != MT6835_OK) {
		return err;
	}

	// Enciendo modo ABZ
	err = mt6835_set_abz_off(hmt6835, settings->abz_off);

	if (err != MT6835_OK) {
		return err;
	}

	// Configuro intercambio de canales AB
	err = mt6835_set_abz_swap(hmt6835, settings->abz_swap);

	if (err != MT6835_OK) {
		return err;
	}

	// Configuro alineacion de los flanco de Z con 0
	err = mt6835_set_z_edge(hmt6835, settings->z_edge);

	if (err != MT6835_OK) {
		return err;
	}

	// Configuro ancho de pulso de Z
	err = mt6835_set_z_width(hmt6835, settings->z_width);

	if (err != MT6835_OK) {
		return err;
	}

	// Configuro fase de Z
	err = mt6835_set_z_phase(hmt6835, settings->z_phase);

	if (err != MT6835_OK) {
		return err;
	}

	// Configuro orden de canales AB al girar
	err = mt6835_set_rot_dir(hmt6835, settings->rot_dir);

	if (err != MT6835_OK) {
		return err;
	}

	hmt6835->settings = *settings;
	hmt6835->settings.init = MT6835_INIT;

	return MT6835_OK;
}













