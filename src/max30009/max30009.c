#include "max30009.h"
#include "spi_functions.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MAX30009, LOG_LEVEL_DBG);

max30009_conf_t_p current_config = NULL;

static int max30009_reg_update(uint8_t reg, uint8_t val)
{
    int err = 0;
    uint8_t current = 0;

    err = spi_read_reg(reg, &current);
    if(err){
        return err;
    }

    current |= val;

    err = spi_write_reg(reg, current);

    return err;
}

static int max30009_soft_reset()
{
    if(spi_write_reg(BIOZ_CONFIGURATION_1_REGISTER, (1 << BIOZ_BG_EN_SHIFT))){
        return -EIO;
    }

    if(spi_write_reg(SYSTEM_CONFIGURATION_1_REGISTER, 0)){
        return -EIO;
    }

    if(spi_write_reg(PLL_CONFIGURATION_4_REGISTER, 0)){
        return -EIO;
    }

    if(spi_write_reg(PLL_CONFIGURATION_1_REGISTER, 0)){
        return -EIO;
    }

    if(spi_write_reg(SYSTEM_CONFIGURATION_1_REGISTER, (1 << RESET_SHIFT))){
        return -EIO;
    }

    k_sleep(K_MSEC(10));

    return 0;
}

static int pll_enable_procedure()
{
    int err = 0;
    uint8_t bioz_status = 0;

    err = spi_read_reg(BIOZ_CONFIGURATION_1_REGISTER, &bioz_status); 
    if(err){
        return err;
    }

    if(bioz_status & (1 << BIOZ_I_EN_SHIFT) || bioz_status & (1 << BIOZ_Q_EN_SHIFT)){
        err = max30009_stop_bioz();
        if(err){
            return err;
        }
    }

    k_sleep(K_MSEC(10));

    err = max30009_reg_update(PLL_CONFIGURATION_1_REGISTER, (1 << PLL_EN_SHIFT));

    return err;
}

static int pll_disable_procedure()
{
    int err = 0;
    uint8_t bioz_status = 0;

    if(current_config == NULL){
        return -ECANCELED;
    }

    err = spi_read_reg(BIOZ_CONFIGURATION_1_REGISTER, &bioz_status); 
    if(err){
        return err;
    }

    if(bioz_status & (1 << BIOZ_I_EN_SHIFT) || bioz_status & (1 << BIOZ_Q_EN_SHIFT)){
        err = max30009_stop_bioz();
        if(err){
            return err;
        }
    }

    k_sleep(K_MSEC(10));

    err = spi_write_reg(PLL_CONFIGURATION_1_REGISTER, (((current_config->bioz.mdiv >> 8) & 0x02) << MDIV_MSB_SHIFT) |
                                                        (current_config->bioz.ndiv << NDIV_SHIFT) | (current_config->bioz.kdiv << KDIV_SHIFT));

    return err;
}

int max30009_init(max30009_conf_t_p conf)
{
    int err = 0;
    uint8_t reg_data;

    err = spi_init();
    if(err){
        return err;
    }

    err = max30009_soft_reset();
    if(err){
        LOG_ERR("Reset procedure fail");
        return err;
    } 

    // Check part id
    err = spi_read_reg(PART_ID_REGISTER, &reg_data);
    if(err){
        LOG_ERR("Read part id fail");
        return err;
    }
    else if(reg_data != 0x42){
        LOG_ERR("Device not found, expected 0x42, got 0x%02x", reg_data);
        return -ENODEV;
    }

    // SPI only mode
    err = spi_write_reg(SYSTEM_CONFIGURATION_1_REGISTER, (1 << DISABLE_I2C_SHIFT));
    if(err){
        LOG_ERR("Set SPI only fail");
        return err;
    }


    err = spi_write_reg(PLL_CONFIGURATION_2_REGISTER, (conf->bioz.mdiv & 0xff));
    if(err){
        LOG_ERR("mdiv(low) set fail");
        return err;
    }

    err = spi_write_reg(PLL_CONFIGURATION_1_REGISTER, (((conf->bioz.mdiv >> 8) & 0x02) << MDIV_MSB_SHIFT) |
                                                        (conf->bioz.ndiv << NDIV_SHIFT) | (conf->bioz.kdiv << KDIV_SHIFT));
    if(err){
        LOG_ERR("MDIV MSB set fail");
        return err;
    }

    // always using 32.768kHz
    err = spi_write_reg(PLL_CONFIGURATION_4_REGISTER, (1 << CLK_FREQ_SEL_SHIFT));
    if(err){
        LOG_ERR("CLK frequency set fail");
        return err;
    }

    err = spi_write_reg(BIOZ_CONFIGURATION_1_REGISTER, (conf->bioz.dac_osr << BIOZ_DAC_OSR_SHIFT) |
                                                        (conf->bioz.adc_osr << BIOZ_ADC_OSR_SHIFT));
    if(err){
        LOG_ERR("OSR config fail");
        return err;
    }

    current_config = conf;

    err = max30009_enable_bioz_reference();
    if(err){
        LOG_ERR("Enable BioZ reference fail");
        return err;
    }

    return 0;
}

int max30009_enable_bioz_reference()
{
    int err = 0;

    err = max30009_reg_update(BIOZ_CONFIGURATION_1_REGISTER, (1 << BIOZ_BG_EN_SHIFT));
    if(err == 0){
        k_sleep(K_MSEC(200));
    }

    return err;
}

int max30009_enable_pll(int en)
{
    if(en){
        return pll_enable_procedure();
    }
    else{
        return pll_disable_procedure();
    }
}

int max30009_start_bioz()
{
    int err = 0;

    err = max30009_reg_update(BIOZ_CONFIGURATION_1_REGISTER, (1 << BIOZ_I_EN_SHIFT) | (1 << BIOZ_Q_EN_SHIFT));
    if(err){
        LOG_ERR("Bioz start fail");
        return err;
    }

    k_sleep(K_MSEC(2));
   
    return 0;
}

int max30009_stop_bioz()
{
    int err = 0;

    if(current_config == NULL){
        return -ECANCELED;
    }

    err = spi_write_reg(BIOZ_CONFIGURATION_1_REGISTER, (current_config->bioz.dac_osr << BIOZ_DAC_OSR_SHIFT) |
                                                        (current_config->bioz.adc_osr << BIOZ_ADC_OSR_SHIFT) |
                                                        (1 << BIOZ_BG_EN_SHIFT));
    if(err){
        LOG_ERR("Bioz stop fail");
        return err;
    }

    k_sleep(K_MSEC(2));
   
    return 0;    
}

int max30009_set_stimulus_mode(max30009_stimulus_modes_e stimulus_mode)
{
    int err = 0;

    if(stimulus_mode == MAX30009_STIMULUS_MODE_SINEWAVE_CURRENT){
        err = spi_write_reg(BIOZ_CONFIGURATION_3_REGISTER, 0x00);
        if(err){
            LOG_ERR("Sine wave current mode fail");
            return err;
        }
    }
    else if(stimulus_mode == MAX30009_STIMULUS_MODE_SINEWAVE_VOLTAGE){
        err = spi_write_reg(BIOZ_CONFIGURATION_3_REGISTER, (0x01 << BIOZ_DRV_MODE_SHIFT));
        if(err){
            LOG_ERR("Sine wave voltage mode fail");
            return err;
        }
    }
    else{
        // H-Bridge
        return 0;
    }

    return 0;
}

int max30009_set_fifo_afull_threshold(uint8_t a_full)
{
    int err = 0;    

    err = spi_write_reg(FIFO_CONFIGURATION_1_REGISTER, a_full);
    if(err){
        LOG_ERR("Config A_FULL fail");
    }

    return err;
}

int max30009_conf_int_pin(max30009_int_pin_conf_e int_pin_config)
{
    int err = 0;

    err = spi_write_reg(PIN_FUNCTIONAL_CONFIGURATION_REGISTER, (int_pin_config << INT_FCFG_SHIFT));
    if(err){
        LOG_ERR("Int pin function config fail");
    }

    return err;
}

int max30009_conf_int_pin_output(max30009_int_pin_output_conf_e int_output_config)
{
    int err = 0;

    err = spi_write_reg(OUTPUT_PIN_CONFIGURATION_REGISTER, (int_output_config << INT_OCFG_SHIFT));
    if(err){
        LOG_ERR("Pin output config fail");
    }

    return err;
}

int max30009_interrupt_enable_1(uint8_t int_en_mask)
{
    int err = 0;

    err = spi_write_reg(INTERRUPT_ENABLE_1_REGISTER, int_en_mask);
    if(err){
        LOG_ERR("Enable interrupt fail");
    }

    return err;
}

int max30009_bioz_enable_mux()
{
    int err = 0;

    err = spi_write_reg(BIOZ_MUX_CONFIGURATION_1_REGISTER, (1 << MUX_EN_SHIFT));
    if(err){
        LOG_ERR("MUX enable fail");
    }

    return err;
}

int max30009_conf_assign_electrodes(max30009_mux_bip_conf_e bip, max30009_mux_bin_conf_e bin, 
                        max30009_mux_drvp_conf_e drvp, max30009_mux_drvn_conf_e drvn)
{
    int err = 0;

    err = spi_write_reg(BIOZ_MUX_CONFIGURATION_3_REGISTER, (bip << BIP_ASSIGN_SHIFT) | (bin << BIN_ASSIGN_SHIFT) |
                                                            (drvp << DRVP_ASSIGN_SHIFT) | (drvn << DRVN_ASSIGN_SHIFT));
    if(err){
        LOG_ERR("Electrode assign fail");
    }

    return err;
}

int max30009_get_status_1(uint8_t *status)
{
    int err = 0;

    err = spi_read_reg(STATUS_1_REGISTER, status);
    if(err){
        LOG_ERR("Read status 1 fail");
        return err;
    }

    return 0;
}

int max30009_read_fifo(uint8_t *data, uint8_t data_len)
{
    int err = 0;

    err = spi_burst_read(FIFO_DATA_REGISTER_REGISTER, data, data_len);
    if(err){
        LOG_ERR("FIFO read fail");
        return err;
    }

    return 0;
}

int max30009_parse_fifo_data(uint8_t *raw, int32_t *data)
{
    int tag = 0;
    int32_t raw_data_24 = 0;

    raw_data_24 = raw[0];
    raw_data_24 <<= 8;
    raw_data_24 |= raw[1];
    raw_data_24 <<= 8;
    raw_data_24 |= raw[2];

    tag = (raw[0] >> 4) & 0x0f;

    if(tag < 0xff){
        *data = raw_data_24 & 0x000fffff;
    }
    else{
        if(raw_data_24 == 0x00ffffff){
            tag = 0; // Invalid data
        }
        else{
            tag = 0xff; // Marker
        }
    }

    return tag;
}