#include "sensor_handler.h"

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t adxl345;
static i2c_master_dev_handle_t vl6180x;

//Settings
#define DISTANCE_AVG_SAMPLES 5

// ============================================================
// I2C helpers
// ============================================================

esp_err_t write_reg8(
    i2c_master_dev_handle_t dev,
    uint16_t reg,
    uint8_t value)
{
    uint8_t data[3];

    data[0] = (reg >> 8) & 0xFF;
    data[1] = reg & 0xFF;
    data[2] = value;

    return i2c_master_transmit(dev, data, 3, 100);
}


esp_err_t read_reg8(
    i2c_master_dev_handle_t dev,
    uint16_t reg,
    uint8_t *value)
{
    uint8_t reg_data[2];

    reg_data[0] = (reg >> 8) & 0xFF;
    reg_data[1] = reg & 0xFF;

    return i2c_master_transmit_receive(
        dev,
        reg_data,
        2,
        value,
        1,
        100
    );
}


// ============================================================
// ADXL345
// ============================================================

#define ADXL345_REG_DEVID       0x00
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0      0x32

void adxl345_init(void)
{
    uint8_t data;

    // Controleren of ADXL345 aanwezig is
    uint8_t reg = ADXL345_REG_DEVID;

    esp_err_t err = i2c_master_transmit_receive(
        adxl345,
        &reg,
        1,
        &data,
        1,
        100
    );

    if (err != ESP_OK)
    {
        printf("Fout bij lezen ADXL345 ID: %s\n",
               esp_err_to_name(err));
        return;
    }

    printf("ADXL345 Device ID: 0x%02X\n", data);

    // DATA_FORMAT:
    // 0x0B = FULL_RES + +/-16g
    uint8_t config1[] = {
        ADXL345_REG_DATA_FORMAT,
        0x0B
    };

    err = i2c_master_transmit(
        adxl345,
        config1,
        2,
        100
    );

    if (err != ESP_OK)
    {
        printf("Fout DATA_FORMAT: %s\n",
               esp_err_to_name(err));
        return;
    }

    // POWER_CTL:
    // 0x08 = Measurement Mode
    uint8_t config2[] = {
        ADXL345_REG_POWER_CTL,
        0x08
    };

    err = i2c_master_transmit(
        adxl345,
        config2,
        2,
        100
    );

    if (err != ESP_OK)
    {
        printf("Fout POWER_CTL: %s\n",
               esp_err_to_name(err));
        return;
    }

    // Controleren of POWER_CTL daadwerkelijk 0x08 bevat
    reg = ADXL345_REG_POWER_CTL;

    err = i2c_master_transmit_receive(
        adxl345,
        &reg,
        1,
        &data,
        1,
        100
    );

    if (err == ESP_OK)
    {
        printf("ADXL345 POWER_CTL: 0x%02X\n", data);
    }
    else
    {
        printf("Fout lezen POWER_CTL: %s\n",
               esp_err_to_name(err));
    }

    printf("ADXL345 gestart.\n");
}


void adxl345_read(float *x, float *y, float *z)
{
    // ADXL345:
    // Bit 7 = READ
    // Bit 6 = MULTI-BYTE
    //
    // 0x32 | 0x80 | 0x40 = 0xF2

    uint8_t reg = ADXL345_REG_DATAX0 | 0x80 | 0x40;
    uint8_t data[6];

    esp_err_t err = i2c_master_transmit_receive(
        adxl345,
        &reg,
        1,
        data,
        6,
        100
    );

    if (err != ESP_OK)
    {
        printf(
            "Fout bij lezen ADXL345: %s\n",
            esp_err_to_name(err)
        );
        return;
    }

    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    /*printf(
        "RAW ADXL345: X=%d Y=%d Z=%d | ",
        raw_x,
        raw_y,
        raw_z
    );*/

    // Full-resolution: ongeveer 3.9 mg/LSB
    *x = raw_x * 0.0039f;
    *y = raw_y * 0.0039f;
    *z = raw_z * 0.0039f;
}


// ============================================================
// VL6180X
// ============================================================

void vl6180x_init(void)
{
    uint8_t reset_status = 0;

    read_reg8(vl6180x, 0x0016, &reset_status);

    printf("VL6180X reset status: 0x%02X\n", reset_status);

    // Recommended VL6180X tuning settings
    write_reg8(vl6180x, 0x0207, 0x01);
    write_reg8(vl6180x, 0x0208, 0x01);
    write_reg8(vl6180x, 0x0096, 0x00);
    write_reg8(vl6180x, 0x0097, 0xFD);
    write_reg8(vl6180x, 0x00E3, 0x00);
    write_reg8(vl6180x, 0x00E4, 0x04);
    write_reg8(vl6180x, 0x00E5, 0x02);
    write_reg8(vl6180x, 0x00E6, 0x01);
    write_reg8(vl6180x, 0x00E7, 0x03);
    write_reg8(vl6180x, 0x00F5, 0x02);
    write_reg8(vl6180x, 0x00D9, 0x05);
    write_reg8(vl6180x, 0x00DB, 0xCE);
    write_reg8(vl6180x, 0x00DC, 0x03);
    write_reg8(vl6180x, 0x00DD, 0xF8);
    write_reg8(vl6180x, 0x009F, 0x00);
    write_reg8(vl6180x, 0x00A3, 0x3C);
    write_reg8(vl6180x, 0x00B7, 0x00);
    write_reg8(vl6180x, 0x00BB, 0x3C);
    write_reg8(vl6180x, 0x00B2, 0x09);
    write_reg8(vl6180x, 0x00CA, 0x09);
    write_reg8(vl6180x, 0x0198, 0x01);
    write_reg8(vl6180x, 0x01B0, 0x17);
    write_reg8(vl6180x, 0x01AD, 0x00);
    write_reg8(vl6180x, 0x00FF, 0x05);
    write_reg8(vl6180x, 0x0100, 0x05);
    write_reg8(vl6180x, 0x0199, 0x05);
    write_reg8(vl6180x, 0x01A6, 0x1B);
    write_reg8(vl6180x, 0x01AC, 0x3E);
    write_reg8(vl6180x, 0x01A7, 0x1F);
    write_reg8(vl6180x, 0x0030, 0x00);

    // Recommended public settings
    write_reg8(vl6180x, 0x0011, 0x10);
    write_reg8(vl6180x, 0x010A, 0x30);
    write_reg8(vl6180x, 0x003F, 0x46);
    write_reg8(vl6180x, 0x0031, 0xFF);
    write_reg8(vl6180x, 0x0040, 0x63);

    // Clear interrupt
    write_reg8(vl6180x, 0x0015, 0x07);

    printf("VL6180X gestart.\n");
}


/*int vl6180x_read_distance(void)
{
    uint8_t status = 0;
    uint8_t distance = 0;

    // Start single-shot ranging
    write_reg8(vl6180x, 0x0018, 0x01);

    // Wait until measurement is ready
    for (int i = 0; i < 100; i++)
    {
        read_reg8(vl6180x, 0x004F, &status);

        if ((status & 0x04) != 0)
        {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Read distance in mm
    if (read_reg8(vl6180x, 0x0062, &distance) != ESP_OK)
    {
        return -1;
    }

    // Clear interrupt
    write_reg8(vl6180x, 0x0015, 0x07);

    return distance;
}*/

int vl6180x_read_distance(void)
{
    uint8_t status = 0;
    uint8_t distance = 0;

    // Start een nieuwe meting
    if (write_reg8(vl6180x, 0x0018, 0x01) != ESP_OK) {
        printf("Fout: starten van meting mislukt\n");
        return -1;
    }

    bool ready = false;

    // Wacht maximaal 100 ms op een nieuwe meting
    for (int i = 0; i < 100; i++)
    {
        if (read_reg8(vl6180x, 0x004F, &status) != ESP_OK) {
            printf("Fout: statusregister lezen mislukt\n");
            return -1;
        }

        if ((status & 0x04) != 0) {
            ready = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Lees geen resultaat als de meting niet klaar is
    if (!ready) {
        printf("Fout: timeout, status = 0x%02X\n", status);
        return -1;
    }

    // Lees de afstand
    if (read_reg8(vl6180x, 0x0062, &distance) != ESP_OK) {
        printf("Fout: afstandsregister lezen mislukt\n");
        return -1;
    }

    // Wis de interruptstatus
    if (write_reg8(vl6180x, 0x0015, 0x07) != ESP_OK) {
        printf("Waarschuwing: interrupt wissen mislukt\n");
    }

    return (int)distance;
}

bool car_on_ground(void)
{
    static int samples[DISTANCE_AVG_SAMPLES];
    static int index = 0;     // volgende plek in de buffer
    static int count = 0;     // aantal geldige metingen in de buffer
    static int sum = 0;       // som van de metingen in de buffer

    int distance = vl6180x_read_distance();
    printf("Afstandssensor: %d mm\n", distance);
    if (distance < 0)
    {
        printf("Fout bij lezen afstandssensor.\n");

        // Buffer leegmaken zodat oude waarden niet worden meegenomen
        index = 0;
        count = 0;
        sum = 0;

        return false;
    }

    // Oudste waarde eruit, nieuwe waarde erin
    if (count == DISTANCE_AVG_SAMPLES)
    {
        sum -= samples[index];
    }
    else
    {
        count++;
    }

    samples[index] = distance;
    sum += distance;
    index = (index + 1) % DISTANCE_AVG_SAMPLES;

    int average = sum / count;

    // Drempelwaarde in mm om te bepalen of de auto op de grond staat
    const int threshold = 150;

    return average < threshold;
}

void sensors_init(void)
{
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = (gpio_num_t)SDA_PIN;
    bus_config.scl_io_num = (gpio_num_t)SCL_PIN;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    i2c_device_config_t adxl_config = {};
    adxl_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    adxl_config.device_address = ADXL345_ADDR;
    adxl_config.scl_speed_hz = I2C_FREQUENCY;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &adxl_config, &adxl345));

    i2c_device_config_t tof_config = {};
    tof_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    tof_config.device_address = VL6180X_ADDR;
    tof_config.scl_speed_hz = I2C_FREQUENCY;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &tof_config, &vl6180x));

    adxl345_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    vl6180x_init();
}