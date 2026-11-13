#include "sensor_handler.h"
#include <math.h>

#define PITCH_LPF_ALPHA   0.5f   // 0..1: lager = vloeiender maar trager
#define PITCH_SIGN        1.0f   // zet op -1.0f als voorover kantelen een negatieve hoek geeft

static volatile float pitch_deg = 0.0f;

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t adxl345;
static i2c_master_dev_handle_t vl6180x;

// ============================================================
// Instellingen
// ============================================================

#define DISTANCE_AVG_SAMPLES   5     // aantal metingen voor het moving average
#define GROUND_THRESHOLD_MM    92    // gemiddelde > drempel  => auto staat op de grond
#define SENSOR_TASK_PERIOD_MS  20    // 50 metingen per seconde (de meting zelf duurt ook even)
#define SENSOR_TASK_STACK      4096  // stackgrootte in bytes
#define SENSOR_TASK_PRIORITY   3     // laag, zodat Bluetooth niet gestoord wordt
#define SENSOR_TASK_CORE       0     // 0 of 1

// Pas dit aan op hoe de ADXL345 gemonteerd zit (teken mag ook: -ay)
#define ACC_FWD   (ay)   // as die naar voren/achteren wijst
#define ACC_SIDE  (ax)   // as die naar links/rechts wijst
#define ACC_UP    (az)   // as die omhoog wijst

// ============================================================
// Gedeelde status (geschreven door de sensortaak, gelezen door loop())
// ============================================================

static volatile bool on_ground = false;       // veilige beginwaarde: in de lucht
static volatile int  last_distance_mm = -1;

// ============================================================
// I2C helpers
// ============================================================

static esp_err_t write_reg8(
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

static esp_err_t read_reg8(
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
// Init van de I2C-bus en beide sensoren
// ============================================================

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

    // DATA_FORMAT: 0x0B = FULL_RES + +/-16g
    uint8_t config1[] = {
        ADXL345_REG_DATA_FORMAT,
        0x0B
    };

    err = i2c_master_transmit(adxl345, config1, 2, 100);

    if (err != ESP_OK)
    {
        printf("Fout DATA_FORMAT: %s\n", esp_err_to_name(err));
        return;
    }

    // POWER_CTL: 0x08 = Measurement Mode
    uint8_t config2[] = {
        ADXL345_REG_POWER_CTL,
        0x08
    };

    err = i2c_master_transmit(adxl345, config2, 2, 100);

    if (err != ESP_OK)
    {
        printf("Fout POWER_CTL: %s\n", esp_err_to_name(err));
        return;
    }

    // Controleren of POWER_CTL daadwerkelijk 0x08 bevat
    reg = ADXL345_REG_POWER_CTL;

    err = i2c_master_transmit_receive(adxl345, &reg, 1, &data, 1, 100);

    if (err == ESP_OK)
    {
        printf("ADXL345 POWER_CTL: 0x%02X\n", data);
    }
    else
    {
        printf("Fout lezen POWER_CTL: %s\n", esp_err_to_name(err));
    }

    printf("ADXL345 gestart.\n");
}

void adxl345_read(float *x, float *y, float *z)
{
    // Bij I2C is het registeradres gewoon 0x32; meerdere bytes achter
    // elkaar lezen werkt automatisch (de 0x80/0x40-bits zijn voor SPI).
    uint8_t reg = ADXL345_REG_DATAX0;
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
        //printf("Fout bij lezen ADXL345: %s\n", esp_err_to_name(err));
        return;
    }

    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

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

    if (read_reg8(vl6180x, 0x0016, &reset_status) != ESP_OK) {
        printf("VL6180X: geen antwoord bij init\n");
        return;
    }

    printf("VL6180X reset status: 0x%02X\n", reset_status);

    if (reset_status != 0x01) {
        printf("VL6180X al geconfigureerd.\n");
        return;
    }

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
    write_reg8(vl6180x, 0x0011, 0x10);   // GPIO1 interrupt-pin
    write_reg8(vl6180x, 0x010A, 0x30);   // averaging sample period
    write_reg8(vl6180x, 0x003F, 0x46);   // ALS analogue gain
    write_reg8(vl6180x, 0x0031, 0xFF);   // VHV repeat rate
    write_reg8(vl6180x, 0x0040, 0x00);   // ALS integration period (hoog)
    write_reg8(vl6180x, 0x0041, 0x63);   // ALS integration period (laag)
    write_reg8(vl6180x, 0x002E, 0x01);   // VHV herkalibratie
    write_reg8(vl6180x, 0x001B, 0x09);   // range meetinterval
    write_reg8(vl6180x, 0x003E, 0x31);   // ALS meetinterval
    write_reg8(vl6180x, 0x0014, 0x24);   // interrupt: nieuwe afstandsmeting klaar

    // Reset-vlag wissen en interrupts wissen
    write_reg8(vl6180x, 0x0016, 0x00);
    write_reg8(vl6180x, 0x0015, 0x07);

    printf("VL6180X gestart.\n");
}

int vl6180x_read_distance(void)
{
    uint8_t v = 0;
    int64_t start;

    // 1. Is de sensor opnieuw opgestart? Dan eerst opnieuw initialiseren.
    if (read_reg8(vl6180x, 0x0016, &v) != ESP_OK) return -1;
    if (v == 0x01) {
        printf("VL6180X was gereset, opnieuw initialiseren\n");
        vl6180x_init();
    }

    // 2. Wacht tot de sensor klaar is voor een nieuwe meting (bit 0 van 0x004D)
    start = esp_timer_get_time();
    do {
        if (read_reg8(vl6180x, 0x004D, &v) != ESP_OK) return -1;
        if (v & 0x01) break;
        vTaskDelay(1);                       // minimaal 1 tick
    } while (esp_timer_get_time() - start < 50000);
    if (!(v & 0x01)) {
        printf("Fout: sensor niet klaar, 0x4D = 0x%02X\n", v);
        return -1;
    }

    // 3. Start een enkele meting
    if (write_reg8(vl6180x, 0x0018, 0x01) != ESP_OK) return -1;

    // 4. Wacht op "nieuwe meting klaar" (onderste 3 bits van 0x4F = 4)
    start = esp_timer_get_time();
    do {
        if (read_reg8(vl6180x, 0x004F, &v) != ESP_OK) return -1;
        if ((v & 0x07) == 0x04) break;
        vTaskDelay(1);
    } while (esp_timer_get_time() - start < 100000);   // 100 ms echte tijd

    if ((v & 0x07) != 0x04) {
        printf("Fout: timeout, 0x4F = 0x%02X\n", v);
        write_reg8(vl6180x, 0x0015, 0x07);              // interrupts wissen
        return -1;
    }

    // 5. Afstand lezen en interrupts wissen
    uint8_t mm = 0;
    esp_err_t err = read_reg8(vl6180x, 0x0062, &mm);
    write_reg8(vl6180x, 0x0015, 0x07);

    return (err == ESP_OK) ? (int)mm : -1;
}

// ============================================================
// Sensortaak: meet continu en berekent het moving average
// ============================================================

static void sensor_task(void *arg)
{
    int samples[DISTANCE_AVG_SAMPLES] = {0};
    int index = 0;     // volgende plek in de buffer
    int count = 0;     // aantal geldige metingen in de buffer
    int sum = 0;       // som van de metingen in de buffer

    while (1)
    {
        int distance = vl6180x_read_distance();
        last_distance_mm = distance;

        if (distance < 0)
        {
            // Bij een fout: buffer leegmaken en veilig "in de lucht" melden
            index = 0;
            count = 0;
            sum = 0;
            on_ground = false;
        }
        else
        {
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

            // Let op: in jouw opstelling betekent een HOGE waarde "op de grond"
            on_ground = (average > GROUND_THRESHOLD_MM);
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

static void attitude_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    while (1)
    {
        float ax = NAN, ay = NAN, az = NAN;
        adxl345_read(&ax, &ay, &az);
        if (!isnan(ax) && !isnan(ay) && !isnan(az))
        {
            float raw = atan2f(ACC_FWD, sqrtf(ACC_SIDE * ACC_SIDE + ACC_UP * ACC_UP)) * 57.29578f;
            raw *= PITCH_SIGN;
            pitch_deg += (raw - pitch_deg) * PITCH_LPF_ALPHA;
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));   // 100 Hz
    }
}

void sensors_start_task(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(
        sensor_task,            // functie
        "sensor_task",          // naam
        SENSOR_TASK_STACK,      // stackgrootte in bytes
        NULL,                   // parameter
        SENSOR_TASK_PRIORITY,   // prioriteit
        NULL,                   // task handle (niet nodig)
        SENSOR_TASK_CORE        // core
    );

    xTaskCreatePinnedToCore(attitude_task, "attitude", 4096, NULL, 3, NULL, SENSOR_TASK_CORE);

    if (ok != pdPASS)
    {
        printf("Fout: sensortaak kon niet gestart worden\n");
    }
}

// ============================================================
// Publieke getters (geen I2C, geven direct de laatste waarde terug)
// ============================================================

bool car_on_ground(void)
{
    return on_ground;
}

int sensors_last_distance(void)
{
    return last_distance_mm;
}

float sensors_pitch_deg(void)
{
    return pitch_deg;
}