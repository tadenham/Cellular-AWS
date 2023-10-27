#include <stdio.h> //Standard C library
#include <inttypes.h>
#include <math.h> //Math function library
#include "nvs.h" //Non-volatile storage
#include "nvs_flash.h" //Non-volatile storage
#include "string.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_adc/adc_oneshot.h" //ADC
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h" //ULP
#include "soc/sens_reg.h" //ULP
#include "soc/rtc_periph.h" //ULP
#include "driver/i2c.h" //I2C
#include "driver/gpio.h" //GPIO
#include "driver/rtc_io.h" //GPIO
#include "driver/uart.h" //UART
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include <sys/time.h> //time
#include <time.h> //time
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h" //SD card
#include "sdmmc_cmd.h" //SD card

#include "bmx280.h" //BMP280 pressure sensor

#define MEASURE_INTERVAL_SEC 10
#define STORE_INTERVAL_MIN 30
#define TRANSMIT_INTERVAL_MIN 30
#define STORE_SD false

#define AVERAGING true
#define MEASURE_PRESSURE false

//I2C definitions
#define I2C_MASTER_NUM 0
#define I2C_MASTER_TIMEOUT_MS 1000
#define I2C_CLK_SPEED 100000 //100 kHz
#define SDA_PIN 21
#define SCL_PIN 22

#if STORE_SD
    //SD Card pins
    #define PIN_NUM_MISO 2
    #define PIN_NUM_MOSI 15
    #define PIN_NUM_CLK 14
    #define PIN_NUM_CS 13
#endif

//Pin definitions
#define RAIN_PIN GPIO_NUM_39 //GPIO_NUM_39: Use ADC_CHANNEL_3 and ADC_UNIT_1; GPIO_NUM_2: Use ADC_CHANNEL_2 and ADC_UNIT_2. Cannot use GPIO_NUM_2 with SD card. 
//#define WIND_PIN GPIO_NUM_39 //GPIO_NUM_39: Use ADC_CHANNEL_3 and ADC_UNIT_1
#define BAT_ADC_UNIT ADC_UNIT_1
#define BAT_ADC ADC_CHANNEL_7
#define RAIN_ADC_UNIT ADC_UNIT_1
#define RAIN_ADC ADC_CHANNEL_3

#define RAIN_ADC_UNIT ADC_UNIT_1
#define RAIN_ADC ADC_CHANNEL_3

#define MODEM 1 //MODEM 0 = A7670, MODEM 1 = SIM7600
#define MODEM_SLEEP 0 //0 = Power off, 1 = Modem sleep

#define MODEM_PWRKEY_PIN GPIO_NUM_4

#if MODEM == 0 //A7670
    #define TXD_PIN 26
    #define RXD_PIN 27
    #define MODEM_DTR_PIN GPIO_NUM_25
    #define BAT_EN GPIO_NUM_12
    #define RESET GPIO_NUM_5
#elif MODEM == 1 //SIM7600
    #define TXD_PIN 27
    #define RXD_PIN 26
    #define MODEM_DTR_PIN GPIO_NUM_32
    #define FLIGHT_PIN GPIO_NUM_25
#endif    

#define UART_BAUD 115200
#define RX_BUF_SIZE 1024

#define PROCEED_BIT BIT0
#define MEASURE_BIT BIT1

RTC_DATA_ATTR int lastFormatMin = -1;

#if STORE_SD
    RTC_DATA_ATTR uint16_t messageQueue;
    sdmmc_card_t *card;
#endif

RTC_DATA_ATTR char imei[16];

static const char *TAG = "AWS";

char* callbackString = "";

i2c_cmd_handle_t cmd;

static EventGroupHandle_t event_group = NULL;

float temp = -9999;
RTC_DATA_ATTR float tempi[] = {-9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999};
RTC_DATA_ATTR float maxTemp = -9999;
RTC_DATA_ATTR float minTemp = 9999;

float dew = -9999;
RTC_DATA_ATTR float dewi[] = {-9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999};
RTC_DATA_ATTR float maxDew = -9999;
RTC_DATA_ATTR float minDew = 9999;

float humidity = -9999;
RTC_DATA_ATTR float humidityi[] = {-9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999};
RTC_DATA_ATTR float maxHumidity = -9999;
RTC_DATA_ATTR float minHumidity = 9999;

RTC_DATA_ATTR bool rainGauge;
RTC_DATA_ATTR uint32_t bootCount;

#if MEASURE_PRESSURE
    float pressure;
#endif

float vbat;

static esp_err_t i2c_master_init(void){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLK_SPEED,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void uart_init(){
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/**
* @brief Publishes MQTT message. connectMQTT must have already been called.
* 
* @param command Command to send. Must be terminated by \r.
* @param callback Modem response to wait for
* @param timeToWait Seconds to wait for response before giving up
*/
bool sendAT(const char* command, char* callback, uint8_t timeToWait){
    const int len = strlen(command);
    callbackString = callback;
    uart_write_bytes(UART_NUM_1, command, len);
    EventBits_t uxBits;
    uxBits = xEventGroupWaitBits(event_group, PROCEED_BIT, pdTRUE, pdTRUE, timeToWait*1000 / portTICK_PERIOD_MS);
    if((uxBits & PROCEED_BIT) != 0){
        ESP_LOGI(TAG, "AT command successful");
        return true;
    } else{
        ESP_LOGE(TAG, "AT command fail");
        return false;
    };
}

/**
* @brief Connects to MQTT broker
* 
*/
bool connectMQTT(){
    char client[64] = "AT+CMQTTACCQ=0,\"";
    strncat(client, imei, strlen(imei));
    strncpy(client + 31, "\"", 2);
    strncat(client, "\r", 2);
    printf("Client: %s\n", client);

    sendAT("AT+CMQTTSTART\r", "+CMQTTSTART: 0", 5);
    sendAT(client, "OK", 5);
    if(sendAT("AT+CMQTTCONNECT=0,\"tcp://142.11.236.169:1883\",60,0\r", "+CMQTTCONNECT: 0,0", 5)){
        ESP_LOGI(TAG, "MQTT connect successful");
        return true;
    } else{
        ESP_LOGE(TAG, "MQTT connect fail");
        return false;
    }
}

/**
* @brief Publishes MQTT message. connectMQTT must have already been called.
* 
* @param msg_buf Message buffer
*/
bool publishMQTT(char *msg_buf){
    char topic[21] = "data/";
    strncat(topic, imei, strlen(imei));

    char pub_buf[32] =  "AT+CMQTTPAYLOAD=0,";
    char msg_len[32];
    sprintf(msg_len, "%d", strlen(msg_buf));
    strncat(pub_buf, msg_len, strlen(msg_len));
    strncat(pub_buf, "\r", 2);
    printf("Pub buf: %s\n", pub_buf);

    sendAT("AT+CMQTTTOPIC=0,20\r", ">", 5);
    sendAT(topic, "OK", 5);
    sendAT(pub_buf, ">", 5);
    sendAT(msg_buf, "OK", 5);
    if(sendAT("AT+CMQTTPUB=0,1,60,1\r", "+CMQTTPUB: 0,0", 5)){
        return true;
    } else{
        return false;
    }
    /*sendAT("AT+CMQTTDISC=0\r", "+CMQTTDISC: 0,0", 5);
    sendAT("AT+CMQTTREL=0\r", "OK", 5);
    sendAT("AT+CMQTTSTOP\r", "+CMQTTSTOP: 0", 5);*/
}

#if STORE_SD
    /**
    * @brief Mount SD card
    * 
    */                      
    void mountSD(void){
        esp_err_t ret;

        // Options for mounting the filesystem.
        // If format_if_mount_failed is set to true, SD card will be partitioned and
        // formatted in case when mounting fails.
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            #ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
                .format_if_mount_failed = true,
            #else
                .format_if_mount_failed = false,
            #endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
            .max_files = 5,
            .allocation_unit_size = 16 * 1024
        };
        ESP_LOGI(TAG, "Initializing SD card");

        // Use settings defined above to initialize SD card and mount FAT filesystem.
        // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
        // Please check its source code and implement error recovery when developing
        // production applications.
        ESP_LOGI(TAG, "Using SPI peripheral");

        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = PIN_NUM_MOSI,
            .miso_io_num = PIN_NUM_MISO,
            .sclk_io_num = PIN_NUM_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4000,
        };
        ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize bus.");
            return;
        }

        // This initializes the slot without card detect (CD) and write protect (WP) signals.
        // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
        sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
        slot_config.gpio_cs = PIN_NUM_CS;
        slot_config.host_id = host.slot;

        ESP_LOGI(TAG, "Mounting filesystem");
        ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);

        if (ret != ESP_OK) {
            if (ret == ESP_FAIL) {
                ESP_LOGE(TAG, "Failed to mount filesystem. "
                        "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
            } else {
                ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                        "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
            }
            return;
        }
        ESP_LOGI(TAG, "Filesystem mounted");

        // Card has been initialized, print its properties
        sdmmc_card_print_info(stdout, card);
    }

    /**
    * @brief Write data to file on SD card
    * 
    * @param data Data to write to SD card
    * @param file File to write to
    */
    void writeSD(char* data, char *file){
        ESP_LOGI(TAG, "Opening file %s", file);
        FILE *f = fopen(file, "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for appending");
            return;
        }
        fprintf(f, "%s\n", data);
        fclose(f);
        ESP_LOGI(TAG, "File written");
    }

    /**
    * @brief Deletes file from SD card
    * 
    * @param file File to delete
    */
    void deleteFileSD(char *file){
        ESP_LOGI(TAG, "Deleting file %s", file);
        FILE *f = fopen(file, "w");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for deleting");
            return;
        }
        remove(file);
        ESP_LOGI(TAG, "File deleted");
    }

    /**
    * @brief Reads file from SD card and publishes any queued messages
    * 
    * @param file File to read
    */
    void readSD(char *file){
        ESP_LOGI(TAG, "Reading file %s", file);
        FILE *f = fopen(file, "r");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for reading");
            return;
        }

        // Read a line from file
        char line[128];
        //fgets(line, sizeof(line), f);

        while (fgets(line, sizeof(line), f)) {
            char *pos = strchr(line, '\n');
            if (pos) {
                *pos = '\0';
            }
            ESP_LOGI(TAG, "Read from file: '%s'", line);
            if(publishMQTT(line)){
                messageQueue--;
            } else{
                writeSD(line, "/sdcard/failed.txt");
            }
        }
        fclose(f);
        deleteFileSD("/sdcard/data.txt");
        if (messageQueue > 0){
            rename("/sdcard/failed.txt", "/sdcard/data.txt");
        }   
    }

    /**
    * @brief Unmount SD card
    * 
    */
    void unmountSD(){
        // All done, unmount partition and disable SPI peripheral
        esp_vfs_fat_sdcard_unmount("/sdcard", card);
        ESP_LOGI(TAG, "Card unmounted");

        //deinitialize the bus after all devices are removed
        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
        spi_bus_free(host.slot);
    }
#endif

/**
 * @brief Set time from response to AT+CCLK?
 * 
 * @param s AT command response to extract time from
 */
void setTime(char *s){    //CCLK: "23/06/10,14:09:19-16"
    char year[3];
    strncpy(year, s + 7, 2); 
    int yearint = atoi(year) + 100; //years since 1900

    char month[3];
    strncpy(month, s + 10, 2);  
    int monthint = atoi(month) - 1; //Jan = 0

    char day[3];
    strncpy(day, s + 13, 2);  
    int dayint = atoi(day);

    char hour[3];
    strncpy(hour, s + 16, 2);  
    int hourint = atoi(hour);

    char min[3];
    strncpy(min, s + 19, 2);  
    int minint = atoi(min);

    char sec[3];
    strncpy(sec, s + 22, 2);  
    int secint = atoi(sec);

    struct tm t = {0};
    t.tm_year = yearint;
    t.tm_mon = monthint;
    t.tm_mday = dayint;
    t.tm_hour = hourint;
    t.tm_min = minint;
    t.tm_sec = secint;
    time_t timeSinceEpoch = mktime(&t);
    struct timeval tv;
    if (timeSinceEpoch > 2082758399){
        tv.tv_sec = timeSinceEpoch - 2082758399;
    } else {
        tv.tv_sec = timeSinceEpoch;
    }
    tv.tv_usec = 0;    // microseconds
    settimeofday(&tv, NULL);

    /*time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);*/
}

/**
 * @brief Receive AT command responses from modem
 * 
 */
static void rx_task(void *arg){
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    char* data = (char*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            char * token = strtok(data, "\n");

            uint8_t i = 0;
            while( token != NULL ){
                i++;
                printf("%d:%s\n", i, token); //printing each token

                char *s;
                s = strstr(token, callbackString);
                if (s != NULL){
                    if(strcmp(callbackString, "CCLK:") == 0){
                        setTime(s);
                    }
                    if(strcmp(callbackString, "AT+CGSN") != 0){
                        xEventGroupSetBits(event_group, PROCEED_BIT);
                    }
                }
                if(strcmp(callbackString, "AT+CGSN") == 0 && strlen(token) == 16){
                    strcpy(imei, token);
                    xEventGroupSetBits(event_group, PROCEED_BIT);
                }
                s = strstr(token, "ERROR");
                if (s != NULL){
                    //xEventGroupSetBits(event_group, RESPONSE_BIT);
                }


                token = strtok(NULL, "\n");
            }
        }
    }
    free(data);
}

/**
 * @brief Measures temperature, humidity, and dew point from a SHT3x sensor
 * 
 */
void measureTemp(){
    uint8_t readings = 300 / MEASURE_INTERVAL_SEC;

    uint8_t i = 0;
    while(1){
        i++;
        printf("%d: Measuring temp\n", i);

        cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (0x44 << 1) | 0, true));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x2400 >> 8, true));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x2400 & 0xFF, true));
        ESP_ERROR_CHECK(i2c_master_stop(cmd));
        if(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK){
            tempi[readings+1] = -9999;
            humidityi[readings+1] = -9999;
            dewi[readings+1] = -9999;
            xEventGroupSetBits(event_group, MEASURE_BIT);
            ESP_LOGE(TAG, "Failed to initialize SHT35");
            vTaskDelete(NULL);
        };
        i2c_cmd_link_delete(cmd);

        vTaskDelay(30 / portTICK_PERIOD_MS); //wait 30ms for measurement

        uint8_t buffer[6];

        cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (0x44 << 1) | 1, true));
        ESP_ERROR_CHECK(i2c_master_read(cmd, buffer, 6, I2C_MASTER_LAST_NACK));
        ESP_ERROR_CHECK(i2c_master_stop(cmd));
        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
        i2c_cmd_link_delete(cmd);

        uint16_t rawHumidity = (buffer[3] << 8) + buffer[4];
        uint16_t rawTemperature = (buffer[0] << 8) + buffer[1];

        tempi[readings+1] = rawTemperature * (175.0 / 65535) - 45; 
        humidityi[readings+1] = rawHumidity * (100.0 / 65535);
        dewi[readings+1] = 1.8 * 243.04*(log(humidityi[readings+1]/100)+((17.625*tempi[readings+1])/(243.04+tempi[readings+1])))/(17.625-log(humidityi[readings+1]/100)-((17.625*tempi[readings+1])/(243.04+tempi[readings+1]))) + 32;
        tempi[readings+1] = 1.8 * tempi[readings+1] + 32;

        #if !AVERAGING
            temp = tempi[readings+1];
            humidity = humidityi[readings+1];
            dew = dewi[readings+1];

            if (temp > -9999){
                if (temp > maxTemp){
                    maxTemp = temp;
                }
                if (temp < minTemp){
                    minTemp = temp;
                }

                if (dew > maxDew){
                    maxDew = dew;
                }
                if (dew < minDew){
                    minDew = dew;
                }

                if (humidity > maxHumidity){
                    maxHumidity = humidity;
                }
                if (humidity < minHumidity){
                    minHumidity = humidity;
                }
            }
        #endif

        xEventGroupSetBits(event_group, MEASURE_BIT);

        ESP_LOGI(TAG, "Current Temp: %.1f", tempi[readings+1]);
        ESP_LOGI(TAG, "Current Dew Point: %.1f", dewi[readings+1]);
        ESP_LOGI(TAG, "Max Temp: %.1f, Min Temp: %.1f", maxTemp, minTemp);

        vTaskDelay(1000 * MEASURE_INTERVAL_SEC / portTICK_PERIOD_MS);
    }
}

#if AVERAGING
    /**
    * @brief Calculate 5-min average temperature, humidity, and dew point
    * 
    */
    static void calculateTemp(){
        uint8_t readings = 300 / MEASURE_INTERVAL_SEC;

        float tempSum = 0;
        float humiditySum = 0;
        float dewSum = 0;

        uint8_t numberValid = 0;

        for (uint8_t i = 1; i <= readings; i++){ 
            tempi[i] = tempi[i+1]; //Reindex values
            humidityi[i] = humidityi[i+1];
            dewi[i] = dewi[i+1];
            printf("Temp [%d]: %.1f\n", i, tempi[i]);

            if (tempi[i] > -49){ //Only sum good values
                tempSum = tempSum + tempi[i];
                humiditySum = humiditySum + humidityi[i];
                dewSum = dewSum + dewi[i];
                numberValid++; //Count number of valid measurements
            }
        }

        //printf("Number valid: %d\n", numberValid);

        if (numberValid >= 240/MEASURE_INTERVAL_SEC){ //Calculate averages and extremes if there are at least 4 minutes of valid measurements
            temp = tempSum/numberValid;
            humidity = humiditySum/numberValid;
            dew = dewSum/numberValid;
        }

        if (temp > -9999){
            if (temp > maxTemp){
                maxTemp = temp;
            }
            if (temp < minTemp){
                minTemp = temp;
            }

            if (dew > maxDew){
                maxDew = dew;
            }
            if (dew < minDew){
                minDew = dew;
            }

            if (humidity > maxHumidity){
                maxHumidity = humidity;
            }
            if (humidity < minHumidity){
                minHumidity = humidity;
            }
        }

        printf("Temp a: %.1f\n", temp);
        printf("Dew Point a: %.1f\n", dew);
    }
#endif

#if MEASURE_PRESSURE
    /**
    * @brief Measure pressure from BMP280 sensor
    * 
     */
    void measurePressure(void){
        bmx280_t* bmx280 = bmx280_create(I2C_MASTER_NUM);

        if (!bmx280) { 
            ESP_LOGE(TAG, "Could not create bmx280 driver.");
            return;
        }

        esp_err_t ret;
        
        ret = bmx280_init(bmx280);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize BMP280.");
            return;
        }

        //bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
        bmx280_config_t bmx_cfg = {BMX280_TEMPERATURE_OVERSAMPLING_X1, BMX280_PRESSURE_OVERSAMPLING_X1, BMX280_DEFAULT_STANDBY, BMX280_IIR_NONE};
        ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));

        ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
        do {
            vTaskDelay(pdMS_TO_TICKS(1));
        } while(bmx280_isSampling(bmx280));

        float temp = 0, hum = 0;
        ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pressure, &hum));

        pressure = pressure/3386.39;

        ESP_LOGI(TAG, "Read Values: temp = %f, pres = %f", temp, pressure);
    }          
#endif

/**
 * @brief Format message for transmission via MQTT
 * 
 * @param msg_buf Time buffer
 * @param transmit_Current Whether to transmit current values
 * @param transmit_MaxMin Whether to transmit max/min values
 */
char* format_data(char *msg_buf, bool transmit_Current, bool transmit_MaxMin){
    char data_buf[32];

    if (transmit_Current){
        if (temp > -9999){
            sprintf(data_buf, "%.1f", temp);
            strncat(msg_buf, ",T:", 4);
            strncat(msg_buf, data_buf, strlen(data_buf));
        }

        if (dew > -9999){
            sprintf(data_buf, "%.1f", dew);
            strncat(msg_buf, ",D:", 4);
            strncat(msg_buf, data_buf, strlen(data_buf));
        }

        if (humidity > -9999){
            sprintf(data_buf, "%.0f", humidity);
            strncat(msg_buf, ",H:", 4);
            strncat(msg_buf, data_buf, strlen(data_buf));
        }  

        #if MEASURE_PRESSURE
            if (pressure > 0){
                sprintf(data_buf, "%.2f", pressure);
                strncat(msg_buf, ",p:", 4);
                strncat(msg_buf, data_buf, strlen(data_buf));
            }
        #endif

        if (vbat > 0){
            sprintf(data_buf, "%.2f", vbat);
            strncat(msg_buf, ",Vbat:", 7);
            strncat(msg_buf, data_buf, strlen(data_buf));
        }
    }

    if (transmit_MaxMin){
        if (maxTemp > -9999){
            sprintf(data_buf, "%.1f", maxTemp);
            strncat(msg_buf, ",Tx:", 5);
            strncat(msg_buf, data_buf, strlen(data_buf));
            maxTemp = -9999;
        }

        if (minTemp < 9999){
            sprintf(data_buf, "%.1f", minTemp);
            strncat(msg_buf, ",Tn:", 5);
            strncat(msg_buf, data_buf, strlen(data_buf));
            minTemp = 9999;
        }

        if (maxDew > -9999){
            sprintf(data_buf, "%.1f", maxDew);
            strncat(msg_buf, ",Dx:", 5);
            strncat(msg_buf, data_buf, strlen(data_buf));
            maxDew = -9999;
        }

        if (minDew < 9999){
            sprintf(data_buf, "%.1f", minDew);
            strncat(msg_buf, ",Dn:", 5);
            strncat(msg_buf, data_buf, strlen(data_buf));
            minDew = 9999;
        }

        if (maxHumidity > -9999){
            sprintf(data_buf, "%.0f", maxHumidity);
            strncat(msg_buf, ",Hx:", 5);
            strncat(msg_buf, data_buf, strlen(data_buf));
            maxHumidity = -9999;
        }

        if (minHumidity < 9999){
            sprintf(data_buf, "%.0f", minHumidity);
            strncat(msg_buf, ",Hn:", 5);
            strncat(msg_buf, data_buf, strlen(data_buf));
            minHumidity = 9999;
        }
    }

    printf("Msg buff: %s\n", msg_buf);
    printf("Length of msg buff: %d\n", strlen(msg_buf));
    return msg_buf;
}

/**
 * @brief Power on modem
 * 
 */
void modem_power_on(){
    printf("Power on\n");
    gpio_set_direction(MODEM_PWRKEY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MODEM_PWRKEY_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(MODEM_PWRKEY_PIN, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(MODEM_PWRKEY_PIN, 0);

    #if MODEM == 1
        gpio_set_direction(FLIGHT_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(FLIGHT_PIN, 1);
    #endif
}

/**
 * @brief Shut down modem
 * 
 */
void modem_power_off(){
    printf("Power off\n");
    #if MODEM == 1
        gpio_set_direction(FLIGHT_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(FLIGHT_PIN, 0);
    #endif
    /*gpio_set_direction(MODEM_PWRKEY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MODEM_PWRKEY_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(MODEM_PWRKEY_PIN, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    gpio_set_level(MODEM_PWRKEY_PIN, 0);*/
    sendAT("AT+CPOF\r", "OK", 1);
    #if MODEM == 0
        gpio_pullup_dis(BAT_EN);
    #endif
}

#if MODEM == 0
void modem_reset() {
    gpio_set_direction(RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(RESET, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(RESET, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(RESET, 0);
}
#endif

void modem_wake(){
    gpio_set_direction(MODEM_DTR_PIN, GPIO_MODE_OUTPUT);
    gpio_hold_dis(MODEM_DTR_PIN);
    gpio_set_level(MODEM_DTR_PIN, 0);
    //gpio_pullup_dis(MODEM_DTR_PIN);
}

void modem_sleep(){
    gpio_set_direction(MODEM_DTR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MODEM_DTR_PIN, 1);
    gpio_hold_en(MODEM_DTR_PIN);
    gpio_deep_sleep_hold_en();
    //gpio_pullup_en(MODEM_DTR_PIN);
}

/**
 * @brief Start modem, set APN, get IMEI, get time, and shut down modem
 * 
 */
void modem_init(){
    printf("Powering on modem\n");
    #if MODEM == 0
        WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
        gpio_pullup_en(BAT_EN); //Enable battery power for modem
    #endif

    xEventGroupClearBits(event_group, PROCEED_BIT); //Clear AT response signal

    uart_init(); //Initialize UART
    xTaskCreate(rx_task, "uart_rx_task", RX_BUF_SIZE * 2, NULL, configMAX_PRIORITIES, NULL); //Start AT response receiver task
    callbackString = "PB DONE"; //Set modem ready signal

    #if MODEM == 0
        modem_reset(); //Reset modem
        //modem_power_on();
    #elif MODEM == 1
        modem_power_on();
    #endif

    xEventGroupWaitBits(event_group, PROCEED_BIT, pdTRUE, pdTRUE, 25000 / portTICK_PERIOD_MS); //Wait for PB DONE

    sendAT("AT+CGDCONT=1,\"IP\",\"hologram\",\"0.0.0.0\",0,0\r", "OK", 1); //Set APN
    sendAT("AT+CGSN\r", "AT+CGSN", 1); //Get IMEI
    //sendAT("AT+CGATT=1\r", "OK", 5); //Enable GPRS

    #if MODEM_SLEEP
        sendAT("AT+CSCLK=1\r", "OK", 1);
    #endif

    #if MODEM == 0
        sendAT("AT+CGATT=0\r", "+CGEV: ME DETACH", 5); //Disable GPRS
    #else
        sendAT("AT+CGATT=0\r", "OK", 5); //Disable GPRS
    #endif

    sendAT("AT+CCLK?\r", "CCLK:", 1); //Get time

    #if MODEM_SLEEP
        modem_sleep();
    #else
        modem_power_off(); //Shut down modem
    #endif
}

/**
 * @brief Start modem and transmit message via MQTT
 * 
 * @param msg_buf Message buffer
 */
void transmit_data(char *msg_buf){
    #if MODEM == 0
        WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
        gpio_pullup_en(BAT_EN); //Enable battery power for modem
    #endif

    xEventGroupClearBits(event_group, PROCEED_BIT); //Clear AT response signal

    uart_init(); //Initialize UART
    xTaskCreate(rx_task, "uart_rx_task", RX_BUF_SIZE * 2, NULL, configMAX_PRIORITIES, NULL); //Start AT response receiver task
    callbackString = "PB DONE"; //Set modem ready signal

    #if MODEM_SLEEP
        modem_wake();
    #else
        #if MODEM == 0
            modem_reset(); //Reset modem
            //modem_power_on();
        #elif MODEM == 1
            modem_power_on();
        #endif
    #endif

    xEventGroupWaitBits(event_group, PROCEED_BIT, pdTRUE, pdTRUE, 35000 / portTICK_PERIOD_MS); //Wait for PB DONE

    sendAT("AT\r", "OK", 5);

    //sendAT("AT+CGDCONT=1,\"IP\",\"hologram\",\"0.0.0.0\",0,0\r", "OK", 1); //Set APN

    //sendAT("AT+CCLK?\r", "CCLK:", 5); //Get time

    //sendAT("AT+CGATT=1\r", "OK", 5); //Enable GPRS

    if(connectMQTT()){ //Connect to MQTT
        if(!publishMQTT(msg_buf)){ //Publish message. If failed, write failed message to SD card and increment failed message counter.
            #if STORE_SD
                mountSD();
                writeSD(msg_buf, "/sdcard/data.txt");
                unmountSD();
                messageQueue++;
            #endif
        } else{ //If success, publish any queued messages.
            #if STORE_SD
                if(messageQueue > 0){
                    mountSD();
                    readSD("/sdcard/data.txt");
                    unmountSD();
                }
            #endif
        }
    } else{ //If fail to connect, write failed message to SD card and increment queued message counter.
        #if STORE_SD
            mountSD();
            writeSD(msg_buf, "/sdcard/data.txt");
            unmountSD();
            messageQueue++;
        #endif
    }

    #if MODEM == 0
        sendAT("AT+CGATT=0\r", "+CGEV: ME DETACH", 5); //Disable GPRS
    #else
        sendAT("AT+CGATT=0\r", "OK", 5); //Disable GPRS
    #endif

    sendAT("AT+CCLK?\r", "CCLK:", 1); //Get time

    #if MODEM_SLEEP
        modem_sleep();
    #else
        modem_power_off(); //Shut down modem
    #endif
}

void getVbat(){
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = BAT_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };

    int adc_raw[10];

    int adc_sum = 0;

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BAT_ADC, &config));

    for (int i = 0; i <= 9; i++){
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BAT_ADC, &adc_raw[i]));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", BAT_ADC_UNIT, BAT_ADC, adc_raw[i]);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    adc_sum = adc_sum + adc_raw[i];
    }
    
    vbat = ((float)adc_sum / 40950.0) * 2.0 * 3.3 * (1100 / 1000.0);
}

void app_main(void){
    bootCount++; //for precip

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause(); //Get wakeup cause

    event_group = xEventGroupCreate(); //Create event group

    i2c_master_init(); //Initialize I2C

    xTaskCreate(measureTemp, "measureTemp", 1024*2, NULL, configMAX_PRIORITIES, NULL); //Start temp measuring task
    xEventGroupWaitBits(event_group, MEASURE_BIT, pdTRUE, pdTRUE, 500 / portTICK_PERIOD_MS); //Wait for temp measurement

    //measurePressure();

    #if AVERAGING
        calculateTemp(); //Calculate average temp
    #endif

    if (cause != ESP_SLEEP_WAKEUP_TIMER){ //Do the following on initial boot
        printf("Initial boot, initializing ULP\n");
        modem_init(); //Initialize modem
    }

    time_t now;
    char time_buf[64];
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M", &timeinfo);
    ESP_LOGI(TAG, "The current date/time is: %s", time_buf);

    int currentHour = timeinfo.tm_hour;
    int currentMin = timeinfo.tm_min;
    int currentSec = timeinfo.tm_sec;

    ESP_LOGI(TAG, "Current min: %d", currentMin);
    ESP_LOGI(TAG, "Last format min: %d", lastFormatMin);

    if (((currentMin % STORE_INTERVAL_MIN == 0) || (currentHour == 23 && currentMin == 59 && currentSec >= (59 - MEASURE_INTERVAL_SEC))) && currentMin != lastFormatMin){
        #if MEASURE_PRESSURE
            measurePressure(); //Measure pressure
        #endif
        getVbat(); //Get battery voltage
        char* msg_buf = format_data(time_buf, true, true);
        if ((currentMin % TRANSMIT_INTERVAL_MIN == 0)){
            transmit_data(msg_buf);
        } else{
            #if STORE_SD
                mountSD();
                writeSD(msg_buf, "/sdcard/data.txt");
                unmountSD();
                messageQueue++;
            #endif
        }
        lastFormatMin = currentMin;
    }

    uint64_t timeToSleep = (MEASURE_INTERVAL_SEC * 1000000) - (esp_timer_get_time() % (MEASURE_INTERVAL_SEC * 1000000)); //Measurement interval minus time elapsed since boot
    esp_sleep_enable_timer_wakeup(timeToSleep); 
    esp_deep_sleep_start();
}