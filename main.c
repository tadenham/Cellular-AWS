#include <stdio.h> //C input/output library
#include <inttypes.h> //Integer types library
#include <math.h> //Math library
#include <string.h> //String library
#include <time.h> //Time library
#include "nvs.h" //Non-volatile storage
#include "nvs_flash.h" //Non-volatile storage
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "ulp.h" //ULP
#include "ulp_main.h" //ULP
#include <sys/time.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#define MEASURE_INTERVAL_SEC 10
#define STORE_INTERVAL_MIN 15
#define TRANSMIT_INTERVAL_MIN 15

#define TRANSMIT_DATA true
#define PROTOCOL 1 //0 = MQTT, 1 = UDP
#define STORE_SD true

#if TRANSMIT_DATA
    #include "driver/uart.h" //UART
#endif

#if STORE_SD
    #include "esp_vfs_fat.h" //SD card
    #include "sdmmc_cmd.h" //SD card
    #define PIN_NUM_MISO 2
    #define PIN_NUM_MOSI 15
    #define PIN_NUM_CLK 14
    #define PIN_NUM_CS 13
#endif

#define AVERAGING true
#define MEASURE_TEMP true
#define MEASURE_PRESSURE true
#define MEASURE_WIND false

#if MEASURE_PRESSURE
    #include "bmx280.h" //BMP280 pressure sensor
#endif

//I2C definitions
#define I2C_MASTER_NUM 0
#define I2C_MASTER_TIMEOUT_MS 1000
#define I2C_CLK_SPEED 100000 //100 kHz
#define SDA_PIN 21
#define SCL_PIN 22

//Pin definitions
#define RAIN_PIN GPIO_NUM_39
#define WIND_PIN GPIO_NUM_13
#define BAT_ADC_UNIT ADC_UNIT_1
#define BAT_ADC ADC_CHANNEL_7

#define MODEM 2 //MODEM 1 = SIM7600, MODEM 2 = SIM7070
#define MODEM_SLEEP 0 //0 = Power off, 1 = Modem sleep

#if MODEM == 1 //SIM7600
    #define MODEM_PWRKEY_PIN GPIO_NUM_4
    #define TXD_PIN 27
    #define RXD_PIN 26
    #define MODEM_DTR_PIN GPIO_NUM_32
    #define FLIGHT_PIN GPIO_NUM_25
#elif MODEM == 2 //SIM7070
    #define MODEM_PWRKEY_PIN GPIO_NUM_4
    #define TXD_PIN 27
    #define RXD_PIN 26
    #define MODEM_DTR_PIN GPIO_NUM_32
#endif

#define UART_NUM UART_NUM_1
#define UART_BAUD 115200
#define RX_BUF_SIZE 1024

#define PROCEED_BIT BIT0
#define MEASURE_BIT BIT1
#define PRECIP_BIT BIT2
#define WIND_BIT BIT3
#define RESPONSE_BIT BIT4

//ULP functions
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

RTC_DATA_ATTR int lastFormatMin = -1;

char* msg_buf;
int msg_len_bytes;

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
RTC_DATA_ATTR float precip = 0;
float precipRate;
RTC_DATA_ATTR float maxPrecipRate = 0;
RTC_DATA_ATTR float pulse_time_last_sec;
RTC_DATA_ATTR uint32_t boot_count;

#if MEASURE_PRESSURE
    float pressure;
#endif

#if MEASURE_WIND
    #define SCALE_FACTOR 3
    float windSpeed = -9999; //2-minute average sustained wind
    RTC_DATA_ATTR int32_t windPulses[] = {-9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999}; //2 minutes of instantaneous values
    RTC_DATA_ATTR float maxWindSpeed = -9999; //max 2-minute average since last transmission
    
    float windGust = -9999;
    RTC_DATA_ATTR float windGusti[] = {-9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999}; //2 minutes of max values
    RTC_DATA_ATTR float maxWindGust = -9999; //Max gust since last transmission
    
    float windDirection = -9999; //2-minute average wind direction
    RTC_DATA_ATTR float windDirectioni[] = {-9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999}; //2 minutes of instantaneous values
    uint16_t ulp_wakeup_period_us = 1000; //Wakeup period of 1 ms
#else
    uint16_t ulp_wakeup_period_us = 10000; //Wakeup period of 10 ms
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

#if TRANSMIT_DATA
    void uart_init(){
        const uart_config_t uart_config = {
            .baud_rate = UART_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
        uart_param_config(UART_NUM, &uart_config);
        uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }

    /**
    * @brief Send AT command to modem and waits for specified response
    * 
    * @param command Command to send. Must be terminated by \r.
    * @param callback Modem response to wait for
    * @param timeToWait Seconds to wait for response before giving up
    */
    bool sendAT(const char* command, char* callback, uint8_t timeToWait){
        const int len = strlen(command);
        callbackString = callback;
        uart_write_bytes(UART_NUM, command, len);
        EventBits_t uxBits;
        uxBits = xEventGroupWaitBits(event_group, PROCEED_BIT, pdTRUE, pdTRUE, timeToWait*1000 / portTICK_PERIOD_MS);
        if((uxBits & PROCEED_BIT) != 0){ //Received correct response
            ESP_LOGI(TAG, "AT command successful");
            return true;
        } else{ //Command timeout
            ESP_LOGE(TAG, "AT command timeout");
            return false;
        };
    }

    /**
    * @brief Connects to MQTT broker
    * 
    */
    bool connectMQTT(){
        #if MODEM != 2
            char client[64] = "AT+CMQTTACCQ=0,\"";
            strncat(client, imei, strlen(imei));
            strncpy(client + 31, "\"", 2);
            strncat(client, "\r", 2);
            printf("Client: %s\n", client);

            sendAT("AT+CMQTTSTART\r", "+CMQTTSTART: 0", 5);
            sendAT(client, "OK", 5);
            if(sendAT("AT+CMQTTCONNECT=0,\"tcp://142.11.236.169:1883\",60,0\r", "+CMQTTCONNECT: 0,0", 60)){
                ESP_LOGI(TAG, "MQTT connect successful");
                return true;
            } else{
                ESP_LOGE(TAG, "MQTT connect fail");
                return false;
            }
        #else
            sendAT("AT+CNACT=1,1\r", "OK", 5);

            char client[64] = "AT+SMCONF=\"CLIENTID\",\"";
            strncat(client, imei, strlen(imei));
            printf("Str len: %d\n", strlen(client));
            strncpy(client + 37, "\"", 2); //need to calc
            strncat(client, "\r", 2);
            printf("Client: %s\n", client);

            sendAT("AT+SMCONF=\"URL\",\"142.11.236.169\",\"1883\"\r", "OK", 5);
            sendAT(client, "OK", 5);
            if(sendAT("AT+SMCONN\r", "OK", 20)){
                ESP_LOGI(TAG, "MQTT connect successful");
                return true;
            } else{
                ESP_LOGE(TAG, "MQTT connect fail");
                return false;
            }
        #endif
    }   

    /**
    * @brief Publishes MQTT message. connectMQTT must have already been called.
    * 
    * @param msg_buf Message buffer
    */
    bool publishMQTT(char *msg_buf){
        #if MODEM != 2
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
            if(sendAT("AT+CMQTTPUB=0,1,60,1\r", "+CMQTTPUB: 0,0", 15)){
                ESP_LOGI(TAG, "MQTT publish successful");
                return true;
            } else{
                ESP_LOGE(TAG, "MQTT publish fail");
                return false;
            }
        #else
            char topic[21] = "data/";
            strncat(topic, imei, strlen(imei));

            char pub_buf[64] =  "AT+SMPUB=\"";
            strncat(pub_buf, topic, strlen(topic));
            strncpy(pub_buf + 30, "\"", 2);
            strncat(pub_buf, ",", 2);

            char msg_len[32];
            sprintf(msg_len, "%d", strlen(msg_buf));
            strncat(pub_buf, msg_len, strlen(msg_len));
            strncat(pub_buf, ",1,1\r", 6);

            printf("Msg buf: %s\n", msg_buf);
            sendAT(pub_buf, ">", 5);
            if(sendAT(msg_buf, "OK", 5)){
                ESP_LOGI(TAG, "MQTT publish successful");
                return true;
            } else{
                ESP_LOGE(TAG, "MQTT publish fail");
                return false;
            }
        #endif
    }

    bool connectUDP(){
        #if MODEM == 1
            sendAT("AT+NETOPEN\r", "OK", 5);

            if(sendAT("AT+CIPOPEN=1,\"UDP\",,,5000\r", "+CIPOPEN: 1,0", 5)){
                ESP_LOGI(TAG, "UDP connect successful");
                return true;
            } else{
                ESP_LOGE(TAG, "UDP connnect fail");
                return false;
            }
        #else
            if(sendAT("AT+CNACT=0,1\r", "+APP PDP: 0,ACTIVE", 10)){
                if(sendAT("AT+CAOPEN=1,0,\"UDP\",\"142.11.236.169\",5254,1\r", "+CAOPEN: 1,0", 5)){
                    ESP_LOGI(TAG, "UDP connnect successful");
                return true;
                } else{
                    ESP_LOGE(TAG, "UDP connnect fail");
                return false;
                }
            } else {
                ESP_LOGE(TAG, "UDP connnect fail");
                return false;
            }
        #endif    
    }

    bool publishUDP(char *msg_buf){
        #if MODEM == 1
            char pub_buf[32] =  "AT+CIPSEND=1,";
            char msg_len[32];
            sprintf(msg_len, "%d", strlen(msg_buf));
            strncat(pub_buf, msg_len, strlen(msg_len));
            strncat(pub_buf, ",\"142.11.236.169\",5254\r", strlen(",\"142.11.236.169\",5254\r")+1);
            printf("Pub buf: %s\n", pub_buf);

            sendAT(pub_buf, ">", 5);
            if(sendAT(msg_buf, "RECV FROM:142.11.236.169", 10)){
                ESP_LOGI(TAG, "UDP publish successful");
                return true;
            } else{
                ESP_LOGE(TAG, "UDP publish fail");
                return false;
            }
        #else
            char pub_buf[32] =  "AT+CASEND=1,";
            msg_len_bytes = strlen(msg_buf);
            char msg_len[32];
            sprintf(msg_len, "%d", msg_len_bytes);
            strncat(pub_buf, msg_len, strlen(msg_len));
            strncat(pub_buf, "\r", strlen("\r")+1);
            sendAT(pub_buf, ">", 5);
            if(sendAT(msg_buf, "+CAURC: \"recv\"", 10)){
                ESP_LOGI(TAG, "UDP publish successful");
                return true;
            } else{
                ESP_LOGE(TAG, "UDP publish fail");
                return false;
            }
        #endif
    }   
#endif

#if STORE_SD
    /**
    * @brief Mount SD card
    * 
    */                      
    bool mountSD(void){
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
            return false;
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
            return false;
        }
        ESP_LOGI(TAG, "Filesystem mounted");

        // Card has been initialized, print its properties
        sdmmc_card_print_info(stdout, card);
        return true;
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

    #if TRANSMIT_DATA
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
            char line[256];

            while (fgets(line, sizeof(line), f)) {
                char *pos = strchr(line, '\n');
                if (pos) {
                    *pos = '\0';
                }
                ESP_LOGI(TAG, "Read from file: '%s'", line);
                #if PROTOCOL == 0
                    if(publishMQTT(line)){
                        messageQueue--;
                    } else{
                        writeSD(line, "/sdcard/failed.txt");
                    }
                #else
                    if(publishUDP(line)){
                        messageQueue--;
                    } else{
                        writeSD(line, "/sdcard/failed.txt");
                    }
                #endif
            }
            fclose(f);
            deleteFileSD("/sdcard/queue.txt");
            if (messageQueue > 0){
                rename("/sdcard/failed.txt", "/sdcard/queue.txt");
            }   
        }
    #endif

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
 * @brief Set time from modem response
 * 
 * @param s AT command response to extract time from
 */
void setTime(char *s){
    //ESP_LOGE(TAG, "Time from modem: %s", s);
    //CCLK: "+CCLK: "23/06/10,14:09:19-16"
    char year[3];
    strncpy(year, s + 7, 2); 
    int yearint = atoi(year) + 100; //years since 1900

    char month[3];
    strncpy(month, s + 10, 2);  
    int monthint = atoi(month) - 1; //Jan = 0

    char day[3];
    strncpy(day, s + 13, 3);  
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
}

#if TRANSMIT_DATA
    /**
     * @brief Receive AT command responses from modem
     * 
     */
    static void rx_task(void *arg){
        static const char *RX_TASK_TAG = "RX_TASK";
        esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
        char* data = (char*) malloc(RX_BUF_SIZE+1);
        while (1) {
            const int rxBytes = uart_read_bytes(UART_NUM, data, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
            if (rxBytes > 0) {
                data[rxBytes] = 0;
                char * token = strtok(data, "\n");

                uint8_t i = 0;
                while( token != NULL ){
                    i++;
                    ESP_LOGI("Modem", "%d %s", i, token);

                    char *s;
                    s = strstr(token, callbackString);
                    if (s != NULL){
                            if(strcmp(callbackString, "CCLK:") == 0){
                                setTime(s);
                            }
                            if(strcmp(callbackString, "AT+CGSN") != 0 && strcmp(callbackString, "+CAURC: \"recv\"") != 0){
                                xEventGroupSetBits(event_group, PROCEED_BIT);
                            }
                    }
                    if(strcmp(callbackString, "AT+CGSN") == 0 && strlen(token) == 16){
                        strcpy(imei, token);
                        xEventGroupSetBits(event_group, PROCEED_BIT);
                    }
                    if(strcmp(callbackString, "+CAURC: \"recv\"") == 0 && i == 3){
                        ESP_LOGI(TAG, "Length received by server: %s", token);
                        ESP_LOGI(TAG, "Length sent: %d", msg_len_bytes);
                        if(msg_len_bytes == atoi(token)){
                            xEventGroupSetBits(event_group, PROCEED_BIT);
                        }
                    }
                    /*s = strstr(token, "ERROR");
                    if (s != NULL){
                        xEventGroupSetBits(event_group, RESPONSE_BIT);
                    }
                    s = strstr(token, "NORMAL POWER DOWN");
                    if (s != NULL && strcmp(callbackString, "PSUTTZ") == 0){
                        ESP_LOGI(TAG, "Premature power down");
                        vTaskDelay(5000 * MEASURE_INTERVAL_SEC / portTICK_PERIOD_MS);
                        esp_restart();
                    }*/
                    token = strtok(NULL, "\n");
                }
            }
        }
        free(data);
    }
#endif

static void init_ulp_program(void) {
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* GPIO used for pulse counting. */
    int rtcio_num_rain = rtc_io_number_get(RAIN_PIN);
    assert(rtc_gpio_is_valid_gpio(RAIN_PIN) && "GPIO used for rain must be an RTC IO");

    int rtcio_num_wind = rtc_io_number_get(WIND_PIN);
    assert(rtc_gpio_is_valid_gpio(WIND_PIN) && "GPIO used for wind must be an RTC IO");

    /* Initialize some variables used by ULP program.
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * These variables are declared in an auto generated header file,
     * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
     * These variables are located in RTC_SLOW_MEM and can be accessed both by the
     * ULP and the main CPUs.
     *
     * Note that the ULP reads only the lower 16 bits of these variables.
     */
    ulp_debounce_counter_rain = 0;
    ulp_debounce_counter_wind = 0;
    ulp_debounce_max_count_rain = 0;
    ulp_debounce_max_count_wind = 0;
    ulp_next_edge_rain = 1;
    ulp_next_edge_wind = 1;
    ulp_io_number_rain = rtcio_num_rain; /* map from GPIO# to RTC_IO# */
    ulp_io_number_wind = rtcio_num_wind; /* map from GPIO# to RTC_IO# */
    //ulp_edge_count_to_wake_up = 10;

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    if(rainGauge){
        rtc_gpio_init(RAIN_PIN);
        rtc_gpio_set_direction(RAIN_PIN, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pulldown_dis(RAIN_PIN);
        rtc_gpio_pullup_dis(RAIN_PIN);
        rtc_gpio_hold_en(RAIN_PIN);
    }
    #if MEASURE_WIND
        rtc_gpio_init(WIND_PIN);
        rtc_gpio_set_direction(WIND_PIN, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pulldown_dis(WIND_PIN);
        rtc_gpio_pullup_dis(WIND_PIN);
        rtc_gpio_hold_en(WIND_PIN);
    #endif

    #if CONFIG_IDF_TARGET_ESP32
        //rtc_gpio_isolate(GPIO_NUM_12);
        //rtc_gpio_isolate(GPIO_NUM_15);
    #endif

    esp_deep_sleep_disable_rom_logging(); // suppress boot messages

    /* Set ULP wake up period
     * Minimum pulse width has to be T * (ulp_debounce_counter + 1)
     */
    ulp_set_wakeup_period(0, ulp_wakeup_period_us);

    /* Start the program */
    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

static void measureRain() {
    while(1){
        boot_count++;

        /* ULP program counts signal edges, convert that to the number of pulses */
        uint32_t pulse_count_from_ulp = (ulp_edge_count_rain & UINT16_MAX) / 2;
        /* In case of an odd number of edges, keep one until next time */
        ulp_edge_count_rain = ulp_edge_count_rain % 2;
        //printf("New pulses: %5"PRIu32"\n", pulse_count_from_ulp);

        if (pulse_count_from_ulp){
            if(boot_count >= (300/MEASURE_INTERVAL_SEC)){ //if over 5 minutes since last pulse, use boot count since last pulse to calculate rain rate
                pulse_time_last_sec = fmin(boot_count*MEASURE_INTERVAL_SEC,3600); //limit to one hour
            } else{ //if less than 5 minutes since last pulse, use ULP timer to calculate rain rate
                pulse_time_last_sec = (float)((ulp_pulse_last_rain & UINT16_MAX) * ulp_wakeup_period_us)/1000000; 
            }
            boot_count = 0;
            precip = precip + ((float)pulse_count_from_ulp)/100;
        }

        if (boot_count < fmax(300/MEASURE_INTERVAL_SEC, pulse_time_last_sec/MEASURE_INTERVAL_SEC) && pulse_time_last_sec){
            precipRate = 36/pulse_time_last_sec;
        } else{
            precipRate = 0;
        }

        if(precipRate > maxPrecipRate){
            maxPrecipRate = precipRate;
        }

        /* Reset shortest edge */
        ulp_pulse_min_rain = 0;

        ESP_LOGI(TAG, "Precip Accum: %.2f", precip);
        ESP_LOGI(TAG, "Max rain rate: %.2f", maxPrecipRate);
        ESP_LOGI(TAG, "Current rain rate: %.2f", precipRate);

        xEventGroupSetBits(event_group, PRECIP_BIT);

        vTaskDelay(1000 * MEASURE_INTERVAL_SEC / portTICK_PERIOD_MS);   
    }
}

#if MEASURE_TEMP
    /**
     * @brief Measures temperature, humidity, and dew point from a SHT3x sensor, calculates 5-minute average if configured, and compares to max/min values
     * 
     */
    void measureTemp() {
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

            #if AVERAGING
                float tempSum = 0;
                float humiditySum = 0;
                float dewSum = 0;

                uint8_t numberValid = 0;

                for (uint8_t i = 1; i <= readings; i++){ 
                    tempi[i] = tempi[i+1]; //Reindex values
                    humidityi[i] = humidityi[i+1];
                    dewi[i] = dewi[i+1];

                    if (tempi[i] > -49){ //Only sum good values
                        tempSum = tempSum + tempi[i];
                        humiditySum = humiditySum + humidityi[i];
                        dewSum = dewSum + dewi[i];
                        numberValid++; //Count number of valid measurements
                    }
                }

                if (numberValid >= 240/MEASURE_INTERVAL_SEC){ //Calculate averages and extremes if there are at least 4 minutes of valid measurements
                    temp = tempSum/numberValid;
                    humidity = humiditySum/numberValid;
                    dew = dewSum/numberValid;
                }
            #else
                temp = tempi[readings+1];
                humidity = humidityi[readings+1];
                dew = dewi[readings+1];
            #endif

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

            xEventGroupSetBits(event_group, MEASURE_BIT);

            ESP_LOGI(TAG, "Current Temp: %.1f", temp);
            ESP_LOGI(TAG, "Current Dew Point: %.1f", dew);
            ESP_LOGI(TAG, "Max Temp: %.1f, Min Temp: %.1f", maxTemp, minTemp);

            vTaskDelay(1000 * MEASURE_INTERVAL_SEC / portTICK_PERIOD_MS);
        }
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

#if MEASURE_WIND
    static void measureWind(){
        uint8_t readings = 120 / MEASURE_INTERVAL_SEC;
        while(1){
            cmd = i2c_cmd_link_create();
            ESP_ERROR_CHECK(i2c_master_start(cmd));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (0x48 << 1) | 0, true));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x90 & 0xFF, true));
            ESP_ERROR_CHECK(i2c_master_stop(cmd));
            if(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK){
                windDirectioni[readings+1] = -9999;
                xEventGroupSetBits(event_group, WIND_BIT);
                ESP_LOGE(TAG, "Failed to initialize wind direction sensor");
                vTaskDelete(NULL);
            };
            i2c_cmd_link_delete(cmd);

            vTaskDelay(30 / portTICK_PERIOD_MS); //wait 30ms for measurement

            uint8_t buffer[2];

            cmd = i2c_cmd_link_create();
            ESP_ERROR_CHECK(i2c_master_start(cmd));
            ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (0x48 << 1) | 1, true));
            ESP_ERROR_CHECK(i2c_master_read(cmd, buffer, 2, I2C_MASTER_LAST_NACK));
            ESP_ERROR_CHECK(i2c_master_stop(cmd));
            ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
            i2c_cmd_link_delete(cmd);

            int16_t rawValue = (buffer[0] << 8) + buffer[1];

            windDirectioni[readings+1] = (float)rawValue*360/2048;

            ///////

            uint32_t wind_pulses = (ulp_edge_count_wind & UINT16_MAX) / 2;
            /* In case of an odd number of edges, keep one until next time */
            ulp_edge_count_wind = ulp_edge_count_wind % 2;
            windPulses[readings+1] = wind_pulses;

            ESP_LOGI(TAG,"Pulses: %ld", wind_pulses);
            
            float gust;

            if(wind_pulses){
                uint32_t pulse_time_min = (ulp_pulse_min_wind & UINT16_MAX) * ulp_wakeup_period_us;
                gust = (1000000/(float)(pulse_time_min)) * SCALE_FACTOR * 60 * 60 * 4.5 * 3.14159 / (12*5280);
            } else{
                gust = 0;
            }
            if (gust > maxWindGust){ //Max since last transmission
                maxWindGust = gust;
            }
            //ESP_LOGI(TAG, "Wind Gust : %.1f%s", gust," mph");
            //float RPM = 60*(1000000/(float)(pulse_time_min));
            //ESP_LOGI(TAG, "Max RPM : %.0f", RPM);
            ulp_pulse_min_wind = 0;

            windGusti[readings+1] = gust;

            int32_t windSum = 0;
            float directionSum = 0;

            uint8_t numberValid = 0;

            float ud_normalized[30];

            for (uint8_t i = 1; i <= readings; i++){ 
                windPulses[i] = windPulses[i+1]; //Reindex values
                windGusti[i] = windGusti[i+1];
                if (windGusti[i] > windGust){ //2-minute max
                    windGust = windGusti[i];
                }
                windDirectioni[i] = windDirectioni[i+1];
                if (i == 1){
                    ud_normalized[i] = windDirectioni[i];
                } else {
                    if (abs(windDirectioni[i]-ud_normalized[i-1]) > 180) {
                        if (windDirectioni[i] < ud_normalized[i-1]){
                            ud_normalized[i] = windDirectioni[i] + 360;
                        } else {
                             ud_normalized[i] = windDirectioni[i] - 360;
                        }
                    } else {
                        ud_normalized[i] = windDirectioni[i];
                    }
                }
                ESP_LOGI(TAG, "Normalized Wind Direction [%d]: %.0f", i, ud_normalized[i]);

                if (windDirectioni[i] > -9999){ //Only sum good values
                    windSum = windSum + windPulses[i];
                    directionSum = directionSum + ud_normalized[i];
                    numberValid++; //Count number of valid measurements
                }
                //ESP_LOGI(TAG, "Wind Pulses [%d]: %ld", i, windPulses[i]);
                //ESP_LOGI(TAG, "Wind Gust [%d]: %.1f", i, windGusti[i]);
            }

            ESP_LOGI(TAG, "2-min Wind Gust: %.1f", windGust);
            ESP_LOGI(TAG, "Max Wind Gust: %.1f", maxWindGust);

            if (numberValid >= 100/MEASURE_INTERVAL_SEC){ //Calculate averages and extremes if there are at least 100 seconds of valid measurements
                windSpeed = ((float)windSum/((float)numberValid * MEASURE_INTERVAL_SEC)) * SCALE_FACTOR * 60 * 60 * 4.5 * 3.14159 / (12*5280);
                ESP_LOGI(TAG, "2-min Sustained Wind: %.1f", windSpeed);
                if (windSpeed > maxWindSpeed){
                    maxWindSpeed = windSpeed;
                }
                windDirection = (int)(directionSum/numberValid) % 360;
                if (windDirection < 0) {
                    windDirection = windDirection + 360;
                }
            }

            char* direction = "";

            if (windDirection >= 23 && windDirection <= 67){
                direction = "NE";
            } else if (windDirection >= 68 && windDirection <= 112){
                direction = "E";
            } else if (windDirection >= 113 && windDirection <= 157){
                direction = "SE";
            } else if (windDirection >= 158 && windDirection <= 202){
                direction = "S";
            } else if (windDirection >= 203 && windDirection <= 247){
                direction = "SW";
            } else if (windDirection >= 248 && windDirection <= 292){
                direction = "W";
            } else if (windDirection >= 293 && windDirection <= 337){
                direction = "NW";
            } else if (windDirection >= 338 || windDirection <= 22){
                direction = "N";
            }

            ESP_LOGI(TAG, "Raw Wind Direction: %.0f", (float)rawValue*360/2048);

            ESP_LOGI(TAG, "Wind Direction: %.0f° (%s)", windDirection, direction);

            ESP_LOGI(TAG, "Wind %s %.1f mph G %.1f mph", direction, windSpeed, windGust);

            xEventGroupSetBits(event_group, WIND_BIT);

            vTaskDelay(1000 * MEASURE_INTERVAL_SEC / portTICK_PERIOD_MS);
        }
    }   
#endif

void getVbat(){
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = BAT_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    int adc_raw[10];

    int adc_sum = 0;

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BAT_ADC, &config));

    for (int i = 0; i <= 9; i++){
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BAT_ADC, &adc_raw[i]));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", BAT_ADC_UNIT + 1, BAT_ADC, adc_raw[i]);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    adc_sum = adc_sum + adc_raw[i];
    }
    
    vbat = ((float)adc_sum / 40950.0) * 2.0 * 3.3 * (1100 / 1000.0);
}

int readADC(int gpio){
    adc_unit_t unit;
    adc_channel_t channel;
    adc_oneshot_io_to_channel(gpio, &unit, &channel);
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = unit,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    int adc_raw;

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &adc_raw));
    ESP_LOGI(TAG, "ADC Unit %d Channel[%d] Raw Data: %d", unit+1, channel, adc_raw);

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    
    return adc_raw;
}

/**
 * @brief Format message for transmission
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
            strcat(msg_buf, ",T:");
            strncat(msg_buf, data_buf, strlen(data_buf));
        }

        if (dew > -9999){
            sprintf(data_buf, "%.1f", dew);
            strcat(msg_buf, ",D:");
            strncat(msg_buf, data_buf, strlen(data_buf));
        }

        if (humidity > -9999){
            sprintf(data_buf, "%.0f", humidity);
            strcat(msg_buf, ",H:");
            strncat(msg_buf, data_buf, strlen(data_buf));
        }  

        #if MEASURE_PRESSURE
            if (pressure > 0){
                sprintf(data_buf, "%.2f", pressure);
                strncat(msg_buf, ",p:", 4);
                strncat(msg_buf, data_buf, strlen(data_buf));
            }
        #endif

        #if MEASURE_WIND
            if (windSpeed >= 0){
                sprintf(data_buf, "%.1f", windSpeed);
                strncat(msg_buf, ",u:", 4);
                strncat(msg_buf, data_buf, strlen(data_buf));
            }

            if (windGust >= 0){
                sprintf(data_buf, "%.1f", windGust);
                strncat(msg_buf, ",ug:", 5);
                strncat(msg_buf, data_buf, strlen(data_buf));
            }

            if (windDirection >= 0){
                sprintf(data_buf, "%.0f", windDirection);
                strncat(msg_buf, ",ud:", 5);
                strncat(msg_buf, data_buf, strlen(data_buf));
            }
        #endif

        if (vbat > 0){
            sprintf(data_buf, "%.2f", vbat);
            strcat(msg_buf, ",V:");
            strncat(msg_buf, data_buf, strlen(data_buf));
        }
    }

    if (transmit_MaxMin){
        if (maxTemp > -9999){
            sprintf(data_buf, "%.1f", maxTemp);
            strcat(msg_buf, ",Tx:");
            strncat(msg_buf, data_buf, strlen(data_buf));
            maxTemp = -9999;
        }

        if (minTemp < 9999){
            sprintf(data_buf, "%.1f", minTemp);
            strcat(msg_buf, ",Tn:");
            strncat(msg_buf, data_buf, strlen(data_buf));
            minTemp = 9999;
        }

        if (maxDew > -9999){
            sprintf(data_buf, "%.1f", maxDew);
            strcat(msg_buf, ",Dx:");
            strncat(msg_buf, data_buf, strlen(data_buf));
            maxDew = -9999;
        }

        if (minDew < 9999){
            sprintf(data_buf, "%.1f", minDew);
            strcat(msg_buf, ",Dn:");
            strncat(msg_buf, data_buf, strlen(data_buf));
            minDew = 9999;
        }

        if (maxHumidity > -9999){
            sprintf(data_buf, "%.0f", maxHumidity);
            strcat(msg_buf, ",Hx:");
            strncat(msg_buf, data_buf, strlen(data_buf));
            maxHumidity = -9999;
        }

        if (minHumidity < 9999){
            sprintf(data_buf, "%.0f", minHumidity);
            strcat(msg_buf, ",Hn:");
            strncat(msg_buf, data_buf, strlen(data_buf));
            minHumidity = 9999;
        }

        #if MEASURE_WIND
            if (maxWindSpeed >= 0){
                sprintf(data_buf, "%.1f", maxWindSpeed);
                strcat(msg_buf, ",ux:");
                strncat(msg_buf, data_buf, strlen(data_buf));
                maxWindSpeed = -9999;
            }

            if (maxWindGust >= 0){
                sprintf(data_buf, "%.1f", maxWindGust);
                strcat(msg_buf, ",ugx:");
                strncat(msg_buf, data_buf, strlen(data_buf));
                maxWindGust = -9999;
            }
        #endif
    }

    if (rainGauge){
        sprintf(data_buf, "%.2f", precip);
        strcat(msg_buf, ",R:");
        strncat(msg_buf, data_buf, strlen(data_buf));

        sprintf(data_buf, "%.2f", precipRate);
        strcat(msg_buf, ",rR:");
        strncat(msg_buf, data_buf, strlen(data_buf));

        sprintf(data_buf, "%.2f", maxPrecipRate);
        strcat(msg_buf, ",rRx:");
        strncat(msg_buf, data_buf, strlen(data_buf));

        maxPrecipRate = 0;
    }

    #if PROTOCOL == 1
        sprintf(data_buf, "%s", imei);
        strncat(msg_buf,",", 2);
        strncat(msg_buf, data_buf, 15);
    #endif

    ESP_LOGI(TAG, "Msg buf: %s", msg_buf);
    printf("Length of msg buff: %d\n", strlen(msg_buf));
    return msg_buf;
}

#if TRANSMIT_DATA
    /**
     * @brief Power on modem
     * 
     */
    void modem_power_on(){
        printf("Power on\n");
        //gpio_set_direction(MODEM_PWRKEY_PIN, GPIO_MODE_OUTPUT);
        //gpio_set_level(MODEM_PWRKEY_PIN, 0);
        //TaskDelay(100 / portTICK_PERIOD_MS);
        gpio_pullup_en(MODEM_PWRKEY_PIN);
        //gpio_set_level(MODEM_PWRKEY_PIN, 1);
        vTaskDelay(1500 / portTICK_PERIOD_MS);
        gpio_pullup_dis(MODEM_PWRKEY_PIN);
        //gpio_set_level(MODEM_PWRKEY_PIN, 0);

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
        #if MODEM == 2
            sendAT("AT+CPOWD=1\r", "NORMAL POWER DOWN", 1);
        #else
            sendAT("AT+CPOF\r", "OK", 1);
        #endif
    }

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

        xEventGroupClearBits(event_group, PROCEED_BIT); //Clear AT response signal

        uart_init(); //Initialize UART
        xTaskCreate(rx_task, "uart_rx_task", RX_BUF_SIZE * 2, NULL, configMAX_PRIORITIES - 1, NULL); //Start AT response receiver task
        #if MODEM == 1
            callbackString = "PB DONE"; //Set modem ready signal
        #elif MODEM == 2  
            callbackString = "PSUTTZ"; //Set modem ready signal
        #endif

        modem_power_on();

        #if MODEM_SLEEP
            modem_wake();
        #endif
        
        xEventGroupWaitBits(event_group, PROCEED_BIT, pdTRUE, pdTRUE, 30000 / portTICK_PERIOD_MS); //Wait for ready signal

        //sendAT("AT+CGDCONT=1,\"IP\",\"hologram\",\"0.0.0.0\",0,0\r", "OK", 1); //Set APN
        sendAT("AT+CGDCONT=1,\"IP\",\"iot.ince.net\",\"0.0.0.0\",0,0\r", "OK", 1); //Set APN
        sendAT("AT+CGSN\r", "AT+CGSN", 1); //Get IMEI

        //sendAT("AT+COPS=1,2,\"310410\"\r", "OK", 60); //AT&T
        //sendAT("AT+COPS=1,2,\"310260\"\r", "OK", 60); //T-Mobile
        while(!sendAT("AT+CREG?\r", "+CREG: 0,5", 1)){
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        //sendAT("AT+CGATT=1\r", "OK", 5); //Enable GPRS

        #if MODEM_SLEEP
            sendAT("AT+CSCLK=1\r", "OK", 1);
        #endif

        sendAT("AT+CGATT=0\r", "OK", 5); //Disable GPRS

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
        xEventGroupClearBits(event_group, PROCEED_BIT); //Clear AT response signal

        uart_init(); //Initialize UART
        xTaskCreate(rx_task, "uart_rx_task", RX_BUF_SIZE * 2, NULL, configMAX_PRIORITIES - 1, NULL); //Start AT response receiver task
        #if MODEM == 1
            callbackString = "PB DONE"; //Set modem ready signal
        #elif MODEM == 2  
            callbackString = "PSUTTZ"; //Set modem ready signal
        #endif

        #if MODEM_SLEEP
            modem_wake();
        #else
            modem_power_on();
        #endif

        xEventGroupWaitBits(event_group, PROCEED_BIT, pdTRUE, pdTRUE, 30000 / portTICK_PERIOD_MS); //Wait for ready signal

        int i = 0;
        while(!sendAT("AT+CREG?\r", "+CREG: 0,5", 1) && i < 5){
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            i++;
        }
            #if PROTOCOL == 0
                if(connectMQTT()){ //Connect to MQTT
                    if(!publishMQTT(msg_buf)){ //Publish message. If failed, write failed message to SD card and increment failed message counter.
                        #if STORE_SD
                            if(mountSD()){
                                writeSD(msg_buf, "/sdcard/queue.txt");
                                unmountSD();
                                messageQueue++;
                            }
                        #endif
                    } else{ //If success, publish any queued messages.
                        #if STORE_SD
                            if(messageQueue > 0){
                                if(mountSD()){
                                    readSD("/sdcard/queue.txt");
                                    unmountSD();
                                }
                            }
                        #endif
                    }
                } else{ //If fail to connect, write failed message to SD card and increment queued message counter.
                    #if STORE_SD
                        if(mountSD()){
                            writeSD(msg_buf, "/sdcard/queue.txt");
                            unmountSD();
                            messageQueue++;
                        }
                    #endif
                }
            #else
                if(connectUDP()){ //Connect to UDP
                    #if STORE_SD //Publish any queued messages first
                        if(messageQueue > 0){
                            if(mountSD()){
                                readSD("/sdcard/queue.txt");
                                unmountSD();
                            }
                        }
                    #endif
                    if(!publishUDP(msg_buf)){ //Publish message. If failed, write failed message to SD card and increment failed message counter.
                        #if STORE_SD
                            if(mountSD()){
                                writeSD(msg_buf, "/sdcard/queue.txt");
                                unmountSD();
                                messageQueue++;
                            }
                        #endif
                    }
                } else{ //If fail to connect, write failed message to SD card and increment queued message counter.
                    #if STORE_SD
                        if(mountSD()){
                            writeSD(msg_buf, "/sdcard/queue.txt");
                            unmountSD();
                            messageQueue++;
                        }
                    #endif
                }    
            #endif

        sendAT("AT+CGATT=0\r", "OK", 5); //Disable GPRS
        sendAT("AT+CCLK?\r", "CCLK:", 1); //Get time

        #if MODEM_SLEEP
            modem_sleep();
        #else
            modem_power_off(); //Shut down modem
        #endif
    }
#endif

void app_main(void){
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause(); //Get wakeup cause

    event_group = xEventGroupCreate(); //Create event group

    i2c_master_init(); //Initialize I2C

    #if MEASURE_TEMP
        xTaskCreate(measureTemp, "measureTemp", 1024*2, NULL, configMAX_PRIORITIES - 1, NULL); //Start temp measuring task
    #endif
    //measurePressure();

    if (cause != ESP_SLEEP_WAKEUP_TIMER){ //Do the following on initial boot
        printf("Initial boot, initializing ULP\n");
        if(readADC(RAIN_PIN) == 4095){
            rainGauge = 1;
            printf("Rain gauge status: %d\n", rainGauge);
            init_ulp_program();
        } else{
            #if MEASURE_WIND
                init_ulp_program();
            #endif
        } 
        #if TRANSMIT_DATA
            modem_init(); //Initialize modem
        #endif
    }

    if(rainGauge){
        printf("Rain gauge connected\n");
        xTaskCreate(measureRain, "measureRain", 1024*2, NULL, configMAX_PRIORITIES - 1, NULL); //Start rain measuring task
        xEventGroupWaitBits(event_group, PRECIP_BIT, pdTRUE, pdTRUE, 500 / portTICK_PERIOD_MS); //Wait for at least one precip measurement
    }

    #if MEASURE_WIND
        xTaskCreate(measureWind, "measureWind", 1024*2, NULL, configMAX_PRIORITIES - 1, NULL); //Start wind speed measuring task
        xEventGroupWaitBits(event_group, WIND_BIT, pdTRUE, pdTRUE, 500 / portTICK_PERIOD_MS); //Wait for at least one wind measurement
    #endif

    #if MEASURE_TEMP
        xEventGroupWaitBits(event_group, MEASURE_BIT, pdTRUE, pdTRUE, 500 / portTICK_PERIOD_MS); //Wait for at least one temp measurement
    #endif

    time_t now;
    char time_buf[256]; //Don't change this or format buffer will overflow
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M", &timeinfo);
    //strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    ESP_LOGI(TAG, "The current date/time is: %s", time_buf);

    int currentHour = timeinfo.tm_hour;
    int currentMin = timeinfo.tm_min;
    int currentSec = timeinfo.tm_sec;

    if (((currentMin % STORE_INTERVAL_MIN == 0) || (currentHour == 23 && currentMin == 59 && currentSec >= (59 - MEASURE_INTERVAL_SEC))) && currentMin != lastFormatMin){
        #if MEASURE_PRESSURE
            measurePressure(); //Measure pressure
        #endif
        getVbat(); //Get battery voltage
        msg_buf = format_data(time_buf, true, true);

        if (currentMin % TRANSMIT_INTERVAL_MIN == 0){
            #if TRANSMIT_DATA
                transmit_data(msg_buf);
            #endif
        } else {
            #if STORE_SD
                if (mountSD()){
                    writeSD(msg_buf, "/sdcard/queue.txt");
                    unmountSD();
                    messageQueue++;
                }
            #endif    
        }
        #if STORE_SD
            if (mountSD()){
                writeSD(msg_buf, "/sdcard/data.txt");
                unmountSD();
            }
        #endif
        lastFormatMin = currentMin;
        if ((currentMin % STORE_INTERVAL_MIN != 0)){
            precip = 0;
        }
    }

    uint64_t timeToSleep = (MEASURE_INTERVAL_SEC * 1000000) - (esp_timer_get_time() % (MEASURE_INTERVAL_SEC * 1000000)); //Measurement interval minus time elapsed since boot
    esp_sleep_enable_timer_wakeup(timeToSleep);
    esp_deep_sleep_start();
}
