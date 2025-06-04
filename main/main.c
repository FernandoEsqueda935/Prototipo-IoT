#include "sys/socket.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "netdb.h"
#include "esp_log.h"
#include "wifi_init.h"
#include "fcntl.h"
#include "errno.h"
#include "esp_err.h"
#include "arpa/inet.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "driver/ledc.h"

#define TAG "IoT"

///PWM
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (GPIO_NUM_2) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY(P)            ( (P * (8192 - 1) ) / 100 ) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz
#define PORCENTAJE(P)           ( P > 100 ? 100 : P ) //overflow 
#define GET_PORCENTAJE(D)       ( D * 100 / (8192 - 1)) //overflow

//CLAVES NVS
#define LED_OFF_KEY "led_off"
#define LED_ON_KEY "led_on"
#define ENABLE_TEMPORIZER_KEY "enable_temp"
//WIFI CREDENTIALS
#define SSID "ESP"
#define PASSWORD "12345678"
//HOST PRINCIPAL
#define MAIN_HOST "espserver.local"
#define MAIN_PORT 8250
//HOST SECUNDARIO
#define SECONDARY_HOST "espserver2.local"
#define SECONDARY_PORT 8250
//SERVER PARA PING
#define TEST_HOST "1.1.1.1" 
#define TEST_PORT 80
#define SELECT_TIMEOUT_MS 5000 
//TIMEOUT SOCKETS TCP
#define MAX_TIMEOUT_MS 1000
#define SOCKET_TIMEOU_MS 100
//ACK PARA COMPARAR
#define ACK "ACK"
//IDLEDS PARA STATUS Y CONEXION
#define KA_IDLE 10000000
#define CONN_IDLE 5000000
//PERIFERICOS
#define LED_GPIO GPIO_NUM_3
#define ADC_CHANNEL ADC_CHANNEL_0
//TIEMPO PARA RESTAR EN COMANDO
#define TIME_TO_RESTART 5000000 
//BUFFER DE TCP COMUNICACION
#define BUFFER_SIZE 256

int32_t time_midnight = 0;

adc_oneshot_unit_handle_t adc_handle;

char cmd_base[BUFFER_SIZE] = "UABC:a1262059:L:payload";

SemaphoreHandle_t led_temporized_semaphore = NULL;
SemaphoreHandle_t restart_temporized_semaphore = NULL;

typedef struct temporized_led {
    int32_t led_on;
    int32_t led_off;
    int32_t enable_temporized;
    bool running;
} temporized_led_t;
temporized_led_t led_temporized = {
    .led_on = 10000000,
    .led_off = 10000000,
    .enable_temporized = 0,
    .running = false
};

typedef struct temporized_restart {
    uint64_t restart_on;
    bool activated;
} temporized_restart_t;
temporized_restart_t restart_temporized = {.restart_on = 0,
    .activated = false
};
//encode un byte
uint16_t encode (char byte) {
    uint16_t aux=0;
    
    aux|= ( (( (byte>>7) & 1) << 1) | ( (~byte >>7) & 1 ) ) << 14 ;
    aux|= ( (( (byte>>5) & 1) << 1) | ( (~byte >>5) & 1 ) ) << 12 ;
    aux|= ( (( (byte>>3) & 1) << 1) | ( (~byte >>3) & 1 ) ) << 10 ;
    aux|= ( (( (byte>>1) & 1) << 1) | ( (~byte >>1) & 1 ) ) << 8 ;
    
    aux|= ( (( (byte>>6) & 1)) | ( (~byte >>6) & 1 ) << 1 ) << 6;
    aux|= ( (( (byte>>4) & 1)) | ( (~byte >>4) & 1 ) << 1 ) << 4;
    aux|= ( (( (byte>>2) & 1)) | ( (~byte >>2) & 1 ) << 1 ) << 2;
    aux|= ( (( (byte>>0) & 1)) | ( (~byte >>0) & 1 ) << 1 ) << 0;
    
    return aux;
}
//decode un byte
char decode (uint16_t encode_byte) {
    char byte = 0;
    byte = ( (encode_byte>>15) & 1 )<<7 | ((encode_byte>>13) & 1 )<<5 | ((encode_byte>>11) & 1 )<<3 | ((encode_byte>>9) & 1 )<<1 | ((encode_byte>>6) & 1 )<<6 | ((encode_byte>>4) & 1 )<<4 | ((encode_byte>>2) & 1 )<<2 | ((encode_byte>>0) & 1 )<< 0;
   return byte; 
}
//encode un string
char * encode_msg(char * msg) {
    char * encode_msg = (char *) malloc(256);
    int size = strlen(msg);
    int index = 0;
    uint16_t resultado = 0;
    while (index < size) {
        resultado = encode(msg[index]);
        memcpy(encode_msg + index * 2, &resultado , 2);
        index ++;
    }
    encode_msg[index*2] = 0;
    return encode_msg;
}
//decode un string
char * decode_msg (char * encode_msg) {
    char * msg = (char *) malloc(256);
    int size = strlen(encode_msg);
    int index = 0;
    uint16_t encode_byte = 0;
    while (index < size) {
        memcpy(&encode_byte, encode_msg + index * 2, 2);
        msg[index] = decode(encode_byte);
        index++;
    }
    msg[index]=0;
    return msg;
}

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void set_ledc_duty(int duty)
{
    duty = PORCENTAJE(duty);
    // Set the duty cycle for the LEDC channel
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY(duty)));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

int get_ledc_duty( void ) {
    return GET_PORCENTAJE(ledc_get_duty(LEDC_MODE, LEDC_CHANNEL));
}


esp_err_t get_temporized_values () {
    nvs_handle_t nvs_handle;
    int err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error abriendo NVS: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "NVS abierto correctamente");

    err = nvs_get_i32(nvs_handle, LED_ON_KEY, &led_temporized.led_on);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No se encontró la clave %s", LED_ON_KEY);
        return err;
    }
    err = nvs_get_i32(nvs_handle, LED_OFF_KEY, &led_temporized.led_off);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No se encontró la clave %s", LED_ON_KEY);
        return err;
    }
    err = nvs_get_i32(nvs_handle, ENABLE_TEMPORIZER_KEY, &led_temporized.enable_temporized);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No se encontró la clave %s", LED_ON_KEY);
        return err;
    }
        
    return ESP_OK;
    nvs_close(nvs_handle);
}

esp_err_t set_temporized_value (const char * key, int32_t value) {
    nvs_handle_t nvs_handle;
    int err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error abriendo NVS: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "NVS abierto correctamente");

    err = nvs_set_i32(nvs_handle, key, value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error guardando la clave %s: %s", key, esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

void init_adc (adc_oneshot_unit_handle_t *out_adc_one_shot_handle) {
    adc_oneshot_unit_handle_t adc_oneshot_handle; 
    adc_oneshot_unit_init_cfg_t  cfg = {
        .unit_id = ADC_UNIT_1,
    }; 
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg, &adc_oneshot_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_oneshot_handle, ADC_CHANNEL , &config));

    *out_adc_one_shot_handle = adc_oneshot_handle;  
}

void init_gpio (void){
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_INPUT_OUTPUT);
}

int adc_read (adc_oneshot_unit_handle_t adc_handle) {
    int val = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, ADC_CHANNEL, &val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading ADC: %d", ret);
        return -1;
    }
    return val;
}

bool set_gpio (int val) {
    if (val == 0) {
        gpio_set_level(LED_GPIO, 0);
    } else if (val == 1) {
        gpio_set_level(LED_GPIO, 1);
    } else {
        ESP_LOGE(TAG, "INVALID GPIO VALUE: %d", val);
        return false;
    }
    return true;
}

int get_gpio (void) {
    return gpio_get_level(LED_GPIO);
}

enum {
    STATUS_UABC = 0,
    STATUS_USER = 5,
    STATUS_OP = 14,
    STATUS_PAYLOAD = 16,
} status_e;

enum {
    SOCKET_OK = 0,
    SOCKET_ERROR,
    SOCKET_NOT_CONNECTED,
    SOCKET_CLOSED_BY_SERVER,
    SOCKET_TIMEOUT,
    ACK_RECEIVED,
    NO_ACK_RECEIVED,
} socket_status_e;

bool set_socket_non_blocking(int fd)
{
    int flags = fcntl(fd, F_GETFL);
    if (flags < 0) return false;
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
        return false;
    }
    return true;
}

bool check_connection(void)
{
    const char tag[] = "SOCKET_PING";
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));

    if (inet_pton(AF_INET, TEST_HOST, &dest_addr.sin_addr) != 1) {
        ESP_LOGE(tag, "Invalid IP address format");
        return false;
    }

    dest_addr.sin_family = addr_family;
    dest_addr.sin_port = htons(TEST_PORT);

    int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(tag, "Unable to create socket: errno %d", errno);
        return false;
    }

    if (!set_socket_non_blocking(sock)) {
        ESP_LOGE(tag, "Unable to set socket non-blocking: errno %d", errno);
        close(sock);
        return false;
    }

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        if (errno == EINPROGRESS) {
            fd_set fdset;
            FD_ZERO(&fdset);
            FD_SET(sock, &fdset);

            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = SELECT_TIMEOUT_MS * 1000;

            int res = select(sock + 1, NULL, &fdset, NULL, &timeout);
            if (res <= 0) {
                ESP_LOGE(tag, "Connection timeout or select error: errno %d", errno);
                close(sock);
                return false;
            }

            int sockerr;
            socklen_t len = sizeof(sockerr);
            if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &sockerr, &len) < 0) {
                ESP_LOGE(tag, "getsockopt failed: errno %d", errno);
                close(sock);
                return false;
            }
            if (sockerr != 0) {
                ESP_LOGE(tag, "Connection failed: errno %d", sockerr);
                close(sock);
                return false;
            }
        } else {
            ESP_LOGE(tag, "Immediate connection failed: errno %d", errno);
            close(sock);
            return false;
        }
    }
    close(sock);
    return true;
}

uint8_t socket_status (int len) {
    if ( (len == -1) && (errno == EAGAIN) ) {
        return SOCKET_TIMEOUT;
    }
    else if (len == -1 && errno == ENOTCONN) {
        ESP_LOGE(TAG, "Socket error: %d", errno);
        return SOCKET_NOT_CONNECTED;
    }
    else if (len == 0) {
        ESP_LOGI(TAG, "Socket disconnected");
        return SOCKET_CLOSED_BY_SERVER;
    }
    else if (len < 0) {
        ESP_LOGE(TAG, "Socket error: %d", errno);
        return SOCKET_ERROR;
    }
    else {
        ESP_LOGI(TAG, "Socket OK");
        return SOCKET_OK;
    }
}

int execute_tcp_cmd (char * cmd) {
        switch (cmd[0]) {
        case 'R': 
            switch (cmd[1]) {
                case 'A':
                    ESP_LOGI("TCP", "Reading ADC... %d", adc_read(adc_handle));
                    return adc_read(adc_handle);
                case 'L':
                    return get_gpio();
                case 'P':
                    return get_ledc_duty();
                default:
                    ESP_LOGE("TCP", "Invalid record: %c", cmd[1]);
                    return -1;
            }
            break;
        case 'W':
                switch (cmd[1]) {
                    case 'A':
                        ESP_LOGE("TCP", "Cannot write to ADC:");
                        return -1;
                    case 'L':
                        set_gpio(atoi(cmd + 2));
                        return get_gpio();
                    case 'N':
                        xSemaphoreTake(led_temporized_semaphore, portMAX_DELAY);
                        led_temporized.led_on = atoi(cmd + 2) * 60;
                        ESP_LOGI("TCP", "LED ON: %d", (int) led_temporized.led_on);
                        set_temporized_value(LED_ON_KEY, led_temporized.led_on);
                        xSemaphoreGive(led_temporized_semaphore);
                        return atoi(cmd + 2);
                    case 'F':
                        xSemaphoreTake(led_temporized_semaphore, portMAX_DELAY);
                        led_temporized.led_off = atoi(cmd + 2) * 60;
                        ESP_LOGI("TCP", "LED OFF: %d", (int) led_temporized.led_off);
                        set_temporized_value(LED_OFF_KEY, led_temporized.led_off);
                        xSemaphoreGive(led_temporized_semaphore);
                        return atoi(cmd + 2);                    
                    case 'H':
                        xSemaphoreTake(led_temporized_semaphore, portMAX_DELAY);
                        led_temporized.enable_temporized = atoi(cmd + 2);
                        set_temporized_value(ENABLE_TEMPORIZER_KEY, led_temporized.enable_temporized);
                        xSemaphoreGive(led_temporized_semaphore);
                        return atoi(cmd + 2);
                    case 'I':
                        xSemaphoreTake(restart_temporized_semaphore, portMAX_DELAY);
                        restart_temporized.restart_on = esp_timer_get_time() + TIME_TO_RESTART;
                        restart_temporized.activated = true;
                        ESP_LOGI("TCP", "RESTART ON: %d", (int) restart_temporized.restart_on);
                        xSemaphoreGive(restart_temporized_semaphore);
                        return atoi(cmd + 2);
                    case 'U':
                        xSemaphoreTake(restart_temporized_semaphore, portMAX_DELAY);
                        restart_temporized.activated = false;
                        ESP_LOGI("TCP", "RESTART OFF: %d", (int) restart_temporized.restart_on);
                        xSemaphoreGive(restart_temporized_semaphore);
                        return atoi(cmd + 2);
                    case 'P':
                        set_ledc_duty(atoi(cmd + 2));
                        return get_ledc_duty();
                    default:
                        ESP_LOGE("TCP", "Invalid record: %c", cmd[1]);
                        return -1;
                }
            break;
        default:
            ESP_LOGE("TCP", "Invalid opcode: %c", cmd[0]);
            break;
    }
    return -1;
}

uint8_t send_status (int sock, char op) {
    char * payload = malloc(BUFFER_SIZE);
    int ret = 0;
    cmd_base[STATUS_OP] = op;
    int timeout = 0;
    char * msg = encode_msg(cmd_base);
    int len = send(sock, msg, strlen(msg), 0);
    free(msg);
    ret = socket_status(len);
    if (ret != SOCKET_OK) {
        ESP_LOGE(TAG, "Socket error: %d", ret);
        free(payload);
        return ret;
    }

    do {
        len = recv(sock, payload, BUFFER_SIZE, 0);
        ret = socket_status(len);
        timeout += SOCKET_TIMEOU_MS;
    } while ((ret == SOCKET_TIMEOUT) && (timeout < MAX_TIMEOUT_MS));

    if (ret!= SOCKET_OK) {
        ESP_LOGE(TAG, "Socket error: %d", ret);
        free(payload);
        return ret;
    }
    //ya que el mensaje esta completo, se decodifica
    payload[len] = '\0';
    msg = decode_msg(payload);
    strcpy(payload, msg);
    free(msg);
    ESP_LOGI(TAG, "Received: %s", payload);
    if (strncmp(payload, ACK, strlen(ACK)) == 0) {
        ESP_LOGI(TAG, "ACK received");
        free(payload);
        return ACK_RECEIVED;
    }
    else {
        ESP_LOGE(TAG, "No ACK received");
        free(payload);
        return NO_ACK_RECEIVED;
    }
}

void init_time(void) {
    setenv("TZ", "PST8PDT,M3.2.0,M11.1.0", 1);
    tzset();

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year < (2020 - 1900)) {
        ESP_LOGW(TAG, "No pudo sincronizarse");
    }

    time_midnight = now - (timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec);
    
    ESP_LOGI(TAG, "Hora: %ld", (long) now);
    esp_sntp_stop();
}

uint8_t receive_cmd (int sock) {
    char * payload = malloc(BUFFER_SIZE);
    int ret = 0;
    int len = recv(sock, payload, BUFFER_SIZE, 0);
    ret = socket_status(len);
    if (ret == SOCKET_TIMEOUT) {
        free(payload);
        return SOCKET_TIMEOUT;
    }
    else if (ret != SOCKET_OK) {
        free(payload);
        return SOCKET_ERROR;
    }
    //ya que el mensaje esta completo, se decodifica
    payload[len] = 0;
    char * msg = decode_msg(payload);
    strcpy(payload, msg);
    free(msg);
    ESP_LOGI("SOCKET", "Received: %s", payload);
    int resp = execute_tcp_cmd(payload);
    if (resp >= 0) {
        strcpy(payload, "ACK:");
        itoa(resp, payload+4, 10);
        ///se codifica antes de enviar
        char * msg = encode_msg(payload);
        len = send(sock, msg, strlen(msg), 0);
        free(msg);
        ret = socket_status(len);
        if (ret != SOCKET_OK) {
            ESP_LOGE(TAG, "Socket error: %d", ret);
            free(payload);
            return SOCKET_ERROR;
        }
        ESP_LOGI(TAG, "Sent: %s", payload);
        free(payload);
        return SOCKET_OK;
    }
    else {
        strcpy(payload, "NACK");
        ///se codifica antes de enviar
        char * msg = encode_msg(payload);
        len = send(sock, msg, strlen(msg), 0);
        free(msg);
        return NO_ACK_RECEIVED;
    }
}

void tcp_task (void * args) {
    char * host = MAIN_HOST;
    int port = MAIN_PORT;
    bool connected = true;
    bool was_connected = true;
    uint8_t ret = SOCKET_OK;
    uint64_t current_time = 0;
    uint64_t last_time = 0;
    int64_t current_system_time = 0;

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = SOCKET_TIMEOU_MS * 1000; 

    struct addrinfo hints, *res;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    //ciclo de la tarea, si no hay internet regresa hasta aqui para cambiar el host
    //cuando se detecte algun cambio en el estado de la conexion se reinicia el socket
    while (true) {
        if (check_connection()) {
            host = MAIN_HOST;
            port = MAIN_PORT;
            was_connected = true;
        }
        else{
            host = SECONDARY_HOST;
            port = SECONDARY_PORT;
            was_connected = false;
        }

        int err = getaddrinfo(host, NULL, &hints, &res);
        if (err != 0) {
            ESP_LOGE(TAG, "getaddrinfo failed: %d", err);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        struct sockaddr_in *addr = (struct sockaddr_in *)res->ai_addr;
        addr->sin_port = htons(port);

        char ip_str[INET_ADDRSTRLEN]; 
        inet_ntop(AF_INET, &(addr->sin_addr), ip_str, sizeof(ip_str));

        ESP_LOGI(TAG, "Resolved IP: %s", ip_str);
        
        int sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: %d", errno);
            close(sock);
        }

        if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            ESP_LOGE(TAG, "Failed to set SO_RCVTIMEO: errno %d", errno);
        }

        if (setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0) {
            ESP_LOGE(TAG, "Failed to set SO_SNDTIMEO: errno %d", errno);
        }

        err = connect(sock, (struct sockaddr *) addr, sizeof( struct sockaddr_in));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket connect failed: %d", err);
            close(sock);
        }
        else {
            ESP_LOGI(TAG, "Socket connected to %s:%d", host, ntohs(addr->sin_port));
            //haciendo el ciclo del login, hasta que el servidor acepte la conexion o 
            //haya un problema con la conexion, se reinicia y se revisa el internet
            while (true) {
                ret = send_status(sock, 'L');
                last_time = esp_timer_get_time();
                if (ret == ACK_RECEIVED) {
                    //haciendo el ciclo principal despues del login
                    while (true) { 
                        current_time = esp_timer_get_time();
                        if (current_time - last_time >= KA_IDLE) {
                            ESP_LOGI(TAG, "CURRENT SYSTEM TIME: %ld", (long) current_system_time);
                            ret = send_status(sock, 'K');
                            if (ret == ACK_RECEIVED) {
                                ESP_LOGI(TAG, "Keep alive sent");
                                last_time = current_time;
                            }
                            else if (ret == NO_ACK_RECEIVED) {
                                ESP_LOGE(TAG, "No ACK received for keep alive");
                            }
                            else {
                                ESP_LOGE(TAG, "Socket error: %d", ret);
                                close(sock);
                                break;
                            }
                        }

                        if (current_time - last_time >= CONN_IDLE) {
                            connected = check_connection();
                            if (connected && !was_connected) {
                                ESP_LOGI(TAG, "Now reconnecting");
                                close(sock);
                                break;
                            }
                            else if (!connected && was_connected) {
                                ESP_LOGI(TAG, "Connecting to secondary server");
                                close(sock);
                                break;
                            }
                        }
                           
                        ret = receive_cmd(sock);
                        if (ret == SOCKET_CLOSED_BY_SERVER) {
                            ESP_LOGI(TAG, "Socket closed by server, reconnecting");
                            close(sock);
                            break;
                        }
                        else if (ret == SOCKET_NOT_CONNECTED) {
                            ESP_LOGI(TAG, "Socket not connected, reconnecting");
                            close(sock);
                            break;
                        }
                        else if (ret == SOCKET_ERROR) {
                            ESP_LOGI(TAG, "Socket error: %d", ret);
                            close(sock);
                            break;
                        }

                        xSemaphoreTake(led_temporized_semaphore, portMAX_DELAY);
                        if (led_temporized.enable_temporized) {
                            time(&current_system_time);
                            current_system_time= current_system_time - time_midnight;
                            if ( (current_system_time) >= 86400) {
                                init_time();
                            } else if ( (current_system_time >= led_temporized.led_on) && (!led_temporized.running) && (current_system_time < led_temporized.led_off)) {
                                gpio_set_level(LED_GPIO, 1);
                                led_temporized.running = true;
                                ESP_LOGI(TAG, "LED ON %ld", (long) current_system_time);
                            } else if ( (current_system_time >= led_temporized.led_off) && (led_temporized.running) ) {
                                gpio_set_level(LED_GPIO, 0);
                                led_temporized.running = false;
                                ESP_LOGI(TAG, "LED OFF %ld", (long) current_system_time);
                            }
                        }
                        xSemaphoreGive(led_temporized_semaphore);
                        
                        xSemaphoreTake(restart_temporized_semaphore, portMAX_DELAY);
                        if (restart_temporized.activated) {
                            if (current_time >= restart_temporized.restart_on) {
                                close(sock);
                                ESP_LOGI(TAG, "Restarting system");
                                esp_restart();
                            }
                        }
                        xSemaphoreGive(restart_temporized_semaphore);

                        vTaskDelay(20 / portTICK_PERIOD_MS);
                    }
                }
                else if (ret == NO_ACK_RECEIVED) {
                    ESP_LOGE(TAG, "No ACK received for login");
                }
                else {
                    ESP_LOGE(TAG, "Socket error: %d", ret);
                    close(sock);
                    break;
                }
                vTaskDelay(20/ portTICK_PERIOD_MS);
            }
        }
        close(sock);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    freeaddrinfo(res);
}

void app_main(void) {
    led_temporized_semaphore = xSemaphoreCreateMutex();
    restart_temporized_semaphore = xSemaphoreCreateMutex();
    get_temporized_values();
    example_ledc_init();
    init_adc(&adc_handle);
    init_gpio();
    wifi_init(SSID, PASSWORD);
    init_time();
    xTaskCreate((TaskFunction_t) tcp_task, "tcp_task", 4096, NULL, 5, NULL);
}
