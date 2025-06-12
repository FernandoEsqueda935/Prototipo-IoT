/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "esp_transport.h"
#include "esp_transport_tcp.h"
#include "lwip/netdb.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "adc_init.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "math.h"

#define TARGET_HOST "192.168.124.89"
#define TARGET_PORT 4000

#define NOT_DEV_ID 0x01
#define SERVER_CODE 0x77

#define SENSORES_NUM 2

#define D_ID_KEY "d_id"
#define S1_ID_KEY "s1_id"
#define S2_ID_KEY "s2_id"

#define ADC_PATTERN_NUM 3

#define GET_AVERAGE_MEASURE_TIMER_PERIOD_uS 208    //208
#define GET_ALL_AVERAGE_MEASUR_PERIOD_uS 833       //833


uint8_t d_id = 0;

typedef struct s_info {
    uint8_t s_id;
    int32_t umbral;
    uint32_t valor;
} s_info_t;

s_info_t sensores[SENSORES_NUM] = { 0 };

typedef struct d_id_status {
    uint8_t dev_id;
    uint8_t status; 
} d_id_status_t;

enum dev_id_state_e {
    DEV_LOGED = 0x00,
    DEV_ALIVE = 0x01,
};

typedef struct tcp_frame {
    uint8_t code;
    uint8_t d_id;
    uint8_t s_id;
    uint8_t op_code;
    uint32_t valor;
} tcp_frame_t;

tcp_frame_t frame = {
    .code = 'E',
    .d_id = 0x00,
    .s_id = 0x00,
    .op_code = 0x00,
    .valor = 0x00000008
};

static const char *TAG = "tcp_transport_client";
char payload[sizeof(tcp_frame_t)] = {0};

// valor maximo de muestreo son 10k muestras por segun con un maximo de 4095, si se saca 4095 * 4095 * 10 k  el valor maximo es 167,XXX,XXX,XXX y 2^50 es 1,125,899,906,842,624
//entonces solo es necesario 50 bits para guardar el valor y el resto para guardar el numero de muestras.

//CAMBIO: para poder medir hasta 10 minutos en una sola variable, se utilizara 20 bits para almanecar los samples que serian 
// ( 1200 samples por segundo * 60 segundos * 10 minutos) = 720000 samples con 2^20 = 1,048,576 alcanza perfecto el maximo de 10 minutos
// y 44 bits para almacenar el valor, en este caso el valor maximo por medicion deberia ser menor a 1 V o (1000 mV porque lo regresa en mV), agregando un pequeño 
// offset para cualquier fallo suponiendo que el valor maximo es de 1500 * 1500 * (1200 * 60 * 10) = 1.62^12 y 2^44 = 1.7^13, entonces se puede almacenar el valor sin problemas
// de esta forma se puede configurar desde 10 minutos hacia abajo en cualquier freciencia. 
typedef struct i_rms {
    uint64_t value : 44;
    uint32_t samples : 20;
} i_rms_t;

static const uint8_t adc_pattern [ADC_PATTERN_NUM] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4
};

adc_cali_handle_t adc_cali_handle = NULL;

EventGroupHandle_t measures_event_group = NULL;
EventBits_t next_average_bit = BIT0, send_all_average_bit = BIT1, total_i_rms_bit = BIT2;

uint64_t cali_average_voltage = 0;
uint16_t cali_average_voltage_count = 0;

i_rms_t total_i_rms = {0};
i_rms_t raw_i_rms = {0};

i_rms_t total_i_rms_v[ADC_PATTERN_NUM - 1] = {0};
i_rms_t raw_i_rms_v[ADC_PATTERN_NUM - 1] = {0};

channel_sampling_t adc_3_channel_sampling = {0};
channel_sampling_t adc_4_channel_sampling = {0};

SemaphoreHandle_t raw_i_rms_mutex = NULL;
SemaphoreHandle_t total_i_rms_mutex = NULL;


static void get_average_measure(void* arg);
static void send_all_average_measure(void* arg);

void marshalling_tcp_frame (char * payload) {
    payload[0] = frame.code;
    payload[1] = frame.d_id;
    payload[2] = frame.s_id;
    payload[3] = frame.op_code;
    payload[4] = (frame.valor >> 24) & 0xFF;
    payload[5] = (frame.valor >> 16) & 0xFF;
    payload[6] = (frame.valor >> 8) & 0xFF;
    payload[7] = frame.valor & 0xFF;
} 

TaskFunction_t get_raw_measure_task (void *args) {
    uint8_t raw_frame[ADC_FRAME_SIZE] = {0};
    uint32_t frame_size = 0;
    int cali_value = 0;
    channel_sampling_t voltage_offset = {0};
    int raw_value = 0;
    esp_err_t ret;
    adc_digi_output_data_t * adc_temp_data = NULL;

    adc_continuous_handle_t adc_handle = NULL;
    init_adc(&adc_handle, adc_pattern, ADC_PATTERN_NUM);
    adc_cali_init(&adc_cali_handle);

    while (true) {
        //Se espera a que se active el timer para poder hacer el promedio de mediciones, puede variar la cantidad de samples para la medicion
        //depende de la frecuencia en que se quiere obtener una muestra
        if (xEventGroupWaitBits(measures_event_group, next_average_bit, pdTRUE, pdFALSE, (20/portTICK_PERIOD_MS)) & next_average_bit) {
            ret = adc_continuous_read(adc_handle, raw_frame, ADC_FRAME_SIZE, &frame_size, 20);
            if (ret == ESP_OK) {
                if (frame_size > 0) {
                    for (uint16_t measure_index = 0; measure_index < frame_size; measure_index+=SOC_ADC_DIGI_RESULT_BYTES) {
                        adc_temp_data = (adc_digi_output_data_t *)&raw_frame[measure_index];
                        raw_value = adc_temp_data->type1.data;
                        //Se acumula las mediciones para hacer un promedio, este promedio se hace cada que el timer se activa y libera la bandera
                        if (adc_temp_data->type1.channel == ADC_CHANNEL_0) {
                            voltage_offset.value += raw_value;
                            voltage_offset.samples++;
                        } else if (adc_temp_data->type1.channel == ADC_CHANNEL_3) {
                            adc_3_channel_sampling.value += raw_value;
                            adc_3_channel_sampling.samples++;
                        }
                        else if (adc_temp_data->type1.channel == ADC_CHANNEL_4) {
                            //se acumula el valor de la medicion de voltaje para hacer un promedio
                            adc_4_channel_sampling.value += raw_value;
                            adc_4_channel_sampling.samples++;
                        }
                    }
                }
            }
            if (voltage_offset.samples > 0 && adc_3_channel_sampling.samples > 0 && adc_4_channel_sampling.samples > 0) {
                //se les saca el promedio
                voltage_offset.value = voltage_offset.value / voltage_offset.samples;
                adc_3_channel_sampling.value = adc_3_channel_sampling.value / adc_3_channel_sampling.samples;
                adc_4_channel_sampling.value = adc_4_channel_sampling.value / adc_4_channel_sampling.samples;
                //se le saca la diferencian entre voltage offset y el valor de la medicion, esto es para que siempre tenga un valor real con respecto al
                //offset
                cali_value = voltage_offset.value - adc_3_channel_sampling.value;
                xSemaphoreTake(raw_i_rms_mutex, portMAX_DELAY);
                //se saca el cuadrado de cada medicion y se acumula, la primera parte del RMS
                raw_i_rms_v[0].value += cali_value * cali_value;
                raw_i_rms_v[0].samples++;
                cali_value = voltage_offset.value - adc_4_channel_sampling.value;
                raw_i_rms_v[1].value += cali_value * cali_value;
                raw_i_rms_v[1].samples++;
                xSemaphoreGive(raw_i_rms_mutex);
                //Se reinician los acumuladores para los promedios
                adc_3_channel_sampling.value = 0;
                adc_3_channel_sampling.samples = 0;
                adc_4_channel_sampling.value = 0;
                adc_4_channel_sampling.samples = 0;
                voltage_offset.value = 0;
                voltage_offset.samples = 0;
            }
        }
    }
}

TaskFunction_t get_average_measure_task (void * args) {
    int i_rms_cali_value = 0;
    int i_rms_cali_value_min_ref = 0;
    adc_cali_raw_to_voltage(adc_cali_handle, 0, &i_rms_cali_value_min_ref);
    while (true) {
        //Ahora se espera a que se active el timer para obtener el RMS de cada medicion con respecto al tiempo de muestreo
        //Esto puede ser arbitrario, pero es necesario nos desbordar las variables
        if (xEventGroupWaitBits(measures_event_group, send_all_average_bit, pdTRUE, pdFALSE, (20/ portTICK_PERIOD_MS)) & send_all_average_bit) {
            xSemaphoreTake(raw_i_rms_mutex, portMAX_DELAY);
            //siempre y cuando no se vaya haber una division entre 0 se hace la siguiente parte
            if (raw_i_rms_v[0].samples > 0 && raw_i_rms_v[1].samples > 0) {
                xSemaphoreTake(total_i_rms_mutex, portMAX_DELAY);
                //la el valor se calibra para que nos de su representacion en voltaje real, entonces nos dira que voltaje fue el que se leyo
                //este voltaje representa la corriente RMS despues de divir entre las muestras y sacar la raiz cuadrada
                adc_cali_raw_to_voltage(adc_cali_handle, (int) (sqrt(raw_i_rms_v[0].value/raw_i_rms_v[0].samples)), &i_rms_cali_value);
                total_i_rms_v[0].value += i_rms_cali_value - i_rms_cali_value_min_ref;
                total_i_rms_v[0].samples++; 
                adc_cali_raw_to_voltage(adc_cali_handle, (int) (sqrt(raw_i_rms_v[1].value/raw_i_rms_v[1].samples)), &i_rms_cali_value);
                total_i_rms_v[1].value += i_rms_cali_value - i_rms_cali_value_min_ref;
                total_i_rms_v[1].samples++;
                //se acumula el valor de rms para sacar un promedio al final, y este podria considerarse como RMS total, con un promedio del RMS
                //ya se puede obtener el valor de corriente en cualquier tiempo
                xSemaphoreGive(total_i_rms_mutex);
                raw_i_rms_v[0].value = 0;
                raw_i_rms_v[0].samples = 0;
                raw_i_rms_v[1].value = 0;
                raw_i_rms_v[1].samples = 0;
            }
            xSemaphoreGive(raw_i_rms_mutex);
        }
    }
}

static void get_average_measure(void* arg) {
    xEventGroupSetBits(measures_event_group, next_average_bit);
}

static void send_all_average_measure(void* arg) {
    xEventGroupSetBits(measures_event_group, send_all_average_bit);
}

void parser_tcp_frame (char * payload) {
    frame.code = payload[0];
    frame.d_id = payload[1];
    frame.s_id = payload[2];
    frame.op_code = payload[3];
    frame.valor = (payload[4] << 24) | (payload[5] << 16) | (payload[6] << 8) | payload[7];
}

void set_d_id ( uint8_t id ) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return; // Error opening NVS
    }

    d_id = id;
    err = nvs_set_u8(nvs_handle, D_ID_KEY, d_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error setting d_id: %s", esp_err_to_name(err));
    } else {
        nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "Device ID set to %d", d_id);
    }

    nvs_close(nvs_handle);
}

void set_s_ids ( uint8_t slot ) {
    char s_id_key[10] = "sn_id";

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return; // Error opening NVS
    }

    sprintf(s_id_key, "s%c_id", (slot + '0') );

    err = nvs_set_u8(nvs_handle, s_id_key, sensores[slot - 1].s_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error setting sn_id: %s", esp_err_to_name(err));
    } else {
        nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "Sensor IDs set to %d and %d", sensores[0].s_id, sensores[1].s_id);
    }

    nvs_close(nvs_handle);
}

void get_d_id (void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return; // Error opening NVS
    }
    err = nvs_get_u8(nvs_handle, D_ID_KEY, &d_id);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        d_id = 0; // Default value
        ESP_LOGI(TAG, "Device ID not found, defaulting to 0");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading d_id: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Device ID retrieved: %d", d_id);
    }
    nvs_close(nvs_handle);
}

void get_s_ids (void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return; // Error opening NVS
    }
    err = nvs_get_u8(nvs_handle, S1_ID_KEY, &sensores[0].s_id);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        sensores[0].s_id = 0; // Default value
        ESP_LOGI(TAG, "Sensor 1 ID not found, defaulting to 0");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading s1_id: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Sensor 1 ID retrieved: %d", sensores[0].s_id);
    }

    err = nvs_get_u8(nvs_handle, S2_ID_KEY, &sensores[1].s_id);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        sensores[1].s_id = 0; // Default value
        ESP_LOGI(TAG, "Sensor 2 ID not found, defaulting to 0");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading s2_id: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Sensor 2 ID retrieved: %d", sensores[1].s_id);
    }

    nvs_close(nvs_handle);
}

uint8_t execute_cmd (esp_transport_handle_t transport) {
    char tx_buffer[sizeof(tcp_frame_t)];
    char rx_buffer[sizeof(tcp_frame_t)];
    switch (frame.op_code) {
        case 'D':
            if (d_id == 0) {
                d_id = frame.d_id;
                tx_buffer[0] = 'E';
                esp_transport_write(transport, tx_buffer, 1, 100 );
                set_d_id(d_id);
            }
            else {
                tx_buffer[0] = 'P'; // Device already registered
                esp_transport_write(transport, tx_buffer, 1, 100);
            }
            return 0;
        case 'S':
            sensores[frame.s_id - 1].s_id = frame.valor;
            tx_buffer[0] = 'E';
            esp_transport_write(transport, tx_buffer, 1, 100);
            ESP_LOGI(TAG, "Sensor %d registered with ID %d", (int) frame.s_id, (int) frame.valor);
            set_s_ids(frame.s_id);
            return 0;
        case 'M':
            marshalling_tcp_frame(tx_buffer);
            esp_transport_write(transport, tx_buffer, sizeof(tcp_frame_t) , 100);
            esp_transport_read(transport, rx_buffer, sizeof(tcp_frame_t) , 100);
            if (rx_buffer[0] == 'E') {
                ESP_LOGI(TAG, "Command executed successfully");
            } else {
                ESP_LOGE(TAG, "Command execution failed");
            }
            return 0;
        case 'C':
            marshalling_tcp_frame(tx_buffer);
            esp_transport_write(transport, tx_buffer, sizeof(tcp_frame_t), 100);
            return 0;
        case 'U':
            marshalling_tcp_frame(tx_buffer);
            esp_transport_write(transport, tx_buffer, sizeof(tcp_frame_t), 100);
            esp_transport_read(transport, rx_buffer, sizeof(tcp_frame_t), 100);
            if (rx_buffer[0] == 'E') {
                ESP_LOGI(TAG, "Update command executed successfully");
            } else {
                ESP_LOGE(TAG, "Update command execution failed");
            }
            return 0;
        default:
            ESP_LOGW(TAG, "Unknown command code: 0x%02X", frame.op_code);
            return 0; // Unknown command
    }

    return 0;
}

int8_t recv_frame (esp_transport_handle_t transport, char *rx_buffer) {
    int len = esp_transport_read(transport, rx_buffer, sizeof(tcp_frame_t), 100);
    if (len < 0) {
        ESP_LOGE(TAG, "Error receiving data: %d", len);
        return -1; // Connection closed
    } else if (len == 0) {
        return 0;
    }

    if (len != sizeof(tcp_frame_t)) {
        ESP_LOGE(TAG, "Received unexpected frame size: %d", len);
        return 0; // Unexpected frame size
    }

    parser_tcp_frame(rx_buffer);
    ESP_LOGI(TAG, "Received frame: code=0x%02X, d_id=0x%02X, s_id=0x%02X, op_code=0x%02X, valor=%u",
             frame.code, frame.d_id, frame.s_id, frame.op_code, (unsigned int) frame.valor);

    execute_cmd(transport);

    return 0; // Frame received successfully
}

uint32_t get_measure ( void ) {
    
    xSemaphoreTake(total_i_rms_mutex, portMAX_DELAY);
    ESP_LOGI(TAG, "adc 1: %u, adc 2: %u", (unsigned int) total_i_rms_v[0].samples, (unsigned int) total_i_rms_v[1].samples);

    if (sensores[0].s_id != 0) {
        
        if (total_i_rms_v[0].samples > 0) {
            //1 volt representa 15 A reales, entonces multiplicamos por 15 el valor que nos de, si fuera 0.5 v entonces serian 7.5 A
            //se divide entre 1000 porque el  valor de salida de adc_cali es en mV, entonces dividimos entre 1000 para obtener el voltaje real
            //hacemos el promedio y obtener el i rms en el tiempo en el que se estuvo midiendo
            sensores[0].valor = ((total_i_rms_v[0].value + total_i_rms_v[0].samples / 2) / total_i_rms_v[0].samples);
            total_i_rms_v[0].value = 0;
            total_i_rms_v[0].samples = 0;
            ESP_LOGI(TAG, "Sensor 1 value: %u", (unsigned int) sensores[0].valor);
        }
    }

    if (sensores[1].s_id != 0) {
        if (total_i_rms_v[1].samples > 0) {
        //1 volt representa 15 A reales, entonces multiplicamos por 15 el valor que nos de, si fuera 0.5 v entonces serian 7.5 A
            sensores[1].valor = ( (total_i_rms_v[1].value + total_i_rms_v[1].samples / 2) / total_i_rms_v[1].samples);
            total_i_rms_v[1].value = 0;
            total_i_rms_v[1].samples = 0;
            ESP_LOGI(TAG, "Sensor 2 value: %u", (unsigned int) sensores[1].valor);
        }
    }
    xSemaphoreGive(total_i_rms_mutex);
    return 0;
}

void init_adc_module () {
    raw_i_rms_mutex = xSemaphoreCreateMutex();
    total_i_rms_mutex = xSemaphoreCreateMutex();
    measures_event_group = xEventGroupCreate();
    const esp_timer_create_args_t get_average_measure_timer_args = {
        .callback = &get_average_measure,
        .name = "get_average_measure"
    };

        const esp_timer_create_args_t send_all_average_measure_timer_args = {
        .callback = &send_all_average_measure,
        .name = "send_all_average_measure"
    };

    esp_timer_handle_t get_average_measure_timer;
    esp_timer_handle_t send_all_average_measure_timer;

    ESP_ERROR_CHECK(esp_timer_create(&get_average_measure_timer_args, &get_average_measure_timer));
    ESP_ERROR_CHECK(esp_timer_create(&send_all_average_measure_timer_args, &send_all_average_measure_timer));

    xTaskCreate(get_raw_measure_task, "get_raw_measure_task", 4096, NULL, 5, NULL);
    xTaskCreate(get_average_measure_task, "get_average_measure_task", 4096, NULL, 5, NULL);
    
    ESP_ERROR_CHECK(esp_timer_start_periodic(get_average_measure_timer, GET_AVERAGE_MEASURE_TIMER_PERIOD_uS));
    ESP_ERROR_CHECK(esp_timer_start_periodic(send_all_average_measure_timer, GET_ALL_AVERAGE_MEASUR_PERIOD_uS));
}

static void tcp_transport_client_task(void *pvParameters) {
    int64_t last_time = 0;
    char rx_buffer[128];
    char host_ip[20];
    esp_transport_handle_t transport = esp_transport_tcp_init();

    strncpy(host_ip, TARGET_HOST, sizeof(host_ip));
    host_ip[sizeof(host_ip) - 1] = '\0';

    
    
    while (1) {
        if (transport == NULL) {
            ESP_LOGE(TAG, "Error occurred during esp_transport_tcp_init()");
            break;
        }
        ESP_LOGI(TAG, "Connecting to %s:%d...", host_ip, TARGET_PORT);
        int err = esp_transport_connect(transport, host_ip, TARGET_PORT, 5000); // 5s timeout
        if (err != 0) {
            ESP_LOGE(TAG, "Client unable to connect: errno %d", errno);
            vTaskDelay(2000 / portTICK_PERIOD_MS); // Espera antes de reintentar
            continue;
        }
        ESP_LOGI(TAG, "Successfully connected");

        last_time = esp_timer_get_time();

        if (d_id == 0) {
            frame.op_code = 'D';
            marshalling_tcp_frame(rx_buffer);
            esp_transport_write(transport, rx_buffer, sizeof(tcp_frame_t), 100 );
            ESP_LOGI(TAG, "Sent device ID request frame");
        }
        else {
            frame.op_code = 'C';
            frame.d_id = d_id;
            marshalling_tcp_frame(rx_buffer);
            esp_transport_write(transport, rx_buffer, sizeof(tcp_frame_t), 100 );
            ESP_LOGI(TAG, "Sent device ID frame with ID %d", (int) d_id);
        }

        while (1) {
            
            int ret = recv_frame(transport, rx_buffer);
            
            //ahcer una funcion para que cada vez que se envie el frame de medidas, cada N segundo
            if (esp_timer_get_time() - last_time > 15000000) { // 5 seconds
                    /*
                    frame.op_code = 'M'; // Enviar medidas
                    frame.valor = 25;
                    marshalling_tcp_frame(rx_buffer);
                    esp_transport_write(transport, rx_buffer, sizeof(tcp_frame_t), 1000);
                    ESP_LOGI(TAG, "Sent measures frame");
                    last_time = esp_timer_get_time();
                    */
                get_measure();
                for (int index = 0; index < SENSORES_NUM; index++) {
                    if (sensores[index].s_id != 0) {
                        frame.op_code = 'M'; 
                        frame.s_id = sensores[index].s_id;
                        frame.valor = sensores[index].valor;
                        marshalling_tcp_frame(rx_buffer);
                        esp_transport_write(transport, rx_buffer, sizeof(tcp_frame_t), 100);
                        ESP_LOGI(TAG, "Sent measures frame for sensor %d with value %u", (int) frame.s_id, (unsigned int) frame.valor);
                    }
                }
                last_time = esp_timer_get_time();
            }

            //check_tasks(transport);
            if (ret < 0) {
                ESP_LOGE(TAG, "Connection closed by peer, restarting...");
                break; // Salir del bucle para reconectar
            }
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }


        ESP_LOGE(TAG, "Shutting down transport and restarting...");
        esp_transport_close(transport);
        vTaskDelay(15000 / portTICK_PERIOD_MS); // Espera antes de intentar reconectar
    }
    esp_transport_destroy(transport);
    vTaskDelete(NULL);
}



void app_main(void)
{
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_proxy", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    get_d_id();
    get_s_ids();

    ESP_ERROR_CHECK(example_connect());

    init_adc_module();
    

    xTaskCreate(tcp_transport_client_task, "tcp_transport_client", 4096, NULL, 5, NULL);
}
