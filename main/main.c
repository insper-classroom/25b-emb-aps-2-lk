#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

#include "mpu6050.h"
#include "mouse.h"
#include "Fusion.h"

// ===================== Config =====================
#define SAMPLE_PERIOD       (0.01f)   // 100 Hz
#define DEADZONE_DPS        (20.0f)   // °/s - Sensibilidade do sensor
#define GAIN_DPS2COUNT      (4.0f)    // °/s -> counts - Velocidade
#define GYRO_SENS           (131.0f)  // LSB -> °/s (±250 dps)
#define ACCEL_SENS          (16384.0f)// LSB -> g (±2 g)

// GPIO Pins
#define BTN_SHOOT_GPIO      14        // Botão de disparo
#define BTN_TOGGLE_GPIO     17        // Botão liga/desliga ARMA
#define BTN_BURST_GPIO      22        // Botão modo burst/trigger (3 tiros)
#define BTN_RELOAD_GPIO     15        // Botão reload (tecla R)
#define LED_STATUS_GPIO     13        // LED indicador (arma ligada/desligada)
#define DEBOUNCE_MS         80        // Debounce
#define BURST_DELAY_MS      100       // Delay entre tiros no modo burst (100ms padrão)

#ifndef USE_PICO_DOCK
#  define I2C_SDA_GPIO 4
#  define I2C_SCL_GPIO 5
#else
#  define I2C_SDA_GPIO 17
#  define I2C_SCL_GPIO 16
#endif

#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define MPU_ADDRESS 0x68

// ===================== CONFIGURAÇÃO DE EIXOS =====================
// Sensor EM PÉ, apontado para DIREITA
#define DIR_X  (1.0f)      // use -1.0f para inverter horizontal
#define DIR_Y  (-1.0f)     // use 1.0f para inverter vertical

typedef struct {
    float bx; 
    float by; 
} gyro_bias_t;

// Semáforos globais
static SemaphoreHandle_t toggle_sem = NULL;
static SemaphoreHandle_t burst_sem  = NULL;
static SemaphoreHandle_t shoot_sem  = NULL;
static SemaphoreHandle_t reload_sem = NULL;

// ===================== Protos =====================
static void mpu6050_reset(void);
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
static inline int16_t sat_i16_from_float(float x);
static gyro_bias_t calibrate_gyro_bias(void);
static void gpio_callback(uint gpio, uint32_t events);
static void setup_buttons(void);
static void mpu6050_task(void *p);
static void uart_task(void *p);

// ===================== MPU6050 =================
static void mpu6050_reset(void) {
    uint8_t buf[] = {0x6B, 0x00}; // sai de sleep
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t reg;

    // Accel 0x3B..0x40
    reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (int16_t)((buffer[i * 2] << 8) | buffer[(i * 2) + 1]);
    }

    // Gyro 0x43..0x48
    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (int16_t)((buffer[i * 2] << 8) | buffer[(i * 2) + 1]);
    }

    // Temp 0x41..0x42
    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = (int16_t)((buffer[0] << 8) | buffer[1]);
}

// ===================== Util =======================
static inline int16_t sat_i16_from_float(float x) {
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)x;
}

static gyro_bias_t calibrate_gyro_bias(void) {
    const int N = 500; // ~5 s a 100 Hz
    int16_t accel[3], gyro[3], temp;
    float sx = 0.0f, sy = 0.0f;

    printf("=== CALIBRACAO: Mantenha o sensor PARADO e EM PE! ===\n");
    
    for (int i = 0; i < N; ++i) {
        mpu6050_read_raw(accel, gyro, &temp);
        sx += (gyro[0] / GYRO_SENS);  // gyro[0] = X do sensor (rotação)
        sy += (gyro[2] / GYRO_SENS);  // gyro[2] = Z do sensor (inclinação)
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    gyro_bias_t b = { .bx = sx / (float)N, .by = sy / (float)N };
    printf("=== CALIBRACAO COMPLETA: bias_x=%.3f, bias_y=%.3f ===\n", b.bx, b.by);

    // LED apaga após calibração (arma inicia DESLIGADA)
    gpio_put(LED_STATUS_GPIO, 0);

    return b;
}

// ===================== GPIO Callback ==============
static void gpio_callback(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // AVISO: As variáveis 'static' foram movidas para dentro dos blocos 'if'
    // para reduzir o escopo conforme exigido pelo cppcheck.
    // Isso mantém o comportamento (elas retêm o valor), mas só são visíveis onde usadas.

    if (gpio == BTN_TOGGLE_GPIO && (events & GPIO_IRQ_EDGE_FALL)) {
        static uint32_t last_toggle_time = 0; // Escopo reduzido
        if ((now - last_toggle_time) > DEBOUNCE_MS) {
            last_toggle_time = now;
            xSemaphoreGiveFromISR(toggle_sem, &xHigherPriorityTaskWoken);
        }
    } else if (gpio == BTN_BURST_GPIO && (events & GPIO_IRQ_EDGE_FALL)) {
        static uint32_t last_burst_time = 0; // Escopo reduzido
        if ((now - last_burst_time) > DEBOUNCE_MS) {
            last_burst_time = now;
            xSemaphoreGiveFromISR(burst_sem, &xHigherPriorityTaskWoken);
        }
    } else if (gpio == BTN_SHOOT_GPIO && (events & GPIO_IRQ_EDGE_FALL)) {
        static uint32_t last_shoot_time = 0; // Escopo reduzido
        if ((now - last_shoot_time) > DEBOUNCE_MS) {
            last_shoot_time = now;
            xSemaphoreGiveFromISR(shoot_sem, &xHigherPriorityTaskWoken);
        }
    } else if (gpio == BTN_RELOAD_GPIO && (events & GPIO_IRQ_EDGE_FALL)) {
        static uint32_t last_reload_time = 0; // Escopo reduzido
        if ((now - last_reload_time) > DEBOUNCE_MS) {
            last_reload_time = now;
            xSemaphoreGiveFromISR(reload_sem, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void setup_buttons(void) {
    // Entradas
    gpio_init(BTN_SHOOT_GPIO);
    gpio_set_dir(BTN_SHOOT_GPIO, GPIO_IN);
    gpio_pull_up(BTN_SHOOT_GPIO);

    gpio_init(BTN_TOGGLE_GPIO);
    gpio_set_dir(BTN_TOGGLE_GPIO, GPIO_IN);
    gpio_pull_up(BTN_TOGGLE_GPIO);

    gpio_init(BTN_BURST_GPIO);
    gpio_set_dir(BTN_BURST_GPIO, GPIO_IN);
    gpio_pull_up(BTN_BURST_GPIO);

    gpio_init(BTN_RELOAD_GPIO);
    gpio_set_dir(BTN_RELOAD_GPIO, GPIO_IN);
    gpio_pull_up(BTN_RELOAD_GPIO);

    // LED
    gpio_init(LED_STATUS_GPIO);
    gpio_set_dir(LED_STATUS_GPIO, GPIO_OUT);
    gpio_put(LED_STATUS_GPIO, 0);

    // Interrupções
    gpio_set_irq_enabled_with_callback(BTN_SHOOT_GPIO,
                                       GPIO_IRQ_EDGE_FALL,
                                       true, &gpio_callback);
    gpio_set_irq_enabled(BTN_TOGGLE_GPIO, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BTN_BURST_GPIO, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BTN_RELOAD_GPIO, GPIO_IRQ_EDGE_FALL, true);

    // Semáforos
    toggle_sem = xSemaphoreCreateBinary();
    burst_sem  = xSemaphoreCreateBinary();
    shoot_sem  = xSemaphoreCreateBinary();
    reload_sem = xSemaphoreCreateBinary();
}

// ===================== Tasks ======================
static void mpu6050_task(void *p) {
    QueueHandle_t q = (QueueHandle_t)p;

    bool mouse_enabled = false;
    bool burst_mode    = false;

    printf("=== SENSOR EM PE, APONTADO PARA DIREITA ===\n");
    printf("=== GPIO 17: Liga/Desliga ARMA ===\n");
    printf("=== GPIO 22: Modo Burst (3 tiros) ===\n");
    printf("=== GPIO 15: Reload (tecla R) ===\n");
    printf("=== GPIO 14: Disparar (1x ou 3x no apertar) ===\n");
    printf("=== GPIO 13: LED Indicador ===\n");

    // I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();

    // Calibração
    gyro_bias_t bias = calibrate_gyro_bias();

    int16_t acceleration[3], gyro[3], temp;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    mouse_t mouse;

    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        // Liga/Desliga arma
        if (xSemaphoreTake(toggle_sem, 0) == pdTRUE) {
            mouse_enabled = !mouse_enabled;

            gpio_put(LED_STATUS_GPIO, mouse_enabled ? 1 : 0);

            printf("\n****************************************\n");
            printf("*** ARMA %s ***\n", mouse_enabled ? "LIGADA" : "DESLIGADA");
            printf("****************************************\n");
        }

        // Modo burst
        if (xSemaphoreTake(burst_sem, 0) == pdTRUE) {
            if (mouse_enabled) {
                burst_mode = !burst_mode;

                printf("\n========================================\n");
                printf("*** MODO %s ATIVADO ***\n",
                       burst_mode ? "BURST (3 TIROS)" : "SEMI-AUTO (1 TIRO)");
                printf("========================================\n");
                printf("[DEBUG] burst_mode = %d\n", burst_mode);
            } else {
                printf("\n[!] ARMA DESLIGADA - Ligue primeiro (GPIO 17)!\n");
            }
        }

        // Disparo
        if (xSemaphoreTake(shoot_sem, 0) == pdTRUE) {
            if (mouse_enabled) {
                printf("[DEBUG] Disparando! burst_mode = %d\n", burst_mode);

                if (burst_mode) {
                    printf(">>> BURST SHOT (3x)! <<<\n");

                    for (int i = 0; i < 3; i++) {
                        mouse.axis = 6;  // 6 = burst shot
                        mouse.val  = 1;
                        xQueueSend(q, &mouse, 0);
                        printf("[DEBUG] Enviado tiro %d/3 (axis=6)\n", i + 1);

                        if (i < 2) {
                            vTaskDelay(pdMS_TO_TICKS(BURST_DELAY_MS));
                        }
                    }
                } else {
                    printf(">>> SINGLE SHOT! <<<\n");
                    mouse.axis = 3;  // 3 = single shot
                    mouse.val  = 1;
                    xQueueSend(q, &mouse, 0);
                    printf("[DEBUG] Enviado single shot (axis=3)\n");
                }
            }
        }

        // Reload
        if (xSemaphoreTake(reload_sem, 0) == pdTRUE) {
            printf(">>> RELOAD! (Tecla R) <<<\n");
            mouse.axis = 7;  // 7 = reload (tecla R)
            mouse.val  = 1;
            xQueueSend(q, &mouse, 0);
            printf("[DEBUG] Enviado reload (axis=7)\n");
        }

        // Processamento do sensor
        if (mouse_enabled) {
            mpu6050_read_raw(acceleration, gyro, &temp);

            float gx = (gyro[0] / GYRO_SENS) - bias.bx; // yaw
            float gy = (gyro[2] / GYRO_SENS) - bias.by; // pitch

            FusionVector gyroscope = {
                .axis.x = gyro[0] / GYRO_SENS,
                .axis.y = gyro[2] / GYRO_SENS,
                .axis.z = gyro[1] / GYRO_SENS,
            };
            FusionVector accelerometer = {
                .axis.x = acceleration[0] / ACCEL_SENS,
                .axis.y = acceleration[2] / ACCEL_SENS,
                .axis.z = acceleration[1] / ACCEL_SENS,
            };

            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope,
                                           accelerometer, SAMPLE_PERIOD);

            gx = (fabsf(gx) < DEADZONE_DPS) ? 0.0f : gx;
            gy = (fabsf(gy) < DEADZONE_DPS) ? 0.0f : gy;

            float vx = DIR_X * gx * GAIN_DPS2COUNT;
            float vy = DIR_Y * gy * GAIN_DPS2COUNT;

            mouse.axis = 0;
            mouse.val  = sat_i16_from_float(vx);
            xQueueSend(q, &mouse, 0);

            mouse.axis = 1;
            mouse.val  = sat_i16_from_float(vy);
            xQueueSend(q, &mouse, 0);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10)); // 100 Hz
    }
}

static void uart_task(void *p) {
    QueueHandle_t q = (QueueHandle_t)p;
    mouse_t mouse;

    uart_init(uart0, 115200);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    while (1) {
        if (xQueueReceive(q, &mouse, portMAX_DELAY)) {
            uart_putc_raw(uart0, 0xFF);
            uart_putc_raw(uart0, (uint8_t)mouse.axis);

            uint16_t u = (uint16_t)mouse.val;
            uart_putc_raw(uart0, (uint8_t)(u & 0xFF));
            uart_putc_raw(uart0, (uint8_t)((u >> 8) & 0xFF));
        }
    }
}

int main(void) {
    stdio_init_all();

    QueueHandle_t q = xQueueCreate(10, sizeof(mouse_t));

    setup_buttons();

    xTaskCreate(mpu6050_task, "mpu6050_Task", 8192, q, 1, NULL);
    xTaskCreate(uart_task,    "uart_Task",    1024, q, 1, NULL);

    vTaskStartScheduler();
    while (true) {
        // nunca deveria chegar aqui
    }
}