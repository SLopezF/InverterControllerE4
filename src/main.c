#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "math.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "driver/mcpwm_prelude.h"

#define F_MOTOR             100         // Frecuencia del motor en Hz
#define F_MOTOR_MIN         100           // Frecuencia del motor en Hz

#define PWM_RES_HZ           10000000LL
#define PWM_PERIOD_TICKS     (PWM_RES_HZ / (F_MOTOR * LUT_SIZE))
#define DEADTIME_TICKS       40           // 2 µs de deadtime en subida y 2 µs en bajada

// Pines de salida de cada fase
#define PHASE_A_P            18
#define PHASE_A_N            19
#define PHASE_B_P            21
#define PHASE_B_N            22
#define PHASE_C_P            23
#define PHASE_C_N            25

// Pines para sincronización
#define SYNC_GPIO_INPUT      4   // Conectado al MCPWM como entrada de sync
#define SYNC_GPIO_OUTPUT     5   // Controlado por software para generar el pulso
#define Test_GPIO_OUTPUT    2   // GPIO para pruebas, no se usa en este ejemplo

//Tamaño Look Up table
#define LUT_SIZE             12

#define LUT_COUNT 2
uint16_t lut_buffers[LUT_COUNT+1][LUT_SIZE]; // Crea un buffer extra para guardar el período y asegurarse que es una operación atómica
volatile uint8_t active_lut_index = 0;

uint16_t periodo = PWM_RES_HZ / ((F_MOTOR_MIN) * LUT_SIZE);

mcpwm_cmpr_handle_t cmpA, cmpB, cmpC;
mcpwm_timer_handle_t timerA, timerB, timerC;

uint16_t debug = 0; 


static const char *TAG = "mcpwm_gpio_sync";

//Interrupción de actualización de variables de comparación
bool isr_update_pwm_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_data)
{
    uint16_t *lut = lut_buffers[active_lut_index];//Apunto al LUT activo
    uint16_t new_periodo = lut_buffers[LUT_COUNT][active_lut_index]; // 

    static int index = -1;
    index = (index + 1) % LUT_SIZE; // Incrementar el índice cíclicamente

    if(index == 0){
        gpio_set_level(Test_GPIO_OUTPUT, 1);
    }
    else{
        gpio_set_level(Test_GPIO_OUTPUT, 0);
    }

    ESP_ERROR_CHECK(mcpwm_timer_set_period(timerA, new_periodo));
    ESP_ERROR_CHECK(mcpwm_timer_set_period(timerB, new_periodo));
    ESP_ERROR_CHECK(mcpwm_timer_set_period(timerC, new_periodo));

    debug = 416;

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpA, debug));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpB, lut[(index + LUT_SIZE / 3) % LUT_SIZE]));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpC, lut[(index + 2*LUT_SIZE / 3) % LUT_SIZE]));
    

    return false;
}


// PWM cuadrado con dead-time entre generadores
static void setup_square(mcpwm_gen_handle_t genH, mcpwm_gen_handle_t genL, mcpwm_cmpr_handle_t cmp, uint32_t deadtime_ticks)
{
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(genH, MCPWM_GEN_TIMER_EVENT_ACTION(
        MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(genH, MCPWM_GEN_COMPARE_EVENT_ACTION(
        MCPWM_TIMER_DIRECTION_UP, cmp, MCPWM_GEN_ACTION_LOW)));

    mcpwm_dead_time_config_t dt_config = {
        .posedge_delay_ticks = deadtime_ticks / 2,
        .negedge_delay_ticks = 0,
    };
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(genH, genH, &dt_config));

    dt_config = (mcpwm_dead_time_config_t){
        .posedge_delay_ticks = 0,
        .negedge_delay_ticks = deadtime_ticks / 2,
        .flags.invert_output = true,
    };
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(genH, genL, &dt_config));
}

void app_main(void)
{
    // 1. Configurar GPIO 5 como salida para generar el pulso
    gpio_config_t gpio_out_cfg = {
        .pin_bit_mask = 1ULL << SYNC_GPIO_OUTPUT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_out_cfg);
    gpio_set_level(SYNC_GPIO_OUTPUT, 0);



    gpio_config_t gpio_out_cfg2 = {
        .pin_bit_mask = 1ULL << Test_GPIO_OUTPUT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_out_cfg2);
    gpio_set_level(Test_GPIO_OUTPUT, 0);

    // === 2. Crear los 3 timers ===
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RES_HZ,
        .period_ticks = PWM_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timerA));
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timerB));
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timerC));

    // === 3. Crear fuente de sincronización GPIO (GPIO 4) ===
    mcpwm_sync_handle_t gpio_sync = NULL;
    mcpwm_gpio_sync_src_config_t sync_cfg = {
        .group_id = 0,
        .gpio_num = SYNC_GPIO_INPUT,
        .flags.active_neg = false, // flanco ascendente
    };
    ESP_ERROR_CHECK(mcpwm_new_gpio_sync_src(&sync_cfg, &gpio_sync));

    // === 4. Asignar fuente de sync a los timers ===
    mcpwm_timer_sync_phase_config_t phase_A = {
        .sync_src = gpio_sync,
        .count_value = 0,
        .direction = MCPWM_TIMER_DIRECTION_UP,
    };
    mcpwm_timer_sync_phase_config_t phase_B = {
        .sync_src = gpio_sync,
        .count_value = PWM_PERIOD_TICKS * 0.33,
        .direction = MCPWM_TIMER_DIRECTION_UP,
    };
    mcpwm_timer_sync_phase_config_t phase_C = {
        .sync_src = gpio_sync,
        .count_value = PWM_PERIOD_TICKS * 0.66,
        .direction = MCPWM_TIMER_DIRECTION_UP,
    };

    ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(timerA, &phase_A));
    ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(timerB, &phase_B));
    ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(timerC, &phase_C));

    // === 5. Crear operadores y conectarlos ===
    mcpwm_oper_handle_t opA, opB, opC;
    mcpwm_operator_config_t op_cfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&op_cfg, &opA));
    ESP_ERROR_CHECK(mcpwm_new_operator(&op_cfg, &opB));
    ESP_ERROR_CHECK(mcpwm_new_operator(&op_cfg, &opC));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(opA, timerA));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(opB, timerB));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(opC, timerC));

    // === 6. Crear comparadores al 50% de duty ===
    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(opA, &cmp_cfg, &cmpA));
    ESP_ERROR_CHECK(mcpwm_new_comparator(opB, &cmp_cfg, &cmpB));
    ESP_ERROR_CHECK(mcpwm_new_comparator(opC, &cmp_cfg, &cmpC));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpA, PWM_PERIOD_TICKS / 2));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpB, PWM_PERIOD_TICKS / 2));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpC, PWM_PERIOD_TICKS / 2));

    // === 7. Crear generadores para las fases ===
    mcpwm_gen_handle_t genA_p, genA_n, genB_p, genB_n, genC_p, genC_n;
    mcpwm_generator_config_t gen_cfg = {};

    gen_cfg.gen_gpio_num = PHASE_A_P;
    ESP_ERROR_CHECK(mcpwm_new_generator(opA, &gen_cfg, &genA_p));
    gen_cfg.gen_gpio_num = PHASE_A_N;
    ESP_ERROR_CHECK(mcpwm_new_generator(opA, &gen_cfg, &genA_n));

    gen_cfg.gen_gpio_num = PHASE_B_P;
    ESP_ERROR_CHECK(mcpwm_new_generator(opB, &gen_cfg, &genB_p));
    gen_cfg.gen_gpio_num = PHASE_B_N;
    ESP_ERROR_CHECK(mcpwm_new_generator(opB, &gen_cfg, &genB_n));

    gen_cfg.gen_gpio_num = PHASE_C_P;
    ESP_ERROR_CHECK(mcpwm_new_generator(opC, &gen_cfg, &genC_p));
    gen_cfg.gen_gpio_num = PHASE_C_N;
    ESP_ERROR_CHECK(mcpwm_new_generator(opC, &gen_cfg, &genC_n));

    // === 8. Configurar PWM cuadrado con dead-time ===
    setup_square(genA_p, genA_n, cmpA, DEADTIME_TICKS);
    setup_square(genB_p, genB_n, cmpB, DEADTIME_TICKS);
    setup_square(genC_p, genC_n, cmpC, DEADTIME_TICKS);
    
    mcpwm_timer_event_callbacks_t cb ={
        .on_empty = isr_update_pwm_cb,
    };

    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timerA, &cb, NULL));

    
    //Poblar la LUT con valores senoidal de duty cycle en float
    uint8_t next_lut = (active_lut_index + 1) % LUT_COUNT;

    for (int i = 0; i < LUT_SIZE; i++) {
        float angle = 2.0f * M_PI * i / LUT_SIZE;
        float val = 0.5f + 0.5f * sinf(angle);  // entre 0 y 1
        ESP_LOGI(TAG, "ANDOOO DEBUGUEANDOOO = %u", (uint16_t)(val * periodo));
        lut_buffers[next_lut][i] = (uint16_t)(val * periodo);
    }
    lut_buffers[LUT_COUNT][next_lut] = PWM_RES_HZ / (F_MOTOR * LUT_SIZE);; // Guardar el período en el último elemento del LUT
    active_lut_index = next_lut;  // atómico

    // === 9. Habilitar y arrancar timers ===
    ESP_ERROR_CHECK(mcpwm_timer_enable(timerA));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timerB));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timerC));

    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timerA, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timerB, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timerC, MCPWM_TIMER_START_NO_STOP));

    ets_delay_us(10); // Espera mínima para asegurar que timers estén listos

    // === 10. Disparar el pulso de sincronización en GPIO 5 ===
    ESP_LOGI(TAG, "Enviando pulso en GPIO 5 conectado a GPIO 4");
    gpio_set_level(SYNC_GPIO_OUTPUT, 1);
    ets_delay_us(2);  // Pulso de 2 µs
    gpio_set_level(SYNC_GPIO_OUTPUT, 0);

    uint16_t timer_i = 0;
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(20));


        timer_i = (timer_i + 1) % (10 * F_MOTOR);

        periodo = PWM_RES_HZ / ((F_MOTOR_MIN + timer_i) * LUT_SIZE);

        next_lut = (active_lut_index + 1) % LUT_COUNT;

        for (int i = 0; i < LUT_SIZE; i++) {
            float angle = 2.0f * M_PI * i / LUT_SIZE;
            float val = 0.5f + 0.5f * sinf(angle);  // entre 0 y 1
            lut_buffers[next_lut][i] = (uint16_t)(val * periodo);
        }
        lut_buffers[LUT_COUNT][next_lut] = periodo; // Guardar el período en el último elemento del LUT
        active_lut_index = next_lut;  // atómico

    }
}
