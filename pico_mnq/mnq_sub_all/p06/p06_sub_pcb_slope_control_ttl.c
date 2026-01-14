#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>

/*
P06 motor_slope_control ver

sw 상태
올라갈 때 위(상) sw가 눌림 (mnq 올라온 상태)
내려갈 때 아래(하) sw가 눌림 (mnq 내려간 상태)

Direction: HIGH=UP, LOW=DOWN
*/

// ------------ pin set ------------
// LED (MNQ state up = HIGH, MNQ state down = LOW)
#define LED             PICO_DEFAULT_LED_PIN

// input
#define DETECT_1        3   // body
#define DETECT_2        4   // head
#define DETECT_3        5   // body

// ACT
#define MNQ_DIR         10  // DIR
#define MNQ_PWM         11   // PWM
#define MNQ_HIGH        1
#define MNQ_LOW         0

// limit sw (normal state = HIGH, press state = LOW)
#define LIMIT_SW_UNDER  17  // under → 내려갈 때 눌리면 정지
#define LIMIT_SW_TOP    18  // top   → 올라갈 때 눌리면 정지

// output (to main pcb)
#define HIT_1           12   // DETECT_2용 출력 head
#define HIT_2           13   // DETECT_1용 출력 body
#define HIT_3           14   // DETECT_3용 출력 body

#define UART_TX_PIN_NUMBER              (8)
#define UART_RX_PIN_NUMBER              (9)
#define Unit_K                          (1000)
#define Unit_M                          (1000000)

#define UART_ID                         (uart1)
#define BAUD_RATE                       (115200)
#define DATA_BITS                       (8)
#define STOP_BITS                       (1)
#define PARITY                          (UART_PARITY_NONE)


#define UART_DATA_LENGTH                (3)
#define UART_START_FLAG                 (0xff)

// pico clk
#define PICO_SYS_CLK_kHz            (125000)            // 125000 kHz
#define PICO_SYS_CLK                (125000000)         // 125 MHz
#define MAX_PWM               255

// 0→255 : 1ms에 5씩 증가 → 약 51ms
#define PWM_UP_STEP    5          // per 1 ms

// 255→150 : 1ms에 1씩 감소 → 105ms
#define PWM_DOWN_STEP  1          // per 1 ms (toward 150)

// 150→0 : 1ms에 3씩 감소 → 약 50ms
#define PWM_BRAKE_STEP      3          // per 1 ms

#define LOW_PWM    150        // 150/255 move

// 풀파워 유지 시간 (1000ms = 1s)
#define FULL_POWER_MS_DOWN  800    // 내려가기 까지 약 1.2초 소요
#define FULL_POWER_MS_UP    800    // 올라가기 까지 약 1.7초 소요

// MNQ Up,Down 후 2.5초 대기
#define HOLD_MS             2500

const uint32_t TIMEOUT_DOWN_MS = 3000;
const uint32_t TIMEOUT_UP_MS   = 3000;

// ------------ interrupt flag ------------
static volatile bool mnq_interrupt_flag = false;
static volatile bool detect1_rise = false;
static volatile bool detect2_rise = false;
static volatile bool detect3_rise = false;

// ------------ body counut (DETECT_1 or DETECT_3) ------------
static int body_shot_count = 0;

// ------------ limit sw set ------------
// static bool up_stop = true;
// static bool down_stop = false;

// ------------ PWM set ------------
static uint slice_num;

// ------------ HIT out state (not use) ------------
static bool hit1_on = false;
static bool hit2_on = false;
static bool hit3_on = false;

static int hit_s1 = 0;
static int hit_s2 = 0;
static int hit_s3 = 0;

static uint8_t Receive_Data[UART_DATA_LENGTH]  = {0, 0, 0};
volatile static uint        Receive_Done                    = 0;
volatile static uint        Receive_Data_Count              = 0;
volatile static uint8_t     Receive_Char                    = 0;
volatile static uint        Frame_Count                     = 0;
static char buffer[128];

static absolute_time_t last_uart_time;
static const uint32_t check_uart_time_out = 2000000;

static int pre_rec = -1;
static uint8_t last_cmd = 0;

void ConfigureUart(void);
void UART_Rx_Handler(void);
void tx_uart_data(int hit_s1, int hit_s2, int hit_s3);

static void gpio_setup(void);
static void StartSignal(void);
static inline uint32_t now_ms(void);
//static void lmit_sw_set(uint32_t now);
static void acting_mnq(int mnq_move);
static void mnq_manual_move(int mnq_manual);
static void hits_clear(void);
static void mnq_boot_home_once(void);

static bool Readsignal_detect_1(void);
static bool Readsignal_detect_2(void);
static bool Readsignal_detect_3(void);

// ------------ main ------------
int main() {
    stdio_init_all();
    sleep_ms(10);

    set_sys_clock_khz(PICO_SYS_CLK_kHz, true);
    busy_wait_ms(100);

    gpio_setup();
    sleep_ms(10);

    ConfigureUart();
    sleep_ms(10);

    StartSignal();
    sleep_ms(10);

    mnq_boot_home_once();   // 추가 (부팅 1회성 홈)
    sleep_ms(10);

    body_shot_count = 0;
    hits_clear();

    while (true) {
        uint32_t now = now_ms();
        tight_loop_contents();

        if (mnq_interrupt_flag){
            mnq_interrupt_flag = false;
            if (Readsignal_detect_1()){
                gpio_put(LED, 1);
                // gpio_put(HIT_1, 1);
                // gpio_put(HIT_2, 0);
                // gpio_put(HIT_3, 0);
                hit_s1 = 1;
                hit_s2 = 0;
                hit_s3 = 0;
                tx_uart_data(hit_s1, hit_s2, hit_s3);
                sleep_ms(10);
                gpio_put(LED, 0);
                hits_clear();
                acting_mnq(2);
                tx_uart_data(0, 0, 0);
            }
            else if (Readsignal_detect_2()){
                //body_shot_count += 1;
                gpio_put(LED, 1);
                // gpio_put(HIT_1, 0);
                // gpio_put(HIT_2, 1);
                // gpio_put(HIT_3, 0);
                hit_s1 = 0;
                hit_s2 = 1;
                hit_s3 = 0;
                tx_uart_data(hit_s1, hit_s2, hit_s3);
                sleep_ms(10);
                gpio_put(LED, 0);
                hits_clear();
                acting_mnq(2);
                tx_uart_data(0, 0, 0);
            }
            else if (Readsignal_detect_3()){
                //body_shot_count += 1;
                gpio_put(LED, 1);
                // gpio_put(HIT_1, 0);
                // gpio_put(HIT_2, 0);
                // gpio_put(HIT_3, 1);
                hit_s1 = 0;
                hit_s2 = 0;
                hit_s3 = 1;
                tx_uart_data(hit_s1, hit_s2, hit_s3);
                sleep_ms(10);
                gpio_put(LED, 0);
                hits_clear();
                acting_mnq(2);
                tx_uart_data(0, 0, 0);
            }
        }

        if (Receive_Done > 0) {
            Receive_Done = 0;
            // 첫 번째 데이터 행렬 값이 0 이면 신호 받지 말고 대기 및 tx_uart_data(0, 0, 0)만 보내기
            if (Receive_Data[0] == 0){
                tx_uart_data(0, 0, 0);
                continue;
            }
            else {
                uint8_t mode = Receive_Data[1];  // 0=auto, 1=manual
                uint8_t cmd = Receive_Data[2];
                // 두 번째 데이터 행렬 값이 0 이면 자동(탄 감지), 1이면 수동(탄 감지 안함)
                if (mode != pre_rec) {
                    if (mode == 0) {
                        // 자동: IRQ ON
                        gpio_set_irq_enabled(DETECT_1, GPIO_IRQ_EDGE_RISE, true);
                        gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, true);
                        gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, true);
                    } else {
                        // 수동: IRQ OFF
                        gpio_set_irq_enabled(DETECT_1, GPIO_IRQ_EDGE_RISE, false);
                        gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, false);
                        gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, false);
                    }
                    mnq_interrupt_flag = false;
                    detect1_rise = detect2_rise = detect3_rise = false;

                    last_cmd = 0;
                    pre_rec = mode;
                }
                if (mode == 1){
                    if (cmd != last_cmd) {
                        if (cmd == 1) mnq_manual_move(1);
                        else if (cmd == 2) mnq_manual_move(2);
                        last_cmd = cmd;
                    }
                }
            }
        }

        busy_wait_ms(1);
    }

    return 0;
}

// ------------ util : now time(ms) ------------
static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

// ------------ start signal ------------
void StartSignal(void)
{
    gpio_put(LED, 1);
    sleep_ms(3000);
    gpio_put(LED, 0);
    sleep_ms(1000);
}

// ------------ GPIO IRQ callback : DETECT_1/2/3 상승엣지 감지 ------------
static void gpio_irq_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        if (gpio == DETECT_1) {
            detect1_rise = true;
        } else if (gpio == DETECT_2) {
            detect2_rise = true;
        } else if (gpio == DETECT_3) {
            detect3_rise = true;
        }
        mnq_interrupt_flag = true;
    }
}

// void UART_Rx_Handler(void)
// {
//     irq_set_enabled(UART1_IRQ, false);

//     if (uart_is_readable_within_us(UART_ID, 100))
//     {
//         Receive_Char = uart_getc(UART_ID);
//     }

//     if (Receive_Char == UART_START_FLAG)
//     {
//         Receive_Data_Count = 0x0;
//     }
//     else
//     {
//         if (Receive_Data_Count < UART_DATA_LENGTH)
//         {
//             Receive_Data[Receive_Data_Count] = Receive_Char;

//             Receive_Data_Count++;

//             if (Receive_Data_Count == UART_DATA_LENGTH)
//             {
//                 Receive_Done = 1;
//             }
//         }
//     }

//     irq_set_enabled(UART1_IRQ, true);
// }

void UART_Rx_Handler(void)
{
    irq_set_enabled(UART1_IRQ, false);

    if (!uart_is_readable(UART_ID)) {
        irq_set_enabled(UART1_IRQ, true);
        return;
    }

    Receive_Char = uart_getc(UART_ID);

    if (Receive_Char == UART_START_FLAG) {
        Receive_Data_Count = 0;
    } else if (Receive_Data_Count < UART_DATA_LENGTH) {
        Receive_Data[Receive_Data_Count++] = Receive_Char;
        if (Receive_Data_Count == UART_DATA_LENGTH) {
            Receive_Done = 1;
        }
    }

    irq_set_enabled(UART1_IRQ, true);
}


void ConfigureUart(void)
{
    gpio_set_function(UART_TX_PIN_NUMBER, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_NUMBER, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);

    uart_set_hw_flow(UART_ID, false, false);

    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    uart_set_fifo_enabled(UART_ID, false);

    irq_set_exclusive_handler(UART1_IRQ, UART_Rx_Handler);
    irq_set_enabled(UART1_IRQ, true);

    uart_set_irq_enables(UART_ID, true, false);
}

void tx_uart_data(int hit_s1, int hit_s2, int hit_s3){
    sprintf(buffer, "%d,%d,%d\n", hit_s1, hit_s2, hit_s3);
    uart_puts(UART_ID, buffer);
}

// ------------ GPIO set ------------
static void gpio_setup(void) {
    // LED
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    gpio_init(DETECT_1);
    gpio_set_dir(DETECT_1, GPIO_IN);
    gpio_pull_down(DETECT_1);
    gpio_set_irq_enabled_with_callback(DETECT_1, GPIO_IRQ_EDGE_RISE, true, &gpio_irq_callback);

    gpio_init(DETECT_2);
    gpio_set_dir(DETECT_2, GPIO_IN);
    gpio_pull_down(DETECT_2);
    gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, true);

    gpio_init(DETECT_3);
    gpio_set_dir(DETECT_3, GPIO_IN);
    gpio_pull_down(DETECT_3);
    gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, true);

    gpio_init(MNQ_DIR);
    gpio_set_dir(MNQ_DIR, GPIO_OUT);
    gpio_put(MNQ_DIR, 0);

    // PWM SET
    const float mnq_pwm_freq_hz = 16000.0f;     // 16kHz

    gpio_set_function(MNQ_PWM, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(MNQ_PWM);

    pwm_set_wrap(slice_num, 255);

    float div = (float)PICO_SYS_CLK / ((255 + 1.0f) * mnq_pwm_freq_hz);
    if (div < 1.0f)   div = 1.0f;
    if (div > 255.0f) div = 255.0f;

    pwm_set_clkdiv(slice_num, div);

    pwm_set_gpio_level(MNQ_PWM, 0);
    pwm_set_enabled(slice_num, true);

    gpio_init(LIMIT_SW_UNDER);
    gpio_set_dir(LIMIT_SW_UNDER, GPIO_IN);
    gpio_pull_up(LIMIT_SW_UNDER);

    gpio_init(LIMIT_SW_TOP);
    gpio_set_dir(LIMIT_SW_TOP, GPIO_IN);
    gpio_pull_up(LIMIT_SW_TOP);

    // HIT output
    gpio_init(HIT_1);
    gpio_set_dir(HIT_1, GPIO_OUT);
    gpio_put(HIT_1, 0);

    gpio_init(HIT_2);
    gpio_set_dir(HIT_2, GPIO_OUT);
    gpio_put(HIT_2, 0);

    gpio_init(HIT_3);
    gpio_set_dir(HIT_3, GPIO_OUT);
    gpio_put(HIT_3, 0);
}

// DETECT_1 신호 high 신호인지 확인 1ms 당 5번
static bool Readsignal_detect_1(void){
    if (gpio_get(DETECT_1) == 0) return false;

    for(int i = 0; i < 5; i++){
        if (gpio_get(DETECT_1) == 0) return false;
        sleep_ms(1);
    }
    return true;
}

// DETECT_2 신호 high 신호인지 확인 1ms 당 5번
static bool Readsignal_detect_2(void){
    if (gpio_get(DETECT_2) == 0) return false;

    for(int i = 0; i < 5; i++){
        if (gpio_get(DETECT_2) == 0) return false;
        sleep_ms(1);
    }
    return true;
}

// DETECT_3 신호 high 신호인지 확인 1ms 당 5번
static bool Readsignal_detect_3(void){
    if (gpio_get(DETECT_3) == 0) return false;

    for(int i = 0; i < 5; i++){
        if (gpio_get(DETECT_3) == 0) return false;
        sleep_ms(1);
    }
    return true;
}

static void mnq_manual_move(int mnq_manual){
    int pwm = 0;
    if (mnq_manual == 1){
        pwm = 0;
        gpio_put(MNQ_DIR, MNQ_LOW);
        pwm_set_gpio_level(MNQ_PWM, 0);

        // 1) 0->255 램프업 (1ms에 5씩)
        while (pwm < MAX_PWM) {
            if (gpio_get(LIMIT_SW_UNDER) == 0) break;     // limit sw 눌렸으면 일단 정지 (오작동 가능성)

            pwm += PWM_UP_STEP;
            if (pwm > MAX_PWM) pwm = MAX_PWM;
            pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);

            busy_wait_ms(1);
        }

        // 2) 풀파워 유지 (0.8s)
        uint32_t t0 = now_ms();
        while ((now_ms() - t0) < FULL_POWER_MS_DOWN) {
            if (gpio_get(LIMIT_SW_UNDER) == 0) break;
            busy_wait_ms(1);
        }

        // 2-추가) 255 -> 150 (1ms에 1씩)
        while (pwm > LOW_PWM) {
            if (gpio_get(LIMIT_SW_UNDER) == 0) break;

            pwm -= PWM_DOWN_STEP;
            if (pwm < LOW_PWM) pwm = LOW_PWM;
            pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);

            busy_wait_ms(1);
        }

        // 2-추가) 150 유지 0.2초 유지
        t0 = now_ms();
        while ((now_ms() - t0) < 200) {
            if (gpio_get(LIMIT_SW_UNDER) == 0) {   // 눌리면 LOW
                pwm = 0;
                pwm_set_gpio_level(MNQ_PWM, 0);
                break;
            }
            busy_wait_ms(1);
        }

        // 4) 0.2초 이후 150→0 : 1ms에 3씩 감소 (브레이크 중 리밋 눌리면 즉시 정지)
        while (pwm > 0) {
            if (gpio_get(LIMIT_SW_UNDER) == 0) {
                pwm = 0;
                pwm_set_gpio_level(MNQ_PWM, 0);
                break;
            }

            pwm -= PWM_BRAKE_STEP;          // 3씩 감소
            if (pwm < 0) pwm = 0;
            pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);
            busy_wait_ms(1);
        }
        pwm_set_gpio_level(MNQ_PWM, 0);
    } 
    else if (mnq_manual == 2){
        pwm = 0;
        gpio_put(MNQ_DIR, MNQ_HIGH);
        pwm_set_gpio_level(MNQ_PWM, 0);

        // 1) 0->255 램프업
        while (pwm < MAX_PWM) {
            if (gpio_get(LIMIT_SW_TOP) == 0) break;     // limit sw 눌렸으면 일단 정지 (오작동 가능성)

            pwm += PWM_UP_STEP;
            if (pwm > MAX_PWM) pwm = MAX_PWM;
            pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);

            busy_wait_ms(1);
        }

        // 2) 풀파워 유지 (0.8s)
        t0 = now_ms();
        while ((now_ms() - t0) < FULL_POWER_MS_UP) {
            if (gpio_get(LIMIT_SW_TOP) == 0) break;
            busy_wait_ms(1);
        }

        // 2-추가) 255 -> 150
        while (pwm > LOW_PWM) {
            if (gpio_get(LIMIT_SW_TOP) == 0) break;

            pwm -= PWM_DOWN_STEP;
            if (pwm < LOW_PWM) pwm = LOW_PWM;
            pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);

            busy_wait_ms(1);
        }

        // 3-1)150으로 약 0.2초 유지
        t0 = now_ms();
        while ((now_ms() - t0) < 200) {
            if (gpio_get(LIMIT_SW_TOP) == 0) {     // 눌리면 LOW
                pwm = 0;
                pwm_set_gpio_level(MNQ_PWM, 0);
                break;
            }
            busy_wait_ms(1);
        }
        // 4) 0.2초 이후 150→0 : 1ms에 3씩 감소 (브레이크 중 리밋 눌리면 즉시 정지)
        while (pwm > 0) {
            if (gpio_get(LIMIT_SW_TOP) == 0) {
                pwm = 0;
                pwm_set_gpio_level(MNQ_PWM, 0);
                break;
            }

            pwm -= PWM_BRAKE_STEP;
            if (pwm < 0) pwm = 0;
            pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);
            busy_wait_ms(1);
        }

        pwm_set_gpio_level(MNQ_PWM, 0);
    }
}

static void acting_mnq(int mnq_move)
{
    // mnq_move: 2=headshot 즉시 down, 1=bodyshot 2회 이상이면 down, 0=무시
    bool start = false;
    if (mnq_move == 2) {
        start = true;
    }
    else if (mnq_move ==1 && body_shot_count >= 2){
        start = true;
    }
    else {
        return;
    }

    if (start) {
        // ---- 모터 움직이는 동안 센서 신호 완전 무시: IRQ OFF ----
        gpio_set_irq_enabled(DETECT_1, GPIO_IRQ_EDGE_RISE, false);
        gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, false);
        gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, false);
        mnq_interrupt_flag = false;
        detect1_rise = detect2_rise = detect3_rise = false;

        int pwm = 0;

        // =========================
        // DOWN 시퀀스
        // =========================
        gpio_put(MNQ_DIR, MNQ_LOW);
        pwm_set_gpio_level(MNQ_PWM, 0);

        // 1) 0->255 램프업 (1ms에 5씩)
        while (pwm < MAX_PWM) {
            if (gpio_get(LIMIT_SW_UNDER) == 0) break;     // limit sw 눌렸으면 일단 정지 (오작동 가능성)

            pwm += PWM_UP_STEP;
            if (pwm > MAX_PWM) pwm = MAX_PWM;
            pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);

            busy_wait_ms(1);
        }

        // 2) 풀파워 유지 (0.8s)
        uint32_t t0 = now_ms();
        while ((now_ms() - t0) < FULL_POWER_MS_DOWN) {
            if (gpio_get(LIMIT_SW_UNDER) == 0) break;
            busy_wait_ms(1);
        }

        // 2-추가) 255 -> 150 (1ms에 1씩)
        while (pwm > LOW_PWM) {
            if (gpio_get(LIMIT_SW_UNDER) == 0) break;

            pwm -= PWM_DOWN_STEP;
            if (pwm < LOW_PWM) pwm = LOW_PWM;
            pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);

            busy_wait_ms(1);
        }

        // 2-추가) 150 유지 0.2초 유지
        t0 = now_ms();
        while ((now_ms() - t0) < 200) {
            if (gpio_get(LIMIT_SW_UNDER) == 0) {   // 눌리면 LOW
                pwm = 0;
                pwm_set_gpio_level(MNQ_PWM, 0);
                break;
            }
            busy_wait_ms(1);
        }

        // 4) 0.2초 이후 150→0 : 1ms에 3씩 감소 (브레이크 중 리밋 눌리면 즉시 정지)
        while (pwm > 0) {
            if (gpio_get(LIMIT_SW_UNDER) == 0) {
                pwm = 0;
                pwm_set_gpio_level(MNQ_PWM, 0);
                break;
            }

            pwm -= PWM_BRAKE_STEP;          // 3씩 감소
            if (pwm < 0) pwm = 0;
            pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);
            busy_wait_ms(1);
        }
        pwm_set_gpio_level(MNQ_PWM, 0);

        // 4) 아래에서 HOLD
        busy_wait_ms(HOLD_MS);

        // =========================
        // UP 시퀀스
        // =========================
        gpio_put(MNQ_DIR, MNQ_HIGH);
        pwm_set_gpio_level(MNQ_PWM, 0);
        pwm = 0;

        // 1) 0->255 램프업
        while (pwm < MAX_PWM) {
            if (gpio_get(LIMIT_SW_TOP) == 0) break;     // limit sw 눌렸으면 일단 정지 (오작동 가능성)

            pwm += PWM_UP_STEP;
            if (pwm > MAX_PWM) pwm = MAX_PWM;
            pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);

            busy_wait_ms(1);
        }

        // 2) 풀파워 유지 (0.8s)
        t0 = now_ms();
        while ((now_ms() - t0) < FULL_POWER_MS_UP) {
            if (gpio_get(LIMIT_SW_TOP) == 0) break;
            busy_wait_ms(1);
        }

        // 2-추가) 255 -> 150
        while (pwm > LOW_PWM) {
            if (gpio_get(LIMIT_SW_TOP) == 0) break;

            pwm -= PWM_DOWN_STEP;
            if (pwm < LOW_PWM) pwm = LOW_PWM;
            pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);

            busy_wait_ms(1);
        }

        // 3-1)150으로 약 0.2초 유지
        t0 = now_ms();
        while ((now_ms() - t0) < 200) {
            if (gpio_get(LIMIT_SW_TOP) == 0) {     // 눌리면 LOW
                pwm = 0;
                pwm_set_gpio_level(MNQ_PWM, 0);
                break;
            }
            busy_wait_ms(1);
        }
        // 4) 0.2초 이후 150→0 : 1ms에 3씩 감소 (브레이크 중 리밋 눌리면 즉시 정지)
        while (pwm > 0) {
            if (gpio_get(LIMIT_SW_TOP) == 0) {
                pwm = 0;
                pwm_set_gpio_level(MNQ_PWM, 0);
                break;
            }

            pwm -= PWM_BRAKE_STEP;
            if (pwm < 0) pwm = 0;
            pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);
            busy_wait_ms(1);
        }

        pwm_set_gpio_level(MNQ_PWM, 0);

        // 5) 위에서 HOLD
        busy_wait_ms(HOLD_MS);

        // ---- 동작 끝: 센서 IRQ 다시 ON ----
        mnq_interrupt_flag = false;
        detect1_rise = detect2_rise = detect3_rise = false;

        gpio_set_irq_enabled(DETECT_1, GPIO_IRQ_EDGE_RISE, true);
        gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, true);
        gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, true);

        // 한 사이클 끝났으니 리셋
        body_shot_count = 0;
    }
    else {
        return;
    }
    start = false;
}

static void mnq_boot_home_once(void)
{
    // 부팅 직후 1회: 아래(under_sw 눌림)면 올리기, 아니면 무시
    /*
    부팅 직후 위의 선언된 변수들 'up_stop', 'down_stop'의 상태를 ture인지 false인지 지정해주는 역할
    static bool up_stop = false;
    static bool down_stop = false;
    */

    // under_sw가 눌려 있지 않음, tow_sw가 눌려 있음 = MNQ는 올라와 있으니 넘어가기
    if (gpio_get(LIMIT_SW_TOP) == 0) {
        pwm_set_gpio_level(MNQ_PWM, 0);
        return;
    } 

    // ---- 홈 동작 중 센서 IRQ OFF ----
    gpio_set_irq_enabled(DETECT_1, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, false);
    mnq_interrupt_flag = false;
    detect1_rise = detect2_rise = detect3_rise = false;

    // ---- UP 시작 ----
    gpio_put(MNQ_DIR, MNQ_HIGH);
    pwm_set_gpio_level(MNQ_PWM, 0);

    int pwm = 0;

    // 1) 0->255 램프업
    while (pwm < MAX_PWM) {
        if (gpio_get(LIMIT_SW_TOP) == 0) break;

        pwm += PWM_UP_STEP;
        if (pwm > MAX_PWM) pwm = MAX_PWM;
        pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);

        busy_wait_ms(1);
    }

    // 2) 풀파워 유지
    uint32_t t0 = now_ms();
    while ((now_ms() - t0) < FULL_POWER_MS_UP) {
        if (gpio_get(LIMIT_SW_TOP) == 0) break;
        busy_wait_ms(1);
    }

    // 3) 255 -> 150
    while (pwm > LOW_PWM) {
        if (gpio_get(LIMIT_SW_TOP) == 0) break;

        pwm -= PWM_DOWN_STEP;
        if (pwm < LOW_PWM) pwm = LOW_PWM;
        pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);

        busy_wait_ms(1);
    }

    // 3-1)150으로 약 0.2초 유지
    t0 = now_ms();
    while ((now_ms() - t0) < 200) {
        if (gpio_get(LIMIT_SW_TOP) == 0) {     // 눌리면 LOW
            pwm = 0;
            pwm_set_gpio_level(MNQ_PWM, 0);
            break;
        }
        busy_wait_ms(1);
    }
    // 4) 0.2초 이후 150→0 : 1ms에 3씩 감소 (브레이크 중 리밋 눌리면 즉시 정지)
    while (pwm > 0) {
        if (gpio_get(LIMIT_SW_TOP) == 0) {
            pwm = 0;
            pwm_set_gpio_level(MNQ_PWM, 0);
            break;
        }

        pwm -= PWM_BRAKE_STEP;
        if (pwm < 0) pwm = 0;
        pwm_set_gpio_level(MNQ_PWM, (uint16_t)pwm);
        busy_wait_ms(1);
    }
    pwm_set_gpio_level(MNQ_PWM, 0);

    // (선택) 기구 안정화 대기 필요하면 100~300ms 정도
    // busy_wait_ms(200);

    // ---- 홈 끝: 센서 IRQ ON ----
    mnq_interrupt_flag = false;
    detect1_rise = detect2_rise = detect3_rise = false;

    gpio_set_irq_enabled(DETECT_1, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, true);
}

// ------------ HIT clear------------
static void hits_clear(void) {
    gpio_put(HIT_1, 0);
    gpio_put(HIT_2, 0);
    gpio_put(HIT_3, 0);
}