#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

//260415 최종 수정
//limit sw, act 위상 반전
//통신 프레임 깨져서 timeout 발생 시 정상 프레임을 다시 받을 경우 재연결 및 복구 추가

#define LED             PICO_DEFAULT_LED_PIN

// --------------------- MOTOR ---------------------
#define MNQ_DIR        13
#define MNQ_PWM        12

#define DIR_DOWN       0
#define DIR_UP         1

#define PWM_MAX_VAL    0
#define PWM_HOLD_VAL   150

static uint32_t motor_timer_start = 0;
static uint8_t prev_manual_cmd = 255;

static char limit_state = '0';
volatile int act = 0;
volatile int pwm = 255;

static int up_step    = 5;
static int down_step  = 5;

volatile bool up_wait = false;
static uint32_t auto_up_wait_start = 0;

// --------------------- Limit SW ---------------------
#define LIMIT_TOP              4  // 타겟 ACT 타워 기준 위에 있는 스위치 = 넘어갈 시 눌리면 정지            
#define LIMIT_UNDER            5  // 타겟 ACT 타워 기준 아래에 있는 스위치 = 올라올 시 눌리면 정지

// --------------------- SBC UART 1 ---------------------
#define UART_TX_PIN_SBC                 (8)
#define UART_RX_PIN_SBC                 (9)

#define UART_ID_SBC                     (uart1)
#define SBC_BAUD_RATE                   (115200)

#define UART_START_FLAG                 (0xff)
#define UART_DATA_LENGTH                (8)

static uint8_t Receive_Data[UART_DATA_LENGTH]  = {0, 0, 0, 0, 0, 0, 0, 0};
//static volatile uint8_t Receive_Data[UART_DATA_LENGTH] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile static uint        Receive_Done                    = 0;
volatile static uint        Receive_Data_Count              = 0;
volatile static uint8_t     Receive_Char                    = 0;
volatile static uint        Frame_Count                     = 0;

static bool control_signal = false;
static bool uart_recover_up = false;

static absolute_time_t last_uart_time;
static const uint32_t check_uart_time_out = 3000000;

// --------------------- Detect Sensor ---------------------
#define DETECT_1        19
#define DETECT_2        20
#define DETECT_3        21
#define GAIN            14

volatile bool mnq_interrupt_flag = false;
volatile bool mnq1_state = false;
volatile bool mnq2_state = false;
volatile bool mnq3_state = false;
static bool motor_active = false;

volatile bool first_channel_ban = false;
volatile bool second_channel_ban = false;
volatile bool third_channel_ban = false;

static int hit_channel = 0;
volatile int pending_hit_channel = 0;   // 감지 이벤트 보관
static int sending_channel = 0;         // SBC로 보낼 값

static uint8_t prev_mask = 0;

static uint8_t default_gain_pwm = 62;
static uint8_t current_gain_pwm = 62;

volatile uint8_t default_cri_set = 0;
volatile uint8_t cur_cri_set = 0;
volatile uint8_t cri_hit_count = 0;

volatile uint8_t default_non_cri_set = 0;
volatile uint8_t cur_non_cri_set = 0;
volatile uint8_t non_cri_hit_count = 0;

// --------------------- Common set ---------------------
#define DATA_BITS                       (8)
#define STOP_BITS                       (1)
#define PARITY                          (UART_PARITY_NONE)

static char buffer[64];

#define PICO_SYS_CLK_kHz                (125000)            // 125000 kHz
#define PICO_SYS_CLK                    (125000000)

// --------------------- Common function ---------------------
void ConfigureGpio_motor(void);
void ConfigureUart_sbc(void);
void ConfigureGpio(void);
static void gpio_irq_callback(uint gpio, uint32_t events);
void StartSignal(void);
void uart_timeout();

// --------------------- Motor function ---------------------
void acting_mnq_motor(int act);

void manual_up(void);
void manual_down(void);

// --------------------- SBC function ---------------------
void Rx_uart_SBC(void);
void Tx_uart_SBC(int sending_channel);

static void detect_channel_ban(void);
void gain_pwm_set(uint8_t gain_pwm);

void start_motor_timer();
bool check_motor_timer(uint32_t duration);

//--------------------- Main ---------------------
int main(){
    stdio_init_all();
    sleep_ms(10);

    set_sys_clock_khz(PICO_SYS_CLK_kHz, true);
    busy_wait_ms(100);

    ConfigureGpio_motor();
    sleep_ms(10);

    ConfigureUart_sbc();
    sleep_ms(10);

    ConfigureGpio();
    sleep_ms(10);

    StartSignal();
    last_uart_time = get_absolute_time();
    sleep_ms(10);

    while (true)
    {
        tight_loop_contents();  
        uart_timeout();

        acting_mnq_motor(0);

        // 구 버전
        // if (mnq_interrupt_flag)
        // {
        //     mnq_interrupt_flag = false;
        //     if (control_signal == true)
        //     {
        //         if (mnq1_state && !mnq2_state && !mnq3_state)   {
        //             hit_channel = 1;
        //         }
        //         else if(!mnq1_state && mnq2_state && !mnq3_state){
        //             hit_channel = 2;
        //         }
        //         else if (!mnq1_state && !mnq2_state && mnq3_state){
        //             hit_channel = 3;
        //         }
        //         else if (!mnq1_state && !mnq2_state && !mnq3_state){
        //             hit_channel = 0;
        //         }
        //     }
        // }

        /*
        Receive_Data[0] → 0 = 제어 off, 1 = on
        Receive_Data[1] → 0 = 자동(자동 모터 제어), 1 = 수동(사용자 직접 제어 및 UP), 2(DOWN)
        Receive_Data[2] → 0 = 1 채널 on, 1 = 1 채널 off
        Receive_Data[3] → 0 = 2 채널 on, 1 = 2 채널 off
        Receive_Data[4] → 0 = 3 채널 on, 1 = 3 채널 off
        Receive_Data[5] → 0 ~ 254 = Gain PWM 값 적용 (감지 센서 민감도) // 해당 버전은 사용 안함 넣어놓기만 할 것
        Receive_Data[6] → 0 ~ 9 치명상
        Receive_Data[7] → 0 ~ 9 비치명상
        */
        if (Receive_Done > 0){
            if (Receive_Data[0] == 0){
                control_signal = false;
                prev_manual_cmd = 255;

                pwm_set_gpio_level(MNQ_PWM, 255);
                pwm = 255;
                act = 0;
                motor_active = false;
                
                up_wait = false;
                limit_state = '0';
                auto_up_wait_start = 0;
                
                pending_hit_channel = 0;
                sending_channel = 0;

                cri_hit_count = 0;
                non_cri_hit_count = 0;
            }
            else {
                control_signal = true;

                // timeout 이후 첫 정상 프레임 복구 시 강제 UP
                if (uart_recover_up) {
                    //hit_channel = 0;
                    up_wait = false;
                    act = 0;
                    limit_state = '0';

                    // 이미 UP 끝 위치가 아니면 올림
                    if (gpio_get(LIMIT_UNDER) != 0) {
                        acting_mnq_motor(2);   // UP
                    }

                    uart_recover_up = false;
                }

                detect_channel_ban();
                if (Receive_Data[5] != current_gain_pwm) {
                    gain_pwm_set(Receive_Data[5]);
                }
                if (Receive_Data[6] != cur_cri_set) {
                    cur_cri_set = Receive_Data[6];
                }
                if (Receive_Data[7] != cur_non_cri_set) {
                    cur_non_cri_set = Receive_Data[7]; 
                }
                
                if (Receive_Data[1] == 0){
                    bool cri_ready = (cur_cri_set > 0) && (cri_hit_count >= cur_cri_set);
                    bool non_cri_ready = (cur_non_cri_set > 0) && (non_cri_hit_count >= cur_non_cri_set);

                    sending_channel = pending_hit_channel;

                    if ((sending_channel == 1 && first_channel_ban) ||
                        (sending_channel == 2 && second_channel_ban) ||
                        (sending_channel == 3 && third_channel_ban)) {
                        sending_channel = 0;
                        pending_hit_channel = 0;
                    }

                    if (sending_channel != 0 && act == 0 && !up_wait) {
                        if (cri_ready || non_cri_ready) {
                            acting_mnq_motor(1);   // DOWN
                            up_wait = true;
                            cri_hit_count = 0;
                            non_cri_hit_count = 0;
                            pending_hit_channel = 0;
                        }
                    }
                }
                else {
                    up_wait = false;
                    cri_hit_count = 0;
                    non_cri_hit_count = 0;
                    // 수동
                    if (Receive_Data[1] != prev_manual_cmd) {
                        if (Receive_Data[1] == 1){
                            acting_mnq_motor(2);    // UP
                        }
                        else if (Receive_Data[1] == 2){
                            acting_mnq_motor(1);    // DOWN
                        }
                        prev_manual_cmd = Receive_Data[1];
                    }
                }
            }
            Tx_uart_SBC(sending_channel);
            last_uart_time = get_absolute_time();
            Receive_Done = 0;
        }

        if (gpio_get(LIMIT_UNDER) == 0) {
            if (limit_state == 'u') {
                pwm_set_gpio_level(MNQ_PWM, 255);
                pwm = 255;
                motor_active = false;
                act = 0;
                limit_state = '0';
            }
        }
        else if (gpio_get(LIMIT_TOP) == 0) {
            if (limit_state == 'd') {
                pwm_set_gpio_level(MNQ_PWM, 255);
                pwm = 255;
                motor_active = false;
                act = 0;
                limit_state = '0';

                // 자동 DOWN이 끝난 직후 1.5초 대기 시작
                if (up_wait) {
                    auto_up_wait_start = to_ms_since_boot(get_absolute_time());
                }
            }
        }

        if (up_wait && act == 0) {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if ((now - auto_up_wait_start) >= 1500) {
                acting_mnq_motor(2);   // UP
                pending_hit_channel = 0;
                sending_channel = 0;
                //hit_channel = 0;
                up_wait = false;
            }
        }
        Frame_Count++;
        busy_wait_ms(1);
    }
    return 0;
}

// --------------------- Common set ---------------------
void ConfigureGpio(void)
{
    const float pwm_freq_hz = 5000.0f;

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    gpio_init(DETECT_1);
    gpio_set_dir(DETECT_1, GPIO_IN);
    gpio_pull_down(DETECT_1);
    gpio_set_irq_enabled_with_callback(DETECT_1, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true, &gpio_irq_callback);

    gpio_init(DETECT_2);
    gpio_set_dir(DETECT_2, GPIO_IN);
    gpio_pull_down(DETECT_2);
    gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true);

    gpio_init(DETECT_3);
    gpio_set_dir(DETECT_3, GPIO_IN);
    gpio_pull_down(DETECT_3);
    gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true);

    gpio_set_function(GAIN, GPIO_FUNC_PWM);
    uint gain_slice_num = pwm_gpio_to_slice_num(GAIN);

    pwm_set_wrap(gain_slice_num, 255);

    float div = (float)PICO_SYS_CLK / ((255 + 1.0f) * pwm_freq_hz);
    if (div < 1.0f) div = 1.0f;
    if (div > 255.0f) div = 255.0f;

    pwm_set_clkdiv(gain_slice_num, div);
    pwm_set_gpio_level(GAIN, 0);

    pwm_set_enabled(gain_slice_num, true);

    pwm_set_gpio_level(GAIN, default_gain_pwm); // default pwm 적용

    gpio_init(LIMIT_TOP);
    gpio_set_dir(LIMIT_TOP, GPIO_IN);
    gpio_pull_up(LIMIT_TOP);

    gpio_init(LIMIT_UNDER);
    gpio_set_dir(LIMIT_UNDER, GPIO_IN);
    gpio_pull_up(LIMIT_UNDER);
}

static void gpio_irq_callback(uint gpio, uint32_t events) {
    if (!control_signal) return;
    if (Receive_Data[1] != 0) return;   // 자동 모드에서만 count 사용

    // 이미 자동 동작 중/대기 중이면 새 hit 무시
    if (act != 0 || up_wait) return;

    if (gpio == DETECT_1 && gpio_get(DETECT_1)) {
        if (pending_hit_channel == 0) {
            pending_hit_channel = 1;
        }
        cri_hit_count += 1;
    }
    else if (gpio == DETECT_2 && gpio_get(DETECT_2)) {
        if (pending_hit_channel == 0) {
            pending_hit_channel = 2;
        }
        non_cri_hit_count += 1;
    }
    else if (gpio == DETECT_3 && gpio_get(DETECT_3)) {
        if (pending_hit_channel == 0) {
            pending_hit_channel = 3;
        }
        non_cri_hit_count += 1;
    }
}

void StartSignal(void)
{
    gpio_put(LED, 1);
    sleep_ms(3000);
    gpio_put(LED, 0);
}

// --------------------- Motor ---------------------
void ConfigureGpio_motor(void){
    gpio_init(MNQ_DIR);
    gpio_set_dir(MNQ_DIR, GPIO_OUT);
    gpio_pull_up(MNQ_DIR);
    gpio_put(MNQ_DIR, 1);

    const float pwm_freq_hz = 5000.0f;

    gpio_set_function(MNQ_PWM, GPIO_FUNC_PWM);

    uint mnq_pwm_slice_num = pwm_gpio_to_slice_num(MNQ_PWM);
    pwm_set_wrap(mnq_pwm_slice_num, 255);

    float div = (float)PICO_SYS_CLK / ((255 + 1.0f) * pwm_freq_hz);
    if (div < 1.0f) div = 1.0f;
    if (div > 255.0f) div = 255.0f;

    pwm_set_clkdiv(mnq_pwm_slice_num, div);

    pwm_set_gpio_level(MNQ_PWM, 255);   // 위상 반전으로 0이 아닌 255
    pwm_set_enabled(mnq_pwm_slice_num, true);
}

// --------------------- Motor Utility ---------------------
// 추가 및 수정 해야할 함수 내려 갈 때 pwm 위상 반전으로 0이 최고속도 255가 정지
void acting_mnq_motor(int cmd){
    if (cmd == 1 || cmd == 2) {
        if (cmd == 1) {
            gpio_put(MNQ_DIR, DIR_DOWN);
            limit_state = 'd';
        }
        else {
            gpio_put(MNQ_DIR, DIR_UP);
            limit_state = 'u';
        }

        pwm = 255;
        pwm_set_gpio_level(MNQ_PWM, pwm);
        act = 1;
        start_motor_timer();
        return;
    }

    if (act == 1) {
        if (check_motor_timer(5)) {
            pwm -= up_step;
            if (pwm < PWM_MAX_VAL) {
                pwm = PWM_MAX_VAL;
            }
            pwm_set_gpio_level(MNQ_PWM, pwm);

            if (pwm == PWM_MAX_VAL) {
                act = 2;
            }
            start_motor_timer();
        }
    }
    else if (act == 2) {
        if (check_motor_timer(800)) {
            act = 3;
            start_motor_timer();
        }
    }
    else if (act == 3) {
        if (check_motor_timer(5)) {
            pwm += down_step;
            if (pwm > PWM_HOLD_VAL) {
                pwm = PWM_HOLD_VAL;
            }
            pwm_set_gpio_level(MNQ_PWM, pwm);

            if (pwm == PWM_HOLD_VAL) {
                act = 4;
                motor_active = false;
            }
            else {
                start_motor_timer();
            }
        }
    }
}

void start_motor_timer()
{
    motor_active = true;
    motor_timer_start = to_ms_since_boot(get_absolute_time());
}

bool check_motor_timer(uint32_t duration)
{
    if (motor_active) 
    {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if ((current_time - motor_timer_start) >= duration)
        {
            motor_active = false;
            return true;
        }
    }
    return false;
}

// --------------------- SBC ---------------------
void ConfigureUart_sbc(void)
{
    gpio_set_function(UART_TX_PIN_SBC, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_SBC, GPIO_FUNC_UART);

    uart_init(UART_ID_SBC, SBC_BAUD_RATE);
    uart_set_hw_flow(UART_ID_SBC, false, false);
    uart_set_format(UART_ID_SBC, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID_SBC, false);
    
    irq_set_exclusive_handler(UART1_IRQ, Rx_uart_SBC);
    irq_set_enabled(UART1_IRQ, true);

    uart_set_irq_enables(UART_ID_SBC, true, false);
}

void Rx_uart_SBC(void)
{
    while (uart_is_readable(UART_ID_SBC))
    {
        uint8_t ch = uart_getc(UART_ID_SBC);

        if (Receive_Done)
        {
            continue;
        }

        if (ch == UART_START_FLAG)
        {
            Receive_Data_Count = 0;
            continue;
        }

        if (Receive_Data_Count < UART_DATA_LENGTH)
        {
            Receive_Data[Receive_Data_Count] = ch;
            Receive_Data_Count++;

            if (Receive_Data_Count >= UART_DATA_LENGTH)
            {
                Receive_Done = 1;
            }
        }
        else
        {
            Receive_Data_Count = 0;
        }
    }
}

// sbc와 통신이 끊길 경우
void uart_timeout(){
    if (absolute_time_diff_us(last_uart_time, get_absolute_time()) >= check_uart_time_out) {
        mnq_interrupt_flag = false;
        prev_manual_cmd = 255;

        motor_active = false;
        pwm_set_gpio_level(MNQ_PWM, 255);
        pwm = 255;
        act = 0;

        //hit_channel = 0;
        pending_hit_channel = 0;
        sending_channel = 0;
        cri_hit_count = 0;
        non_cri_hit_count = 0;

        up_wait = false;
        limit_state = '0';
        auto_up_wait_start = 0;

        uart_recover_up = true;
    }
}

void Tx_uart_SBC(int sending_channel)
{
    sprintf(buffer, "%d\n", sending_channel);
    uart_puts(UART_ID_SBC, buffer);
}

// --------------------- MNQ ---------------------

static void detect_channel_ban(void){
    // 3비트 마스크로 관리: bit0=ch1, bit1=ch2, bit2=ch3 (1이면 ban)
    uint8_t mask;

    mask = 0;
    if (Receive_Data[2] == 1) mask |= 0x01; // ch1 ban
    if (Receive_Data[3] == 1) mask |= 0x02; // ch2 ban
    if (Receive_Data[4] == 1) mask |= 0x04; // ch3 ban

    // bool로도 세팅
    first_channel_ban  = (mask & 0x01) != 0;
    second_channel_ban = (mask & 0x02) != 0;
    third_channel_ban  = (mask & 0x04) != 0;

    // IRQ enable/disable (ban이면 인터럽트 꺼서 핀 신호 무시)
    gpio_set_irq_enabled(DETECT_1, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, !first_channel_ban);
    gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, !second_channel_ban);
    gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, !third_channel_ban);

    // 밴 상태가 바뀐 순간, 남아있는 hit/flag 정리
    if (mask != prev_mask) {
        pending_hit_channel = 0;
        sending_channel = 0;
        //hit_channel = 0;
        mnq_interrupt_flag = false;
        prev_mask = mask;
    }
}

void gain_pwm_set(uint8_t gain_pwm)
{
    if (gain_pwm > 254) {
        gain_pwm = 254;
    }
    current_gain_pwm = gain_pwm;
    pwm_set_gpio_level(GAIN, current_gain_pwm);
}