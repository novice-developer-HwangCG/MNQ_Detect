// ros2 버전 총맞고 마네킹 넘기기 추가
// p06 버전, 260109 수정, sbc 켜져야만 넘어가게함. 마네킹 넘어갓다가 올라오는거 2.5초
// 마네킹 올라오는게 DIR HIGH, 내려가는게 LOW
// Hit 판정 추가 안됨
// 알람 리셋 핀 수정 필요 구형 pcb 버전에서는 GND가 공통이라 문제 없었는데 신형 pcb는 GND가 다 떨어져 있어서 모터 드라이버가 리셋 되었다가 바로 알람 뜸

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>

#define LED             PICO_DEFAULT_LED_PIN

#define LIMIT_DOWN      22   // down switch
#define LIMIT_UP        21   // up switch
#define MNQ1            10
#define MNQ2            11
#define MNQ3            12
#define MNQ_DIR         19
#define MNQ_PWM         20
#define MNQ_HIGH        1
#define MNQ_LOW         0

#define ADC_PIN         28

#define R_ENABLE        18
#define R_DIR           17
#define R_PWM           16

#define L_ENABLE        13
#define L_DIR           14
#define L_PWM           15

#define R_ENCODER_A     8
#define R_ENCODER_B     9

#define L_ENCODER_A     2
#define L_ENCODER_B     3

// ++++++++++ 알람 리셋 추가 ++++++++++
#define L_ALARM_RESET   27  // pcb 표기 = vol
#define R_ALARM_RESET   26  // pcb 표기 = mode

#define UART_TX_PIN_NUMBER              (0)
#define UART_RX_PIN_NUMBER              (1)

#define PICO_SYS_CLK_kHz                (125000)            // 125000 kHz
#define PICO_SYS_CLK                    (125000000)         // 125 MHz
#define PWM_DUTY_CLK                    (2000)              // 2 KHz
#define Unit_K                          (1000)
#define Unit_M                          (1000000)

#define UART_ID                         (uart0)
#define BAUD_RATE                       (115200)
#define DATA_BITS                       (8)
#define STOP_BITS                       (1)
#define PARITY                          (UART_PARITY_NONE)

#define UART_DATA_LENGTH                (7)                 // ++++++++++ 1 byte 추가
#define UART_START_FLAG                 (0xff)

#define ENC_HZ                          20.0f
#define DT                              (1.0f/ENC_HZ)

//P06 ROBOT SETTING
#define TRACK_WIDTH                     0.717
#define WHEEL_RADIOUS                   0.2
#define PPR                             100
#define GEAR_RATIO                      20

void UART_Rx_Handler(void);
void ConfigureUart(void);
void ConfigureGpio(void);
void tx_uart_data(float velocity, float angle, float l_enc_sec, float r_enc_sec, int mnq_shot);
void StartSignal(void);
void encoder_irq_handler(uint gpio, uint32_t events);
bool repeating_timer_callback(struct repeating_timer *t);
void reset_motor_if_uart_timeout();
void react_mnq(int mnq_command);
//adc 추가
void configure_adc(void);
//mnq shot setting 추가
void start_mnq_timer();
bool check_mnq_timer(uint32_t duration);
//mnq timer
void update_mnq_motion(void);

static uint8_t Receive_Data[UART_DATA_LENGTH]  = {0, 0, 0, 0, 0, 0, 0}; // ++++++++++ 배열 1개 추가
volatile static uint        Receive_Done                    = 0;
volatile static uint        Receive_Data_Count              = 0;
volatile static uint8_t     Receive_Char                    = 0;
volatile static uint        Frame_Count                     = 0;
static char buffer[128];

volatile int L_encoderPos = 0;
volatile int R_encoderPos = 0;

static int L_encoderOld = 0;
static int R_encoderOld = 0;

static float L_encoderSec = 0.;
static float R_encoderSec = 0.;

static float left_velocity = 0.;
static float right_velocity = 0.;
static float angular_velocity = 0.;
static float velocity = 0.;

static absolute_time_t last_uart_time;
static const uint32_t check_uart_time_out = 2000000;

// ++++++++++ 알람 리셋 추가 ++++++++++
static int alarm_cmd = 0;
static int pre_alarm = 0;

//250822 dir 이전 값 저장 변수
static int l_last = 0;
static int r_last = 1;

//mnq shot 추가
volatile bool mnq_interrupt_flag = false;
volatile bool mnq1_state = false;
volatile bool mnq2_state = false;
volatile bool mnq3_satet = false;
static bool mnq_active = false;
static uint32_t mnq_timer_start = 0;
static uint8_t mnq_live = 0;
static bool up_stop = true;
static bool down_stop = false;
static int mnq_shot = 0;
static bool up_statue = true;

//MNQ PWM
static uint     pwm_slice_num;
static uint     pwm_chan;
static bool     mnq_profile_active = false;
static uint32_t mnq_profile_start_ms = 0;

// hit count
static int hit_count = 0;

int main()
{
    stdio_init_all();
    sleep_ms(10);
    
    set_sys_clock_khz(PICO_SYS_CLK_kHz, true);
    busy_wait_ms(100);

    ConfigureUart();
    sleep_ms(10);

    ConfigureGpio();
    sleep_ms(10);

    configure_adc();
    sleep_ms(10);

    StartSignal();
    last_uart_time = get_absolute_time();
    sleep_ms(10);

    struct repeating_timer timer;
    add_repeating_timer_ms((int)(1000.0/ENC_HZ), repeating_timer_callback, NULL, &timer);

    while (true)
    {
        tight_loop_contents();
        reset_motor_if_uart_timeout();

        if (mnq_interrupt_flag)
        {
            mnq_interrupt_flag = false;

            //if (Receive_Data[5] == 0 && mnq_live != 2)
            if (Receive_Data[5] == 0)
            {
                if (mnq1_state && !mnq2_state && !mnq3_satet)   {
                    mnq_shot = 1;
                    hit_count += 1; // body hit count
                }
                else if(!mnq1_state && mnq2_state && !mnq3_satet){
                    mnq_shot = 2;
                }
                else if (!mnq1_state && !mnq2_state && mnq3_satet){
                    mnq_shot = 3;
                    hit_count += 1; // body hit count
                }
                else if (!mnq1_state && !mnq2_state && !mnq3_satet) {
                    mnq_shot = 0;
                }
            }
        }

        if (Receive_Done > 0)
        {
            if (Receive_Data[0] == 0)
            {
                gpio_put(L_ENABLE, 1);
                gpio_put(R_ENABLE, 1);
                gpio_put(L_DIR, 0);
                gpio_put(R_DIR, 0);
                pwm_set_gpio_level(L_PWM, 0);
                pwm_set_gpio_level(R_PWM, 0);
            }
            else            
            {
                gpio_put(L_ENABLE, 0);
                gpio_put(L_DIR, Receive_Data[1]);
                pwm_set_gpio_level(L_PWM, Receive_Data[2]);

                gpio_put(R_ENABLE, 0);
                gpio_put(R_DIR, Receive_Data[3]);
                pwm_set_gpio_level(R_PWM, Receive_Data[4]);
            }

            if (Receive_Data[5] != 0) {
                if (Receive_Data[5] == 1 && mnq_live != 1)  react_mnq(1);
                else if(Receive_Data[5] == 2 && mnq_live != 2)  react_mnq(2);
            } 
            else if (Receive_Data[5] == 0) {
                if (mnq_shot != 0 && mnq_live != 2) {react_mnq(2); start_mnq_timer();}
            }

            // ++++++++++ 알람 리셋 추가 ++++++++++
            alarm_cmd = Receive_Data[6]
            if((alarm_cmd == 1 || alarm_cmd == 2) && pre_alarm == 0){
                alarm_set(alarm_cmd);
            }
            pre_alarm = alarm_cmd;

            left_velocity = ((L_encoderSec / PPR) * (2*M_PI)) * WHEEL_RADIOUS / GEAR_RATIO;   //((초당 엔코더 펄스수 / 엔코더 레졸루션) * (2*pi) * (바퀴반지름 / 감속비)) 
            right_velocity = ((R_encoderSec / PPR) * (2*M_PI)) * WHEEL_RADIOUS / GEAR_RATIO;
            
            //선속도
            velocity = (left_velocity + right_velocity) / 2;
            //각속도 
            angular_velocity = (right_velocity - left_velocity) / TRACK_WIDTH;
            
            //angle이 각속도(라디안/sec) 식: 5000 -> 1바퀴 펄스값 , 20 -> 감속비 =========> 병신 공식
            //angular_velocity = (GEAR_RATIO/TRACK_WIDTH) * ((R_encoderSec-L_encoderSec)/PPR)/GEAR_RATIO* (2*M_PI); //(바퀴 반지름 / 차체 정면 바퀴사이 길이) * ((오른쪽 초당 엔코더 펄스 - 왼쪽)/레졸루션)/ 감속시 * (2*pi)

            tx_uart_data(velocity, angular_velocity, L_encoderSec, R_encoderSec, mnq_shot);
            last_uart_time = get_absolute_time();
            Receive_Done = 0;
        }

        if (check_mnq_timer(2500))
        {
            if (mnq_active == false && mnq_live != 1)
            {
                mnq_shot = 0;
                react_mnq(1);
                gpio_put(LED, 0);
            }
        }

        if (gpio_get(LIMIT_UP)==0) {
            if (up_stop == false && down_stop == true)
            {
                up_statue = true;
                pwm_set_gpio_level(MNQ_PWM, 0);
                mnq_profile_active = false;
                up_stop = true;
                down_stop = false;
                gpio_put(LED, 1);
            }
        } else if (gpio_get(LIMIT_DOWN)==0) {
            if (up_stop == true && down_stop == false)
            {
                up_statue = false;
                pwm_set_gpio_level(MNQ_PWM, 0);
                mnq_profile_active = false;
                up_stop = false;
                down_stop = true;
                gpio_put(LED, 1);
            }
        }

        update_mnq_motion();

        Frame_Count++;
        busy_wait_ms(1);
    }
    return 0;
}
void UART_Rx_Handler(void)
{
    irq_set_enabled(UART0_IRQ, false);

    if (uart_is_readable_within_us(UART_ID, 100))
    {
        Receive_Char = uart_getc(UART_ID);
    }

    if (Receive_Char == UART_START_FLAG)
    {
        Receive_Data_Count = 0x0;
    }
    else
    {
        if (Receive_Data_Count < UART_DATA_LENGTH)
        {
            Receive_Data[Receive_Data_Count] = Receive_Char;

            Receive_Data_Count++;

            if (Receive_Data_Count == UART_DATA_LENGTH)
            {
                Receive_Done = 1;
            }
        }
    }

    irq_set_enabled(UART0_IRQ, true);
}
void ConfigureUart(void)
{
    gpio_set_function(UART_TX_PIN_NUMBER, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_NUMBER, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);

    uart_set_hw_flow(UART_ID, false, false);

    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    uart_set_fifo_enabled(UART_ID, false);

    irq_set_exclusive_handler(UART0_IRQ, UART_Rx_Handler);
    irq_set_enabled(UART0_IRQ, true);

    uart_set_irq_enables(UART_ID, true, false);
}
void ConfigureGpio(void)
{
    const float pwm_freq_hz = 20000.0f;

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    gpio_init(R_ENABLE);
    gpio_set_dir(R_ENABLE, GPIO_OUT);
    gpio_put(R_ENABLE, 0);

    gpio_init(R_DIR);
    gpio_set_dir(R_DIR, GPIO_OUT);

    gpio_init(L_ENABLE);
    gpio_set_dir(L_ENABLE, GPIO_OUT);
    gpio_put(L_ENABLE, 0);

    gpio_init(L_DIR);
    gpio_set_dir(L_DIR, GPIO_OUT);

    // PWM SETTING
    gpio_set_function(R_PWM, GPIO_FUNC_PWM);
    uint R_slice_num = pwm_gpio_to_slice_num(R_PWM);

    gpio_set_function(L_PWM, GPIO_FUNC_PWM);
    uint L_slice_num = pwm_gpio_to_slice_num(L_PWM);

    pwm_set_wrap(R_slice_num, 255);
    pwm_set_wrap(L_slice_num, 255);

    float div = (float)PICO_SYS_CLK / ((255 + 1.0f) * pwm_freq_hz);
    if (div < 1.0f) div = 1.0f;
    if (div > 255.0f) div = 255.0f;

    pwm_set_clkdiv(R_slice_num, div);
    pwm_set_clkdiv(L_slice_num, div);

    pwm_set_gpio_level(R_PWM, 0);
    pwm_set_gpio_level(L_PWM, 0);

    pwm_set_enabled(R_slice_num, true);
    pwm_set_enabled(L_slice_num, true);

    const float mnq_pwm_freq_hz = 16000.0f;     // MNQ PWM 주파수 추가 16kHz

    gpio_set_function(MNQ_PWM, GPIO_FUNC_PWM);
    uint mnq_slice_num = pwm_gpio_to_slice_num(MNQ_PWM);

    pwm_set_wrap(mnq_slice_num, 255);

    float mnq_div = (float)PICO_SYS_CLK / ((255 + 1.0f) * mnq_pwm_freq_hz);
    if (mnq_div < 1.0f)   mnq_div = 1.0f;
    if (mnq_div > 255.0f) mnq_div = 255.0f;

    pwm_set_clkdiv(mnq_slice_num, mnq_div);

    pwm_set_gpio_level(MNQ_PWM, 0);
    pwm_set_enabled(mnq_slice_num, true);
    
    gpio_init(L_ENCODER_A);
    gpio_set_dir(L_ENCODER_A, GPIO_IN);
    gpio_pull_down(L_ENCODER_A);
    gpio_set_irq_enabled_with_callback(L_ENCODER_A, GPIO_IRQ_EDGE_RISE, true, &encoder_irq_handler);

    gpio_init(L_ENCODER_B);
    gpio_set_dir(L_ENCODER_B, GPIO_IN);
    gpio_pull_down(L_ENCODER_B);

    gpio_init(R_ENCODER_A);
    gpio_set_dir(R_ENCODER_A, GPIO_IN);
    gpio_pull_down(R_ENCODER_A);
    gpio_set_irq_enabled(R_ENCODER_A, GPIO_IRQ_EDGE_RISE, true); 

    gpio_init(R_ENCODER_B);
    gpio_set_dir(R_ENCODER_B, GPIO_IN);
    gpio_pull_down(R_ENCODER_B);

    gpio_init(MNQ_DIR);
    gpio_set_dir(MNQ_DIR, GPIO_OUT);
    gpio_put(MNQ_DIR, 0);

    gpio_init(MNQ1);
    gpio_set_dir(MNQ1, GPIO_IN);
    gpio_pull_down(MNQ1);
    gpio_set_irq_enabled(MNQ1, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true);

    gpio_init(MNQ2);
    gpio_set_dir(MNQ2, GPIO_IN);
    gpio_pull_down(MNQ2);
    gpio_set_irq_enabled(MNQ2, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true);

    gpio_init(MNQ3);
    gpio_set_dir(MNQ3, GPIO_IN);
    gpio_pull_down(MNQ3);
    gpio_set_irq_enabled(MNQ3, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true);   

    gpio_init(LIMIT_DOWN);
    gpio_set_dir(LIMIT_DOWN, GPIO_IN);
    gpio_pull_up(LIMIT_DOWN);

    gpio_init(LIMIT_UP);
    gpio_set_dir(LIMIT_UP, GPIO_IN);
    gpio_pull_up(LIMIT_UP);

    // ++++++++++ 알람 리셋 추가 ++++++++++
    gpio_init(L_ALARM_RESET);
    gpio_set_dir(L_ALARM_RESET, GPIO_OUT);
    gpio_pull_up(L_ALARM_RESET);
    gpio_put(L_ALARM_RESET, 1); // 기본 high

    gpio_init(R_ALARM_RESET);
    gpio_set_dir(R_ALARM_RESET, GPIO_OUT);
    gpio_pull_up(R_ALARM_RESET);
    gpio_put(R_ALARM_RESET, 1); // 기본 high
}
void configure_adc(void)
{
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(2);
}
void StartSignal(void)
{
    gpio_put(LED, 1);
    sleep_ms(1000);
    gpio_put(LED, 0);
    sleep_ms(1000);
}
void encoder_irq_handler(uint gpio, uint32_t events)
{
    if (gpio == L_ENCODER_A)    L_encoderPos += (gpio_get(L_ENCODER_A)==gpio_get(L_ENCODER_B)?1:-1);
    else if (gpio == R_ENCODER_A)   R_encoderPos += (gpio_get(R_ENCODER_A)==gpio_get(R_ENCODER_B)?-1:1);
    else if (gpio == MNQ1 || gpio == MNQ2 || gpio == MNQ3)
    {
        mnq1_state = gpio_get(MNQ1);
        mnq2_state = gpio_get(MNQ2);
        mnq3_satet = gpio_get(MNQ3);
        mnq_interrupt_flag = true;
    }
}
bool repeating_timer_callback(struct repeating_timer *t)
{
    float dL = (float)(L_encoderPos - L_encoderOld);
    float dR = (float)(R_encoderPos - R_encoderOld);

    L_encoderOld = L_encoderPos;
    R_encoderOld = R_encoderPos;

    L_encoderSec = dL / DT;
    R_encoderSec = dR / DT;

    return true;
}

void tx_uart_data(float velocity, float angle, float left_enc_sec, float right_enc_sec, int mnq_shot){
    const float conversion_factor = 3.3f / (1 << 12);
    uint16_t result = adc_read();
    sprintf(buffer, "%.7f,%.7f,%f,%f,%.2f,%d\n", velocity, angle, left_enc_sec, right_enc_sec, result*conversion_factor, mnq_shot);
    uart_puts(UART_ID, buffer);
}

// ++++++++++ 알람 리셋 추가 ++++++++++
void alarm_set(int alarm_reset){
    if (alarm_reset == 1){
        gpio_put(L_ALARM_RESET, 0);
        sleep_ms(1000);       // 제어 중 신호 줄 꺼면 sleep 말고 논 블로킹으로 사용해야 함
        gpio_put(L_ALARM_RESET, 1);
        pre_alarm = 1;
    } else if (alarm_reset == 2){
        gpio_put(R_ALARM_RESET, 0);
        sleep_ms(1000);
        gpio_put(R_ALARM_RESET, 1);
        pre_alarm = 2;
    }
}

void reset_motor_if_uart_timeout() {
    if (absolute_time_diff_us(last_uart_time, get_absolute_time()) >= check_uart_time_out) {
        gpio_put(L_DIR, 0);
        gpio_put(R_DIR, 0);
        pwm_set_gpio_level(L_PWM, 0);
        pwm_set_gpio_level(R_PWM, 0);
    }
}

void react_mnq(int mnq_command)
{
    if (mnq_command == 1) {
        mnq_live = 1;
        if (up_statue == false) {
            gpio_put(MNQ_DIR, MNQ_HIGH);
            mnq_profile_start_ms = to_ms_since_boot(get_absolute_time());
            mnq_profile_active = true;
            pwm_set_gpio_level(MNQ_PWM, 254);
            gpio_put(LED, 0);
        }
    }
    else if (mnq_command == 2) {
        mnq_live = 2;
        if (up_statue == true){
            gpio_put(MNQ_DIR, MNQ_LOW);
            mnq_profile_start_ms = to_ms_since_boot(get_absolute_time());
            mnq_profile_active = true;
            pwm_set_gpio_level(MNQ_PWM, 254);
            gpio_put(LED, 0);
        }
    }
}
void update_mnq_motion(void)
{
    if (!mnq_profile_active) {
        return;
    }

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed = now_ms - mnq_profile_start_ms;

    if (elapsed < 500) pwm_set_gpio_level(MNQ_PWM, 254);
    else pwm_set_gpio_level(MNQ_PWM, 150);
}

void start_mnq_timer()
{
    mnq_active = true;
    mnq_timer_start = to_ms_since_boot(get_absolute_time());
}
bool check_mnq_timer(uint32_t duration)
{
    if (mnq_active) 
    {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if ((current_time - mnq_timer_start) >= duration)
        {
            mnq_active = false;
            return true;
        }
    }
    return false;
}