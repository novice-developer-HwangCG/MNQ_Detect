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

// 신호 P1 P2 확인

#define LED             PICO_DEFAULT_LED_PIN

// --------------------- MOTOR UART 0 ---------------------
#define UART_TX_PIN_MOTOR              (0)
#define UART_RX_PIN_MOTOR              (1)

#define UART_ID_MOTOR                   (uart0)
#define MOTOR_BAUD_RATE                 (9600)

#define MOTOR_ID                        0x00     // 공장 초기 값

static const uint8_t HEADER[2] = { 0xFF, 0xFE };

// --------------------- Control (common) ---------------------
#define MODE_POS_SPEED                  0x01   
#define MODE_ACC_POS                    0x02
#define MODE_ACC_SPEED                  0x03
#define MODE_OPEN_LOOP                  0x11     // SA only

// --------------------- Motor Settings (SA) ---------------------
#define SA_MODE_SET_POS_CTRL     0x04   // 위치 제어기 설정
#define SA_MODE_SET_SPD_CTRL     0x05   // 속도 제어기 설정
#define SA_MODE_SET_ID           0x06   // ID 설정
#define SA_MODE_SET_BAUD         0x07   // 통신 속도 설정
#define SA_MODE_SET_RESPTIME     0x08   // 통신 응답시간 설정
#define SA_MODE_SET_EXT_RATIO    0x09   // 외부 감속비 설정
#define SA_MODE_SET_ONOFF        0x0A   // 제어 On/Off 설정
#define SA_MODE_SET_POSMODE      0x0B   // 위치제어 모드 설정
#define SA_MODE_RESET_POS        0x0C   // 위치 초기화
#define SA_MODE_RESET_FACTORY    0x0D   // 공장 초기화

// --------------------- Motor Feedback Requests (SA) ---------------------
#define SA_REQ_PING         0xA0   // Ping 요청
#define SA_REQ_POS          0xA1   // 위치 피드백 요청
#define SA_REQ_SPD          0xA2   // 속도 피드백 요청
#define SA_REQ_POS_CTRL     0xA3   // 위치 제어기 피드백 요청
#define SA_REQ_SPD_CTRL     0xA4   // 속도 제어기 피드백 요청
#define SA_REQ_RESP_TIME    0xA5   // 통신 응답시간 피드백 요청
#define SA_REQ_EXT_RATIO    0xA6   // 외부 감속비 피드백 요청
#define SA_REQ_ONOFF        0xA7   // 제어 On/Off 피드백 요청
#define SA_REQ_POSMODE      0xA8   // 위치제어 모드 피드백 요청
#define SA_REQ_FIRMWARE     0xCD   // 펌웨어 버전 요청

// --------------------- Motor Feedback Responses (SA) ---------------------
#define SA_FEED_PING        0xD0    // ping 응답
#define SA_FEED_POS         0xD1    // 위치 피드백 응답
#define SA_FEED_SPD         0xD2    // 속도 피드백 응답
#define SA_FEED_POS_CTRL    0xD3    // 위치 제어기 피드백 응답
#define SA_FEED_SPD_CTRL    0xD4    // 속도 제어기 피드백 응답
#define SA_FEED_RESP_TIME   0xD5    // 통신 응답시간 피드백 응답
#define SA_FEED_EXT_RATIO   0xD6    // 외부 감속비 피드백 응답
#define SA_FEED_ONOFF       0xD7    // 제어 on/off 피드백 응답
#define SA_FEED_POSMODE     0xD8    // 위치 제어 모드 피드백 응답
#define SA_FEED_FIRMWARE    0xFD    // 펌웨어 버전 응답

volatile bool acting_auto = false;

static bool prev_control_signal = false;
static uint8_t motor_phase = 0;
static uint32_t motor_timer_start = 0;
static uint8_t prev_manual_cmd = 255;
float cur_deg = 0.0f;

// --------------------- SBC UART 1 ---------------------
#define UART_TX_PIN_SBC                 (8)
#define UART_RX_PIN_SBC                 (9)

#define UART_ID_SBC                     (uart1)
#define SBC_BAUD_RATE                   (115200)

#define UART_START_FLAG                 (0xff)
#define UART_DATA_LENGTH                (8)

static uint8_t Receive_Data[UART_DATA_LENGTH]  = {0, 0, 0, 0, 0, 0, 0, 0};
volatile static uint        Receive_Done                    = 0;
volatile static uint        Receive_Data_Count              = 0;
volatile static uint8_t     Receive_Char                    = 0;
volatile static uint        Frame_Count                     = 0;

static bool control_signal = false;

static absolute_time_t last_uart_time;
static const uint32_t check_uart_time_out = 2000000;

// --------------------- Detect Sensor ---------------------
#define DETECT_1        19
#define DETECT_2        20
#define DETECT_3        21
#define GAIN            14

volatile bool mnq_interrupt_flag = false;
volatile bool mnq1_state = false;
volatile bool mnq2_state = false;
volatile bool mnq3_satet = false;
static bool motor_active = false;

volatile bool Ban_channel_all = false;
volatile bool first_channel_ban = false;
volatile bool second_channel_ban = false;
volatile bool third_channel_ban = false;

static int hit_channel = 0;

static uint8_t prev_mask = 0;

// --------------------- Common set ---------------------
#define DATA_BITS                       (8)
#define STOP_BITS                       (1)
#define PARITY                          (UART_PARITY_NONE)

static char buffer[64];

#define PICO_SYS_CLK_kHz                (125000)            // 125000 kHz
#define PICO_SYS_CLK                    (125000000)

// --------------------- Common function ---------------------
void ConfigureUart_motor(void);
void ConfigureUart_sbc(void);
void ConfigureGpio(void);
static void gpio_irq_callback(uint gpio, uint32_t events);
void StartSignal(void);
void uart_timeout();

// --------------------- Motor function ---------------------
// static uint16_t be16(uint8_t hi, uint8_t lo);
uint8_t calcChecksum(const uint8_t* frame, size_t len);
size_t  buildFrame(uint8_t dev_id, uint8_t mode, const uint8_t* data, size_t data_len, uint8_t* out, size_t out_max);
static void motor_send(uint8_t dev_id, uint8_t mode, const uint8_t* data, size_t data_len);

void motor_set_onoff(uint8_t dev_id, bool on);
void sa_set_posmode(uint8_t dev_id, bool absolute);
void motor_reset_pos(uint8_t dev_id);

int  cmd_pos_speed(uint8_t dev_id, bool cw, float pos_deg, float spd_rpm);
int  cmd_acc_pos(uint8_t dev_id, bool cw, float pos_deg, float arrive_s);
int  cmd_acc_speed(uint8_t dev_id, bool cw, float spd_rpm, float arrive_s);
int  cmd_open_loop(uint8_t dev_id, bool cw, float duty_percent);
void safe_stop(uint8_t dev_id);

static bool read_one_motor_frame(uint8_t *frame, size_t *out_len, uint32_t timeout_ms);
bool motor_request_pos_deg(uint8_t dev_id, float *pos_deg);

// --------------------- SBC function ---------------------
void Rx_uart_SBC(void);
void Tx_uart_SBC(int hit_channel, float cur_deg);

static void detect_channel_ban(void);
static bool signal_confirm_5ms(uint pin);

void start_motor_timer();
bool check_motor_timer(uint32_t duration);

//--------------------- Main ---------------------
int main(){
    stdio_init_all();
    sleep_ms(10);

    set_sys_clock_khz(PICO_SYS_CLK_kHz, true);
    busy_wait_ms(100);

    ConfigureUart_sbc();
    sleep_ms(10);

    ConfigureUart_motor();
    sleep_ms(10);

    ConfigureGpio();
    sleep_ms(10);

    StartSignal();
    last_uart_time = get_absolute_time();
    sleep_ms(10);

    sa_set_posmode(MOTOR_ID, true);   // absolute
    sleep_ms(300);

    // // 절대 모드 설정 이 후 현재 각도가 틀어진 상태면 0도 각도로 움직이게 하기
    // cmd_acc_pos(MOTOR_ID, false, 0.0f, 2.0f);
    // sleep_ms(2000);

    // 0도 각도 이동 후 위치 초기화 하여 현재 위치를 0도로 다시 잡기 (mnq가 올라와 있을 때)
    motor_reset_pos(MOTOR_ID);        // 현재 위치를 0도로
    sleep_ms(200);

    // motor_set_onoff(MOTOR_ID, true);
    // sleep_ms(120);

    // cmd_acc_pos(MOTOR_ID, false, 0.0f, 2.0f);
    // sleep_ms(2000);

    // motor_set_onoff(MOTOR_ID, false);
    // sleep_ms(50);

    // motor_reset_pos(MOTOR_ID);
    // sleep_ms(200);

    while (true)
    {
        tight_loop_contents();
        uart_timeout();

        // 제어 신호가 들어오면 신호 감지하기 (수동 상태에서도 신호는 감지하되 모터 동작은 못하게 막기)
        if (mnq_interrupt_flag)
        {
            mnq_interrupt_flag = false;
            if (control_signal == true)
            {
                if (mnq1_state && !mnq2_state && !mnq3_satet)   {
                    hit_channel = 1;
                }
                else if(!mnq1_state && mnq2_state && !mnq3_satet){
                    hit_channel = 2;
                }
                else if (!mnq1_state && !mnq2_state && mnq3_satet){
                    hit_channel = 3;
                }
                else {
                    hit_channel = 0;
                }
            }
        }

        /*
        Receive_Data[0] → 0 = 제어 off, 1 = on
        Receive_Data[1] → 0 = 자동(자동 모터 제어), 1 = 수동(사용자 직접 제어)
        Receive_Data[2] → 수동일 경우 0 = Up, 1 = Down
        Receive_Data[3] → 0 = Default(pass), 1 = Reset (모터 각도가 0도가 아닐 경우)
        Receive_Data[4] → 0 = 채널 모두 on, 1 채널 모두 off
        Receive_Data[5] → 0 = 1 채널 on, 1 = 1 채널 off
        Receive_Data[6] → 0 = 2 채널 on, 1 = 2 채널 off
        Receive_Data[7] → 0 = 3 채널 on, 1 = 3 채널 off
        */
        if (Receive_Done > 0){
            if (Receive_Data[0] == 0){
                control_signal = false;
                motor_phase = 0;
                motor_active = false;
                hit_channel = 0;    // 모터 제어는 하지 않고 센서 신호는 보고 싶을 때 제거
                prev_manual_cmd = 255;
                acting_auto = false;
            }
            else{
                control_signal = true;
            }

            bool control_changed = (control_signal != prev_control_signal);
            if (control_changed) {
                motor_set_onoff(MOTOR_ID, control_signal);
                prev_control_signal = control_signal;

                if (control_signal) {
                    sleep_ms(120);
                }
            }

            detect_channel_ban();

            // 보고용 hit는 스냅샷으로 따로 잡기
            int report_hit = hit_channel;
            // 코드 버그, 레이스 등으로 인해 밴된 채널이 값이 전송이 될 경우 아래 if문 주석 해제
            // if (Ban_channel_all ||
            //     (report_hit == 1 && first_channel_ban) ||
            //     (report_hit == 2 && second_channel_ban) ||
            //     (report_hit == 3 && third_channel_ban)) {
            //     report_hit = 0;
            // }

            if (control_signal){
                // 자동
                if (Receive_Data[1] == 0){
                    acting_auto = true;
                    prev_manual_cmd = 255;
                    // 모터 동작용 hit는 auto일 때만 사용
                    int hit_for_motor = hit_channel;

                    if (Ban_channel_all ||
                        (hit_for_motor == 1 && first_channel_ban) ||
                        (hit_for_motor == 2 && second_channel_ban) ||
                        (hit_for_motor == 3 && third_channel_ban)) {
                        hit_for_motor = 0;
                    }
                    if (hit_for_motor != 0 && motor_phase == 0) {
                        cmd_acc_pos(MOTOR_ID, true, 90.0f, 1.0f);  // 신호 들어오면 목표 각도 까지 1.0초 만에 도착
                        // cmd_pos_speed(MOTOR_ID, false, 90.0f, 4.0f);
                        start_motor_timer();
                        motor_phase = 1;          // to 90
                        gpio_put(LED, 0);
                    }
                }
                // 수동
                else{
                    if (acting_auto) {
                        motor_phase = 0;
                        motor_active = false;
                    }
                    acting_auto = false;    // 자동 제어 중 'Receive_Data[2]'값이 들어올 경우 수동 제어에 대한 오동작 방지용, 'Receive_Data[1]'값이 0이 아닌 값이 들어오는게 아닌 이상 오동작 막기
                    if (Receive_Data[2] != prev_manual_cmd){
                        prev_manual_cmd = Receive_Data[2];
                        // 0 = up (0도), 1 = down (90도)
                        if (Receive_Data[2] == 0){
                            cmd_acc_pos(MOTOR_ID, false, 0.6f, 1.0f);  // 목표 각도 까지 1.0초 만에 도착  방향 true => 뒤로 넘어감
                            // cmd_pos_speed(MOTOR_ID, false, 0.0f, 4.0f);
                        }
                        else {
                            cmd_acc_pos(MOTOR_ID, true, 90.0f, 1.0f); // 목표 각도 까지 1.0초 만에 도착   방향 false => 위로 올라옴
                            // cmd_pos_speed(MOTOR_ID, true, 90.0f, 4.0f);
                        }
                    }
                }
                if (Receive_Data[3] == 1){
                    // TODO : 현재 각도 값이 0도(진동으로 인해 움직인 ±3는 무시)가 아닐 경우 0도로 복귀
                    // 해당 부분은 위치(각도) 값 피드백 받아서 위치(각도) 확인 후에 실행
                    if (motor_request_pos_deg(MOTOR_ID, &cur_deg)) {
                        if (fabsf(cur_deg) > 2.0f) {
                            cmd_acc_pos(MOTOR_ID, false, 0.6f, 1.0f);
                        }
                    }
                }
            }
            Tx_uart_SBC(report_hit, cur_deg);
            hit_channel = 0;
            last_uart_time = get_absolute_time();
            Receive_Done = 0;
        }
        // 90도 이동 1.0초 끝 -> 2초 대기 시작
        if (motor_phase == 1) {
            if (check_motor_timer(1300)) {
                if (motor_request_pos_deg(MOTOR_ID, &cur_deg)) {
                    Tx_uart_SBC(0, cur_deg);   // 90도 도달 후 현재 각도 전송
                }
                start_motor_timer();
                motor_phase = 2;      // holding 90
                gpio_put(LED, 1);
            }
        }
        // 90도에서 2.0초 대기 끝 -> 0도 복귀 시작
        else if (motor_phase == 2) {
            if (check_motor_timer(2000)){
                cmd_acc_pos(MOTOR_ID, false, 0.6f, 1.0f); // 목표 각도 까지 1.0초 만에 도착
                // cmd_pos_speed(MOTOR_ID, false, 0.0f, 4.0f);
                start_motor_timer();
                motor_phase = 3;      // to 0
            }
        }
        // 0도 이동 1.0초 끝 -> 2초 대기 시작
        else if (motor_phase == 3) {
            if (check_motor_timer(1300)) {
                if (motor_request_pos_deg(MOTOR_ID, &cur_deg)) {
                    Tx_uart_SBC(0, cur_deg);   // 0도 도달 후 현재 각도 전송
                }
                start_motor_timer();
                motor_phase = 4;
                hit_channel = 0;
            }
        }
        // 0도에서 2.0초 대기 끝 -> 종료
        else if (motor_phase == 4) {
            if (check_motor_timer(2000)) {
                motor_phase = 0;      // normal state
                hit_channel = 0;
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

    pwm_set_gpio_level(GAIN, 100);
}

static void gpio_irq_callback(uint gpio, uint32_t events) {
    if (gpio == DETECT_1 || gpio == DETECT_2 || gpio == DETECT_3) {
        mnq1_state = gpio_get(DETECT_1);
        mnq2_state = gpio_get(DETECT_2);
        mnq3_satet = gpio_get(DETECT_3);
        mnq_interrupt_flag = true;
    }
}

void StartSignal(void)
{
    gpio_put(LED, 1);
    sleep_ms(3000);
    gpio_put(LED, 0);
}

// motor와 통신이 끊길 경우 정지, sbc와 통신이 끊길 경우 control_signal = false;로 변환
void uart_timeout(){
    if (absolute_time_diff_us(last_uart_time, get_absolute_time()) >= check_uart_time_out) {
        control_signal = false;
        motor_set_onoff(MOTOR_ID, false);

        prev_control_signal = false;
        prev_manual_cmd = 255;
        acting_auto = false;

        motor_phase = 0;
        motor_active = false;
        hit_channel = 0;
        mnq_interrupt_flag = false;
    }
}

// --------------------- Motor ---------------------
void ConfigureUart_motor(void)
{
    gpio_set_function(UART_TX_PIN_MOTOR, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_MOTOR, GPIO_FUNC_UART);

    uart_init(UART_ID_MOTOR, MOTOR_BAUD_RATE);
    uart_set_hw_flow(UART_ID_MOTOR, false, false);
    uart_set_format(UART_ID_MOTOR, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID_MOTOR, false);

    // irq_set_exclusive_handler(UART0_IRQ, Rx_uart_motor);
    // irq_set_enabled(UART0_IRQ, true);

    // uart_set_irq_enables(UART_ID_MOTOR, true, false);
}

// --------------------- Motor Utility ---------------------
// static uint16_t be16(uint8_t hi, uint8_t lo)
// {
//     return ((uint16_t)hi << 8) | lo;
// }

static inline uint16_t clamp_u16_i32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return (uint16_t)lo;
    if (v > hi) return (uint16_t)hi;
    return (uint16_t)v;
}
static inline uint8_t clamp_u8_i32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return (uint8_t)lo;
    if (v > hi) return (uint8_t)hi;
    return (uint8_t)v;
}

// 체크섬: 헤더(2바이트) & checksum 바이트 제외한 모든 바이트 합의 NOT
uint8_t calcChecksum(const uint8_t* frame, size_t len)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        if (i == 0 || i == 1 || i == 4) continue;
        sum += frame[i];
    }
    return (uint8_t)(~(sum & 0xFF));
}

// 프레임 생성
// out: FF FE | ID | LEN | CHK | MODE | DATA...
// LEN = 1(chk)+1(mode)+data_len
size_t buildFrame(uint8_t dev_id, uint8_t mode,
                  const uint8_t* data, size_t data_len,
                  uint8_t* out, size_t out_max)
{
    size_t total = 6 + data_len;
    if (out_max < total) return 0;

    out[0] = 0xFF;
    out[1] = 0xFE;
    out[2] = dev_id;
    out[3] = (uint8_t)(1 + 1 + data_len); // LEN
    out[4] = 0x00;                        // checksum placeholder
    out[5] = mode;

    if (data_len && data) {
        memcpy(&out[6], data, data_len);
    }

    out[4] = calcChecksum(out, total);
    return total;
}

static void motor_send(uint8_t dev_id, uint8_t mode, const uint8_t* data, size_t data_len)
{
    uint8_t frame[32];
    size_t n = buildFrame(dev_id, mode, data, data_len, frame, sizeof(frame));
    if (n == 0) return;
    uart_write_blocking(UART_ID_MOTOR, frame, n);
}

// ON/OFF: 0x00=On, 0x01=Off
void motor_set_onoff(uint8_t dev_id, bool on)
{
    uint8_t d[1] = { (uint8_t)(on ? 0x00 : 0x01) };
    motor_send(dev_id, SA_MODE_SET_ONOFF, d, 1);
}

// 위치 초기화 (data 없음)
void motor_reset_pos(uint8_t dev_id)
{
    uint8_t d[1] = { 0x00 };
    motor_send(dev_id, SA_MODE_RESET_POS, d, 1);
}

// 절대 모드 설정
void sa_set_posmode(uint8_t dev_id, bool absolute)
{
    uint8_t d[1] = { (uint8_t)(absolute ? 0x00 : 0x01) };
    motor_send(dev_id, SA_MODE_SET_POSMODE, d, 1);
}

// 모터 제어 선택
// 0x01 위치+속도: [dir(1), pos(2), spd(2)]
int cmd_pos_speed(uint8_t dev_id, bool cw, float pos_deg, float spd_rpm)
{
    int32_t pos_raw_i = (int32_t)lroundf(pos_deg * 100.0f); // 0.01°
    int32_t spd_raw_i = (int32_t)lroundf(spd_rpm * 10.0f);  // 0.1RPM

    uint16_t pos_raw = clamp_u16_i32(pos_raw_i, 0, 65533);
    uint16_t spd_raw = clamp_u16_i32(spd_raw_i, 0, 65533);

    uint8_t d[5];
    d[0] = (uint8_t)(cw ? 0x01 : 0x00);
    d[1] = (uint8_t)((pos_raw >> 8) & 0xFF);
    d[2] = (uint8_t)(pos_raw & 0xFF);
    d[3] = (uint8_t)((spd_raw >> 8) & 0xFF);
    d[4] = (uint8_t)(spd_raw & 0xFF);

    motor_send(dev_id, MODE_POS_SPEED, d, sizeof(d));
    return 0;
}
// 0x02 가감속 위치: [dir(1), pos(2), arrive(1)] arrive=0.1s
int cmd_acc_pos(uint8_t dev_id, bool cw, float pos_deg, float arrive_s)
{
    int32_t pos_raw_i = (int32_t)lroundf(pos_deg * 100.0f);
    int32_t arr_raw_i = (int32_t)lroundf(arrive_s * 10.0f);

    uint16_t pos_raw = clamp_u16_i32(pos_raw_i, 0, 65533);
    uint8_t  arr_raw = clamp_u8_i32(arr_raw_i, 1, 255);

    uint8_t d[4];
    d[0] = (uint8_t)(cw ? 0x01 : 0x00);
    d[1] = (uint8_t)((pos_raw >> 8) & 0xFF);
    d[2] = (uint8_t)(pos_raw & 0xFF);
    d[3] = arr_raw;

    motor_send(dev_id, MODE_ACC_POS, d, sizeof(d));
    return 0;
}
// 0x03 가감속 속도: [dir(1), spd(2), arrive(1)]
int cmd_acc_speed(uint8_t dev_id, bool cw, float spd_rpm, float arrive_s)
{
    int32_t spd_raw_i = (int32_t)lroundf(spd_rpm * 10.0f);
    int32_t arr_raw_i = (int32_t)lroundf(arrive_s * 10.0f);

    uint16_t spd_raw = clamp_u16_i32(spd_raw_i, 0, 65533);
    uint8_t  arr_raw = clamp_u8_i32(arr_raw_i, 1, 255);

    uint8_t d[4];
    d[0] = (uint8_t)(cw ? 0x01 : 0x00);
    d[1] = (uint8_t)((spd_raw >> 8) & 0xFF);
    d[2] = (uint8_t)(spd_raw & 0xFF);
    d[3] = arr_raw;

    motor_send(dev_id, MODE_ACC_SPEED, d, sizeof(d));
    return 0;
}
// 0x11 open-loop: [dir(1), duty(2)] duty=0.01% (0~10000)
int cmd_open_loop(uint8_t dev_id, bool cw, float duty_percent)
{
    int32_t duty_raw_i = (int32_t)lroundf(duty_percent * 100.0f);
    uint16_t duty_raw = clamp_u16_i32(duty_raw_i, 0, 10000);

    uint8_t d[3];
    d[0] = (uint8_t)(cw ? 0x01 : 0x00);
    d[1] = (uint8_t)((duty_raw >> 8) & 0xFF);
    d[2] = (uint8_t)(duty_raw & 0xFF);

    motor_send(dev_id, MODE_OPEN_LOOP, d, sizeof(d));
    return 0;
}
// 안전 정지 사용 X 일단 보류
void safe_stop(uint8_t dev_id)
{
    // 감속 정지 후 OFF
    cmd_acc_speed(dev_id, false, 0.0f, 0.2f);
    motor_set_onoff(dev_id, false);
}

static bool read_one_motor_frame(uint8_t *frame, size_t *out_len, uint32_t timeout_ms)
{
    uint32_t start = to_ms_since_boot(get_absolute_time());
    size_t idx = 0;
    size_t total_len = 0;

    while ((to_ms_since_boot(get_absolute_time()) - start) < timeout_ms) {
        if (!uart_is_readable(UART_ID_MOTOR)) {
            sleep_us(200);
            continue;
        }
        uint8_t b = uart_getc(UART_ID_MOTOR);
        if (idx == 0) {
            if (b == 0xFF) {
                frame[idx++] = b;
            }
            continue;
        }
        if (idx == 1) {
            if (b == 0xFE) {
                frame[idx++] = b;
            } else {
                idx = 0;
            }
            continue;
        }
        frame[idx++] = b;
        if (idx == 4) {
            total_len = (size_t)frame[3] + 4;   // total = LEN + 4
            if (total_len < 6 || total_len > 32) {
                idx = 0;
                total_len = 0;
            }
        }
        if (total_len > 0 && idx >= total_len) {
            if (calcChecksum(frame, total_len) != frame[4]) {
                return false;
            }
            *out_len = total_len;
            return true;
        }
    }
    return false;
}

bool motor_request_pos_deg(uint8_t dev_id, float *pos_deg)
{
    uint8_t req[8];
    size_t n = buildFrame(dev_id, SA_REQ_POS, NULL, 0, req, sizeof(req));
    if (n == 0) return false;

    uart_write_blocking(UART_ID_MOTOR, req, n);

    uint8_t rx[32];
    size_t rx_len = 0;
    if (!read_one_motor_frame(rx, &rx_len, 50)) {   // 10~50ms 권장 범위
        return false;
    }

    // 기대 응답: D1, 데이터 6바이트 = dir(1), pos(2), spd(2), current(1)
    if (rx_len < 12) return false;
    if (rx[2] != dev_id) return false;
    if (rx[5] != SA_FEED_POS) return false;

    uint16_t pos_raw = ((uint16_t)rx[7] << 8) | rx[8];
    *pos_deg = (float)pos_raw / 100.0f;   // 0.01°
    return true;
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
    irq_set_enabled(UART1_IRQ, false);
    if (uart_is_readable_within_us(UART_ID_SBC, 100))
    {
        Receive_Char = uart_getc(UART_ID_SBC);
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
    irq_set_enabled(UART1_IRQ, true);
}

void Tx_uart_SBC(int hit_channel, float cur_deg)
{
    sprintf(buffer, "%d, %.2f\n", hit_channel, cur_deg);
    uart_puts(UART_ID_SBC, buffer);
}

static void detect_channel_ban(void){
    // 3비트 마스크로 관리: bit0=ch1, bit1=ch2, bit2=ch3 (1이면 ban)
    uint8_t mask;

    Ban_channel_all = (Receive_Data[4] == 1);

    if (Ban_channel_all) {
        mask = 0x07; // 111: 전부 ban
    } else {
        mask = 0;
        if (Receive_Data[5] == 1) mask |= 0x01; // ch1 ban
        if (Receive_Data[6] == 1) mask |= 0x02; // ch2 ban
        if (Receive_Data[7] == 1) mask |= 0x04; // ch3 ban
    }

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
        hit_channel = 0;
        mnq_interrupt_flag = false;
        prev_mask = mask;
    }
}

// 해당 신호가 진짜 신호인지 확인 하기 위한 1ms 당 5번 확인
static bool signal_confirm_5ms(uint pin) {
    for (int i = 0; i < 5; i++) {
        if (gpio_get(pin) == 0) return false;
        sleep_ms(1);
    }
    return true;
}

// 모터가 움직일 시간을 주기 위한 별도 타이머
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
