#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

// P1_P2 신호 확인
// NM-1 (NuriMotor-1)
// 제어 on 이후 속도 지령 주어야 토크 유지이니 제어 on되면 현 위치(각도)로 제어 명령주어서 토크 유지하게 하기 

#define LED             PICO_DEFAULT_LED_PIN

// --------------------- MOTOR UART 0 ---------------------
#define UART_TX_PIN_MOTOR              (0)
#define UART_RX_PIN_MOTOR              (1)

#define UART_ID_MOTOR                  (uart0)
#define MOTOR_BAUD_RATE                (9600)

#define MOTOR_ID                       0x00     // 공장 초기 값

// --------------------- SBC UART 1 ---------------------
#define UART_TX_PIN_SBC                (8)
#define UART_RX_PIN_SBC                (9)

#define UART_ID_SBC                    (uart1)
#define SBC_BAUD_RATE                  (115200)

#define UART_START_FLAG                (0xff)
#define UART_DATA_LENGTH               (9)

static uint8_t Receive_Data[UART_DATA_LENGTH]  = {0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile static uint        Receive_Done       = 0;
volatile static uint        Receive_Data_Count = 0;
volatile static uint8_t     Receive_Char       = 0;

static bool control_signal = false;

static absolute_time_t last_uart_time;
static const uint32_t check_uart_time_out = 5000000;

// --------------------- Detect Sensor ---------------------
#define DETECT_1_P1        19
#define DETECT_2_P2        20
//#define DETECT_3        21
#define GAIN               14

#define P1_CONFIRM_US          5000u
#define P2_CHECK1_US           1000u
#define P2_CHECK2_US           2000u
#define DETECT_COOLDOWN_US     80000u

#define P2_HIST_SIZE           8
#define P2_ACTIVE_NONE         0xFFu

typedef enum {
    SIG_IDLE = 0,
    SIG_CONFIRM_P1,
    SIG_WAIT_P1_FALL,
    SIG_WAIT_P2_JUDGE,
    SIG_COOLDOWN
} signal_detect_state_t;

typedef struct {
    uint32_t rise_us;
    uint32_t fall_us;
    uint32_t rise_seq;
    uint32_t fall_seq;
    bool valid;
    bool low_seen;
} p2_pulse_t;

typedef struct {
    uint32_t p1_rise_us;
    uint32_t p1_fall_us;
    uint32_t p1_rise_seq;
    uint32_t p1_fall_seq;
    bool p1_high;

    p2_pulse_t p2_hist[P2_HIST_SIZE];
} signal_snapshot_t;

// ISR 갱신값
static volatile uint32_t sig_event_seq = 0;

static volatile uint32_t isr_p1_rise_us = 0;
static volatile uint32_t isr_p1_fall_us = 0;
static volatile uint32_t isr_p1_rise_seq = 0;
static volatile uint32_t isr_p1_fall_seq = 0;
static volatile bool     isr_p1_high = false;

static volatile bool     isr_p2_high = false;
static volatile p2_pulse_t isr_p2_hist[P2_HIST_SIZE];
static volatile uint8_t  isr_p2_write_idx = 0;
static volatile uint8_t  isr_p2_active_idx = P2_ACTIVE_NONE;

// main loop 상태값
static signal_detect_state_t sig_state = SIG_IDLE;

static uint32_t active_p1_rise_us = 0;
static uint32_t active_p1_fall_us = 0;
static uint32_t active_p1_rise_seq = 0;
static uint32_t last_used_p1_rise_seq = 0;
static uint32_t sig_cooldown_start_us = 0;

volatile bool first_channel_ban = false;
volatile bool second_channel_ban = false;

static uint8_t default_gain_pwm = 65;
static uint8_t current_gain_pwm = 65;

volatile uint8_t cur_cri_set = 0;
volatile uint8_t cri_hit_count = 0;

volatile uint8_t cur_non_cri_set = 0;
volatile uint8_t non_cri_hit_count = 0;

static int hit_channel = 0;
static float cur_deg = 0.0f;
static uint8_t cur_abs_dir = 0x00;

static uint8_t prev_mask = 0;
static uint8_t prev_motor_ctrl = 0xFF;

static uint8_t cur_mnq_rst_cmd = 0;

static bool motor_on_state = false;

// --------------------- D9 absolute encoder target control ---------------------
#define D9_TARGET_UP_DEG              (96.8f)
#define D9_TARGET_DOWN_DEG            (12.0f)  // 앞 다운 각도 170

#define D9_UP_TOL_DEG                 (1.0f)
#define D9_DOWN_TOL_DEG               (2.5f)

// 일반 UP/DOWN 동작 제한 범위
#define D9_MIN_LIMIT_DEG              (5.0f)
#define D9_MAX_LIMIT_DEG              (105.0f)

// RST 기준: UP 상태 96.8도 ± 1.5도
#define D9_RST_TOL_DEG                (1.5f)
#define D9_UP_NORMAL_MIN_DEG          (D9_TARGET_UP_DEG - D9_RST_TOL_DEG)  // 95.3도
#define D9_UP_NORMAL_MAX_DEG          (D9_TARGET_UP_DEG + D9_RST_TOL_DEG)  // 98.3도

// 일반 UP/DOWN 이동 시간
#define MOTOR_ARRIVE_TIME_S           (0.7f)

// RST 복구 이동 시간
#define MOTOR_RST_ARRIVE_TIME_S       (0.9f)    // 1.5f

#define MOTOR_MOVE_EXTRA_WAIT_MS      (800u)
#define MOTOR_CORRECTION_COUNT        (1u)

// MicroPython 실측 기준
// D9 degree 증가 방향 = CW, D9 degree 감소 방향 = CCW
#define D9_INCREASE_CMD_CW            (false)
#define D9_DECREASE_CMD_CW            (true)

typedef enum {
    AUTO_SEQ_IDLE = 0,
    AUTO_SEQ_WAIT_DOWN,
    AUTO_SEQ_WAIT_UP
} auto_seq_state_t;

static auto_seq_state_t auto_seq_state = AUTO_SEQ_IDLE;

// --------------------- Common set ---------------------
#define DATA_BITS                       (8)
#define STOP_BITS                       (1)
#define PARITY                          (UART_PARITY_NONE)

static char buffer[128];

#define PICO_SYS_CLK_kHz                (125000)            // 125000 kHz
#define PICO_SYS_CLK                    (125000000)

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
#define SA_MODE_SET_POSMODE      0x0B   // 위치제어 모드 설정 (0x00 절대, 0x01 상대)
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
#define SA_REQ_ABS_ENC      0xA9   // 절대 엔코더 피드백 요청
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
#define SA_FEED_ABS_ENC     0xD9    // 절대 엔코더 피드백 응답
#define SA_FEED_FIRMWARE    0xFD    // 펌웨어 버전 응답

// --------------------- Common function ---------------------
void ConfigureUart_motor(void);
void ConfigureUart_sbc(void);
void ConfigureGpio_sensor(void);
void StartSignal(void);
void uart_timeout(void);

void Rx_uart_SBC(void);
void Tx_uart_SBC(int hit_channel, float cur_deg);

// --------------------- Motor function ---------------------
uint8_t calcChecksum(const uint8_t* frame, size_t len);
size_t  buildFrame(uint8_t dev_id, uint8_t mode, const uint8_t* data, size_t data_len, uint8_t* out, size_t out_max);
static void motor_send(uint8_t dev_id, uint8_t mode, const uint8_t* data, size_t data_len);
static void motor_clear_rx(void);
static bool read_one_motor_frame(uint8_t *frame, size_t *out_len, uint32_t timeout_ms);
static bool motor_request_abs_deg(uint8_t dev_id, uint8_t *dir, float *pos_deg);
static bool mnq_read_abs_update(void);
static bool motor_calc_d9_move(float current_deg, float target_deg, float *move_deg, bool *move_cw, float *predicted_deg);
static bool motor_move_to_d9_target(float target_deg, float tol_deg);
static float d9_to_signed_deg(uint8_t dir, float deg);

void motor_set_onoff(uint8_t dev_id, bool on);
void sa_set_posmode(uint8_t dev_id, bool absolute);
void motor_reset_pos(uint8_t dev_id);
void safe_stop(uint8_t dev_id);
static void motor_hold_stop(uint8_t dev_id);

int cmd_pos_speed(uint8_t dev_id, bool cw, float pos_deg, float spd_rpm);
int cmd_acc_pos(uint8_t dev_id, bool cw, float pos_deg, float arrive_s);
int cmd_acc_speed(uint8_t dev_id, bool cw, float spd_rpm, float arrive_s);
int cmd_open_loop(uint8_t dev_id, bool cw, float duty_percent);

void start_motor_timer(void);
bool check_motor_timer(uint32_t duration_ms);

static void motor_up(void);
static void motor_down(void);
static bool motor_rst_recover(void);

// --------------------- sensor function ---------------------
static void gpio_irq_callback(uint gpio, uint32_t events);

static void signal_detector_update(void);
static void signal_detector_reset(void);
static void clear_p2_history_irq_unsafe(uint32_t start_us);

static void detect_channel_ban(void);
void gain_pwm_set(uint8_t gain_pwm);

//--------------------- Main ---------------------
int main()
{
    set_sys_clock_khz(PICO_SYS_CLK_kHz, true);
    busy_wait_ms(100);

    ConfigureUart_sbc();
    sleep_ms(10);

    ConfigureUart_motor();
    sleep_ms(10);

    ConfigureGpio_sensor();
    sleep_ms(10);

    StartSignal();
    last_uart_time = get_absolute_time();
    sleep_ms(10);

    // 절대엔코더 값을 읽고, 차이값만큼 상대 위치제어로 움직이기
    sa_set_posmode(MOTOR_ID, false);   // false = 상대 위치제어 모드
    sleep_ms(80);

    motor_set_onoff(MOTOR_ID, false);
    sleep_ms(120);

    while (true) {
        tight_loop_contents();
        uart_timeout();

        signal_detector_update();   // 감지는 계속
        /*
        Receive_Data[0] → 0 = 제어 off, 1 = on (0이 들어올 일은 없음)
        Receive_Data[1] → 0 = 자동(자동 모터 제어), 1 = 수동 UP, 2 = 수동 DOWN, 3 = target_mode
        Receive_Data[2] → 0 = 1 채널 on, 1 = 1 채널 off
        Receive_Data[3] → 0 = 2 채널 on, 1 = 2 채널 off
        Receive_Data[4] → 0 = 3 채널 on, 1 = 3 채널 off
        Receive_Data[5] → 0 ~ 254 = Gain PWM 값 적용 (감지 센서 민감도)
        Receive_Data[6] → 0 ~ 9 치명상
        Receive_Data[7] → 0 ~ 9 비치명상
        Receive_Data[8] mnq_rst → 0 = None, 1 = UP 복귀
        */

        if (Receive_Done > 0) {
            Receive_Done = 0;
            last_uart_time = get_absolute_time();

            bool rst_handled = false;

            if (Receive_Data[0] == 0) {
                control_signal = false;
                prev_motor_ctrl = 0xFF;
                signal_detector_reset();
                motor_hold_stop(MOTOR_ID);

                motor_on_state = false;
            }
            else {
                if (!motor_on_state) {
                    motor_set_onoff(MOTOR_ID, true);
                    sleep_ms(80);
                    motor_on_state = true;
                    motor_hold_stop(MOTOR_ID);
                }

                detect_channel_ban();

                if (Receive_Data[5] != current_gain_pwm) {
                    gain_pwm_set(Receive_Data[5]);
                }

                if (Receive_Data[6] != cur_cri_set) {
                    cur_cri_set = Receive_Data[6];
                    cri_hit_count = 0;
                }

                if (Receive_Data[7] != cur_non_cri_set) {
                    cur_non_cri_set = Receive_Data[7];
                    non_cri_hit_count = 0;
                }

                if (Receive_Data[8] != cur_mnq_rst_cmd) {
                    cur_mnq_rst_cmd = Receive_Data[8];

                    if (cur_mnq_rst_cmd == 1) {
                        control_signal = false;
                        auto_seq_state = AUTO_SEQ_IDLE;
                        gpio_put(LED, 0);
                        signal_detector_reset();

                        cri_hit_count = 0;
                        non_cri_hit_count = 0;

                        (void)motor_rst_recover();

                        prev_motor_ctrl = Receive_Data[1];
                        rst_handled = true;
                    }
                }

                if (!rst_handled) {
                    if (Receive_Data[1] == 0) {
                        // auto
                        switch (auto_seq_state) {
                        case AUTO_SEQ_IDLE:
                            control_signal = true;

                            if ((cur_cri_set > 0 && cri_hit_count >= cur_cri_set) ||
                                (cur_non_cri_set > 0 && non_cri_hit_count >= cur_non_cri_set)) {

                                control_signal = false;

                                motor_down();
                                gpio_put(LED, 1);

                                start_motor_timer();
                                auto_seq_state = AUTO_SEQ_WAIT_DOWN;
                            }
                            break;

                        case AUTO_SEQ_WAIT_DOWN:
                            control_signal = false;

                            if (check_motor_timer(2000)) {
                                motor_up();

                                start_motor_timer();
                                auto_seq_state = AUTO_SEQ_WAIT_UP;
                            }
                            break;

                        case AUTO_SEQ_WAIT_UP:
                            control_signal = false;

                            if (check_motor_timer(1000)) {
                                cri_hit_count = 0;
                                non_cri_hit_count = 0;

                                gpio_put(LED, 0);
                                signal_detector_reset();

                                auto_seq_state = AUTO_SEQ_IDLE;
                                control_signal = true;
                            }
                            break;

                        default:
                            auto_seq_state = AUTO_SEQ_IDLE;
                            control_signal = true;
                            break;
                        }

                        prev_motor_ctrl = 0;
                    }
                    else if (Receive_Data[1] == 1 || Receive_Data[1] == 2) {
                        // manual
                        // 같은 명령이 10ms 주기로 반복 수신되어도 1회만 동작
                        signal_detector_reset();

                        if (Receive_Data[1] == 1) {
                            auto_seq_state = AUTO_SEQ_IDLE;
                            gpio_put(LED, 0);

                            control_signal = false;

                            if (prev_motor_ctrl != 1) {
                                motor_up();
                                prev_motor_ctrl = 1;
                            }
                        }
                        else if (Receive_Data[1] == 2) {
                            auto_seq_state = AUTO_SEQ_IDLE;
                            gpio_put(LED, 0);

                            control_signal = false;

                            if (prev_motor_ctrl != 2) {
                                motor_down();
                                prev_motor_ctrl = 2;
                            }
                        }
                    }
                    else if (Receive_Data[1] == 3) {
                        // target mode
                        // 추후 상태머신 추가 예정. 현재는 감지만 유지하고 자동 복귀는 하지 않음.
                        prev_motor_ctrl = 3;
                    }
                    else {
                        prev_motor_ctrl = 0xFF;
                    }
                }
            }

            // D9 읽기에 실패하면 마지막 cur_deg 값을 전송
            (void)mnq_read_abs_update();
            Tx_uart_SBC(hit_channel, cur_deg);

            // 송신 후 0으로 복귀
            hit_channel = 0;
        }

        busy_wait_ms(1);
    }

    return 0;
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
    snprintf(buffer, sizeof(buffer), "%d, %.2f\n", hit_channel, cur_deg);
    uart_puts(UART_ID_SBC, buffer);
}

// mtoorsbc와 통신이 끊길 경우 정지
void uart_timeout(){
    if (absolute_time_diff_us(last_uart_time, get_absolute_time()) >= check_uart_time_out) {
        motor_hold_stop(MOTOR_ID);
        motor_on_state = false;
        signal_detector_reset();
    }
}

void StartSignal(void)
{
    gpio_put(LED, 1);
    sleep_ms(1000);
    gpio_put(LED, 0);
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

static void motor_down(void)
{
    if (!mnq_read_abs_update()) {
        return;
    }

    if (cur_deg <= D9_UP_NORMAL_MIN_DEG) {
        motor_hold_stop(MOTOR_ID);
        return;
    }
    // D9 signed angle 기준 DOWN = 170도
    (void)motor_move_to_d9_target(D9_TARGET_DOWN_DEG, D9_DOWN_TOL_DEG);
}

static void motor_up(void)
{
    // D9 signed angle 기준 UP = 90도
    (void)motor_move_to_d9_target(D9_TARGET_UP_DEG, D9_UP_TOL_DEG);
}

static bool motor_rst_recover(void)
{
    /*
     * RST 복구 명령
     *
     * 목적:
     * - 일반 UP/DOWN 제한 범위를 벗어나도 UP 상태(90도)로 복귀
     * - 360도 최단거리 계산을 사용하지 않음
     * - signed D9 기준 diff만 사용
     *
     * 방향:
     * diff = D9_TARGET_UP_DEG - current
     * diff > 0  → signed D9 값을 증가시켜야 함 → CW
     * diff < 0  → signed D9 값을 감소시켜야 함 → CCW
     */

    sa_set_posmode(MOTOR_ID, false);   // false = 상대 위치제어 모드
    sleep_ms(30);

    motor_set_onoff(MOTOR_ID, true);
    sleep_ms(80);

    if (!mnq_read_abs_update()) {
        return false;
    }

    float current = cur_deg;
    float diff = D9_TARGET_UP_DEG - current;
    float err = fabsf(diff);

    // 이미 UP 정상 범위(90 ± 1.5도)에 있으면 RST 무시
    if (err <= D9_RST_TOL_DEG) {
        return true;
    }

    bool move_cw = false;
    float move_deg = 0.0f;

    if (diff > 0.0f) {
        // signed D9 값을 증가시켜야 함
        move_cw = D9_INCREASE_CMD_CW;
        move_deg = diff;
    }
    else {
        // signed D9 값을 감소시켜야 함
        move_cw = D9_DECREASE_CMD_CW;
        move_deg = -diff;
    }

    if (move_deg < 0.01f) {
        return true;
    }

    // RST는 한 번에 UP 목표각까지 복구
    cmd_acc_pos(MOTOR_ID, move_cw, move_deg, MOTOR_RST_ARRIVE_TIME_S);

    sleep_ms((uint32_t)(MOTOR_RST_ARRIVE_TIME_S * 1000.0f) + MOTOR_MOVE_EXTRA_WAIT_MS); // 2.5초

    // 복구 후 현재 D9 재확인
    if (!mnq_read_abs_update()) {
        return false;
    }

    if (fabsf(cur_deg - D9_TARGET_UP_DEG) <= D9_RST_TOL_DEG) {
        return true;
    }

    // 복구 실패 시 모터 보호를 위해 정지
    motor_hold_stop(MOTOR_ID);
    return false;
}

// --------------------- Motor Utility ---------------------
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

uint8_t calcChecksum(const uint8_t* frame, size_t len)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        if (i == 0 || i == 1 || i == 4) continue;
        sum += frame[i];
    }
    return (uint8_t)(~(sum & 0xFF));
}

size_t buildFrame(uint8_t dev_id, uint8_t mode, const uint8_t* data, size_t data_len, uint8_t* out, size_t out_max)
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

static void motor_clear_rx(void)
{
    while (uart_is_readable(UART_ID_MOTOR)) {
        (void)uart_getc(UART_ID_MOTOR);
    }
}

static float d9_to_signed_deg(uint8_t dir, float deg)
{
    (void)dir;
    return deg;
}

static bool mnq_read_abs_update(void)
{
    uint8_t dir = 0;
    float deg = 0.0f;

    if (!motor_request_abs_deg(MOTOR_ID, &dir, &deg)) {
        return false;
    }

    cur_abs_dir = dir;
    cur_deg = d9_to_signed_deg(dir, deg);
    return true;
}

static bool motor_calc_d9_move(float current_deg, float target_deg, float *move_deg, bool *move_cw, float *predicted_deg)
{
    if (move_deg == NULL || move_cw == NULL || predicted_deg == NULL) {
        return false;
    }

    // signed angle 기준 범위 확인
    // 예: -15도 ~ +90도
    if (current_deg < D9_MIN_LIMIT_DEG || current_deg >= D9_MAX_LIMIT_DEG) {
        return false;
    }

    if (target_deg < D9_MIN_LIMIT_DEG || target_deg >= D9_MAX_LIMIT_DEG) {
        return false;
    }

    float diff = target_deg - current_deg;

    if (diff > 0.0f) {
        // signed D9 값을 증가시켜야 함
        *move_cw = D9_INCREASE_CMD_CW;
        *move_deg = diff;
    }
    else {
        // signed D9 값을 감소시켜야 함
        *move_cw = D9_DECREASE_CMD_CW;
        *move_deg = -diff;
    }

    *predicted_deg = current_deg + diff;

    if (*predicted_deg < D9_MIN_LIMIT_DEG || *predicted_deg >= D9_MAX_LIMIT_DEG) {
        return false;
    }

    return true;
}

static bool motor_move_to_d9_target(float target_deg, float tol_deg)
{
    uint32_t max_attempt = 1u + MOTOR_CORRECTION_COUNT;

    sa_set_posmode(MOTOR_ID, false);
    sleep_ms(30);
    motor_set_onoff(MOTOR_ID, true);
    sleep_ms(80);

    for (uint32_t attempt = 0; attempt < max_attempt; attempt++) {
        if (!mnq_read_abs_update()) {
            return false;
        }

        float current = cur_deg;
        float diff = target_deg - current;
        float err = fabsf(diff);

        if (current < D9_MIN_LIMIT_DEG || current >= D9_MAX_LIMIT_DEG) {
            motor_hold_stop(MOTOR_ID);
            return false;
        }

        if (target_deg < D9_MIN_LIMIT_DEG || target_deg >= D9_MAX_LIMIT_DEG) {
            motor_hold_stop(MOTOR_ID);
            return false;
        }

        if (err <= tol_deg) {
            return true;
        }

        float move_deg = 0.0f;
        bool move_cw = false;
        float predicted = 0.0f;

        if (!motor_calc_d9_move(current, target_deg, &move_deg, &move_cw, &predicted)) {
            motor_hold_stop(MOTOR_ID);
            return false;
        }

        if (predicted < D9_MIN_LIMIT_DEG || predicted >= D9_MAX_LIMIT_DEG) {
            motor_hold_stop(MOTOR_ID);
            return false;
        }

        if (move_deg < 0.01f) {
            return true;
        }

        cmd_acc_pos(MOTOR_ID, move_cw, move_deg, MOTOR_ARRIVE_TIME_S);
        sleep_ms((uint32_t)(MOTOR_ARRIVE_TIME_S * 1000.0f) + MOTOR_MOVE_EXTRA_WAIT_MS); // 2.5초
    }

    if (!mnq_read_abs_update()) {
        return false;
    }

    if (cur_deg < D9_MIN_LIMIT_DEG || cur_deg >= D9_MAX_LIMIT_DEG) {
        motor_hold_stop(MOTOR_ID);
        return false;
    }

    return (fabsf(cur_deg - target_deg) <= tol_deg);
}

void safe_stop(uint8_t dev_id)
{
    // 감속 정지 후 OFF
    cmd_acc_speed(dev_id, false, 0.0f, 0.2f);
    motor_set_onoff(dev_id, false);
}

static void motor_hold_stop(uint8_t dev_id)
{
    // 토크 유지 제어 ON 이후 속도 0 지령을 보내고, 제어 OFF는 하지 않음
    cmd_acc_speed(dev_id, false, 0.0f, 0.2f);
    sleep_ms(100);
}

// ON/OFF: 0x00=On, 0x01=Off
void motor_set_onoff(uint8_t dev_id, bool on)
{
    uint8_t d[1] = { (uint8_t)(on ? 0x00 : 0x01) };
    motor_send(dev_id, SA_MODE_SET_ONOFF, d, 1);
}

// 상대각도 위치 초기화 사용 X
void motor_reset_pos(uint8_t dev_id)
{
    uint8_t d[1] = { 0x00 };
    motor_send(dev_id, SA_MODE_RESET_POS, d, 1);
}

// 모드 설정
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

static bool motor_request_abs_deg(uint8_t dev_id, uint8_t *dir, float *pos_deg)
{
    uint8_t req[8];
    size_t n = buildFrame(dev_id, SA_REQ_ABS_ENC, NULL, 0, req, sizeof(req));
    if (n == 0) return false;

    motor_clear_rx();
    uart_write_blocking(UART_ID_MOTOR, req, n);

    uint8_t rx[32];
    size_t rx_len = 0;

    if (!read_one_motor_frame(rx, &rx_len, 80)) {
        return false;
    }

    /*
     * 기대 응답:
     * FF FE ID 05 CHK D9 DIR ENC_H ENC_L
     */
    if (rx_len != 9) return false;
    if (rx[2] != dev_id) return false;
    if (rx[5] != SA_FEED_ABS_ENC) return false;

    uint16_t raw = ((uint16_t)rx[7] << 8) | rx[8];

    *dir = rx[6];
    *pos_deg = (float)raw / 100.0f;   // 0.01°

    return true;
}

static absolute_time_t motor_timer_start_time;

void start_motor_timer(void)
{
    motor_timer_start_time = get_absolute_time();
}

bool check_motor_timer(uint32_t duration_ms)
{
    int64_t diff_us = absolute_time_diff_us(motor_timer_start_time, get_absolute_time());
    return diff_us >= ((int64_t)duration_ms * 1000ll);
}

// --------------------- Sensor ---------------------
void ConfigureGpio_sensor(void)
{
    const float pwm_freq_hz = 5000.0f;

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    gpio_init(DETECT_1_P1);
    gpio_set_dir(DETECT_1_P1, GPIO_IN);
    gpio_pull_down(DETECT_1_P1);
    gpio_set_irq_enabled_with_callback(DETECT_1_P1, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true, &gpio_irq_callback);

    gpio_init(DETECT_2_P2);
    gpio_set_dir(DETECT_2_P2, GPIO_IN);
    gpio_pull_down(DETECT_2_P2);
    gpio_set_irq_enabled(DETECT_2_P2, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true);

    // gpio_init(DETECT_3);
    // gpio_set_dir(DETECT_3, GPIO_IN);
    // gpio_pull_down(DETECT_3);
    // gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true);

    gpio_set_function(GAIN, GPIO_FUNC_PWM);
    uint gain_slice_num = pwm_gpio_to_slice_num(GAIN);

    pwm_set_wrap(gain_slice_num, 255);

    float div = (float)PICO_SYS_CLK / ((255 + 1.0f) * pwm_freq_hz);
    if (div < 1.0f) div = 1.0f;
    if (div > 255.0f) div = 255.0f;

    pwm_set_clkdiv(gain_slice_num, div);
    pwm_set_gpio_level(GAIN, 0);

    pwm_set_enabled(gain_slice_num, true);

    pwm_set_gpio_level(GAIN, default_gain_pwm);
}

static void gpio_irq_callback(uint gpio, uint32_t events)
{
    uint32_t now = time_us_32();

    if (gpio == DETECT_1_P1) {
        // P1 RISE
        if (events & GPIO_IRQ_EDGE_RISE) {
            if (isr_p1_high) {
                return;   // 중복 RISE 무시
            }

            uint32_t seq = ++sig_event_seq;

            isr_p1_rise_us = now;
            isr_p1_rise_seq = seq;
            isr_p1_high = true;
            /*
            * 중요:
            * P1 RISE 순간에 P2 history를 초기화해야 함
            * main loop에서 나중에 초기화하면 이미 잡힌 P2 pulse를 지워버릴 수 있음
            */
            clear_p2_history_irq_unsafe(now);
        }

        // P1 FALL
        if (events & GPIO_IRQ_EDGE_FALL) {
            if (!isr_p1_high) {
                return;   // 중복 FALL 무시
            }

            uint32_t seq = ++sig_event_seq;

            isr_p1_fall_us = now;
            isr_p1_fall_seq = seq;
            isr_p1_high = false;
        }
    }
    else if (gpio == DETECT_2_P2) {
        // P2 RISE
        if (events & GPIO_IRQ_EDGE_RISE) {
            if (isr_p2_high) {
                return;   // 중복 RISE 무시
            }

            uint32_t seq = ++sig_event_seq;

            uint8_t idx = isr_p2_write_idx;
            isr_p2_write_idx = (uint8_t)((isr_p2_write_idx + 1u) % P2_HIST_SIZE);
            isr_p2_active_idx = idx;

            isr_p2_hist[idx].rise_us = now;
            isr_p2_hist[idx].fall_us = 0;
            isr_p2_hist[idx].rise_seq = seq;
            isr_p2_hist[idx].fall_seq = 0;
            isr_p2_hist[idx].valid = true;
            isr_p2_hist[idx].low_seen = false;

            isr_p2_high = true;
        }

        // P2 FALL
        if (events & GPIO_IRQ_EDGE_FALL) {
            if (!isr_p2_high) {
                return;   // 중복 FALL 무시
            }

            uint32_t seq = ++sig_event_seq;

            if (isr_p2_active_idx != P2_ACTIVE_NONE) {
                uint8_t idx = isr_p2_active_idx;

                isr_p2_hist[idx].fall_us = now;
                isr_p2_hist[idx].fall_seq = seq;
                isr_p2_hist[idx].low_seen = true;

                isr_p2_active_idx = P2_ACTIVE_NONE;
            }

            isr_p2_high = false;
        }
    }
}

static inline bool time_after_eq_u32(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) >= 0;
}

static inline bool time_reached_u32(uint32_t now, uint32_t target)
{
    return (int32_t)(now - target) >= 0;
}

static void signal_get_snapshot(signal_snapshot_t *s)
{
    uint32_t irq = save_and_disable_interrupts();

    s->p1_rise_us = isr_p1_rise_us;
    s->p1_fall_us = isr_p1_fall_us;
    s->p1_rise_seq = isr_p1_rise_seq;
    s->p1_fall_seq = isr_p1_fall_seq;
    s->p1_high = isr_p1_high;

    for (int i = 0; i < P2_HIST_SIZE; i++) {
        s->p2_hist[i].rise_us = isr_p2_hist[i].rise_us;
        s->p2_hist[i].fall_us = isr_p2_hist[i].fall_us;
        s->p2_hist[i].rise_seq = isr_p2_hist[i].rise_seq;
        s->p2_hist[i].fall_seq = isr_p2_hist[i].fall_seq;
        s->p2_hist[i].valid = isr_p2_hist[i].valid;
        s->p2_hist[i].low_seen = isr_p2_hist[i].low_seen;
    }

    restore_interrupts(irq);
}

static void clear_p2_history_irq_unsafe(uint32_t start_us)
{
    for (int i = 0; i < P2_HIST_SIZE; i++) {
        isr_p2_hist[i].rise_us = 0;
        isr_p2_hist[i].fall_us = 0;
        isr_p2_hist[i].rise_seq = 0;
        isr_p2_hist[i].fall_seq = 0;
        isr_p2_hist[i].valid = false;
        isr_p2_hist[i].low_seen = false;
    }

    isr_p2_write_idx = 0;
    isr_p2_active_idx = P2_ACTIVE_NONE;

    /*
     * P1 RISE 순간에 P2가 이미 High라면,
     * P2가 P1보다 먼저 올라와서 아직 유지 중인 것으로 보고
     * 열린 pulse를 하나 만들어두기
     */
    isr_p2_high = gpio_get(DETECT_2_P2);

    if (isr_p2_high) {
        uint32_t seq = ++sig_event_seq;

        uint8_t idx = 0;
        isr_p2_hist[idx].rise_us = start_us;
        isr_p2_hist[idx].fall_us = 0;
        isr_p2_hist[idx].rise_seq = seq;
        isr_p2_hist[idx].fall_seq = 0;
        isr_p2_hist[idx].valid = true;
        isr_p2_hist[idx].low_seen = false;

        isr_p2_write_idx = 1;
        isr_p2_active_idx = idx;
    }
}

static bool p2_was_high_at_time(const signal_snapshot_t *s, uint32_t check_us)
{
    for (int i = 0; i < P2_HIST_SIZE; i++) {
        const p2_pulse_t *p = &s->p2_hist[i];

        if (!p->valid) {
            continue;
        }

        // check_us가 P2 rise 이후인지 확인
        if (!time_after_eq_u32(check_us, p->rise_us)) {
            continue;
        }

        // 아직 fall이 기록되지 않았다면 현재까지 High로 판단
        if (!p->low_seen) {
            return true;
        }

        // fall_us가 check_us 이후 또는 같은 시점이면 check_us 당시 High
        if (time_after_eq_u32(p->fall_us, check_us)) {
            return true;
        }
    }

    return false;
}

static void signal_detector_reset(void)
{
    signal_snapshot_t s;
    signal_get_snapshot(&s);

    sig_state = SIG_IDLE;
    active_p1_rise_us = 0;
    active_p1_fall_us = 0;
    active_p1_rise_seq = 0;
    sig_cooldown_start_us = 0;

    // reset 전에 이미 들어온 P1은 처리하지 않음
    last_used_p1_rise_seq = s.p1_rise_seq;
}

static void signal_detector_update(void)
{
    /*
     * control_signal이 false면 감지하지 않음
     * first_channel_ban은 기존 채널 무시 정책 유지
     */
    if (!control_signal || first_channel_ban) {
        signal_detector_reset();
        return;
    }

    signal_snapshot_t s;
    signal_get_snapshot(&s);

    uint32_t now = time_us_32();

    switch (sig_state) {
    case SIG_IDLE:
        if (s.p1_rise_seq != 0 &&
            s.p1_rise_seq != last_used_p1_rise_seq) {

            active_p1_rise_us = s.p1_rise_us;
            active_p1_rise_seq = s.p1_rise_seq;
            last_used_p1_rise_seq = s.p1_rise_seq;

            sig_state = SIG_CONFIRM_P1;
        }
        break;

    case SIG_CONFIRM_P1:
        /*
         * P1 상승 후 이미 하강이 들어온 경우:
         * High 폭으로 유효성 판단
         */
        if (s.p1_fall_seq > active_p1_rise_seq) {
            uint32_t width = s.p1_fall_us - active_p1_rise_us;

            if (width >= P1_CONFIRM_US) {
                active_p1_fall_us = s.p1_fall_us;
                sig_state = SIG_WAIT_P2_JUDGE;
            }
            else {
                sig_state = SIG_IDLE;
            }

            break;
        }

        /*
         * 아직 P1이 High 상태이고 5ms가 지났다면:
         * 유효 P1로 인정하고 fall 대기
         */
        if (time_reached_u32(now, active_p1_rise_us + P1_CONFIRM_US)) {
            if (s.p1_high) {
                sig_state = SIG_WAIT_P1_FALL;
            }
            else {
                sig_state = SIG_IDLE;
            }
        }
        break;

    case SIG_WAIT_P1_FALL:
        if (s.p1_fall_seq > active_p1_rise_seq) {
            active_p1_fall_us = s.p1_fall_us;
            sig_state = SIG_WAIT_P2_JUDGE;
        }
        break;

    case SIG_WAIT_P2_JUDGE:
        /*
         * P1 fall + 2ms까지 지난 뒤 판정
         */
        if (time_reached_u32(now, active_p1_fall_us + P2_CHECK2_US)) {
            uint32_t t_check1 = active_p1_fall_us + P2_CHECK1_US;
            uint32_t t_check2 = active_p1_fall_us + P2_CHECK2_US;

            bool p2_c1 = p2_was_high_at_time(&s, t_check1);
            bool p2_c2 = p2_was_high_at_time(&s, t_check2);

            if ((p2_c1 || p2_c2) && !second_channel_ban) {
                // critical
                hit_channel = 2;
                cri_hit_count += 1;
            }
            // if (p2_c1 && p2_c2 && !second_channel_ban) {
            //     // critical
            //     hit_channel = 2;
            //     cri_hit_count += 1;
            // }
            else {
                // non-critical
                // DETECT_1과 DETECT_3을 하나로 묶은 P1 계열로 처리
                hit_channel = 1;
                non_cri_hit_count += 1;
            }

            sig_cooldown_start_us = now;
            sig_state = SIG_COOLDOWN;
        }
        break;

    case SIG_COOLDOWN:
        if ((uint32_t)(now - sig_cooldown_start_us) >= DETECT_COOLDOWN_US) {
            signal_get_snapshot(&s);
            last_used_p1_rise_seq = s.p1_rise_seq;
            sig_state = SIG_IDLE;
        }
        break;

    default:
        sig_state = SIG_IDLE;
        break;
    }
}

static void detect_channel_ban(void)
{
    /*
     * 현재 하드웨어는 P1, P2 두 입력만 사용
     * SBC 데이터는 ch1/ch2/ch3 구조이므로 P1 계열은 ch1 또는 ch3가 모두 off일 때만 ban 처리
     */
    first_channel_ban = (Receive_Data[2] == 1 && Receive_Data[4] == 1);
    second_channel_ban = (Receive_Data[3] == 1);
}

void gain_pwm_set(uint8_t gain_pwm)
{
    if (gain_pwm > 254u) {
        gain_pwm = 254u;
    }

    current_gain_pwm = gain_pwm;
    pwm_set_gpio_level(GAIN, current_gain_pwm);
}