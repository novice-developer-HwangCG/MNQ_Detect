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
// NM-2 (NuriMotor-2)

#define LED             PICO_DEFAULT_LED_PIN

// --------------------- MOTOR UART 0 ---------------------
#define UART_TX_PIN_MOTOR              (0)
#define UART_RX_PIN_MOTOR              (1)

#define UART_ID_MOTOR                  (uart0)
#define MOTOR_BAUD_RATE                (9600)

#define MOTOR_ID                       0x00     // 공장 초기 값

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

// --------------------- MNQ motor command / absolute feedback 기준 ---------------------
/*
 * 주의:
 * CMD_MOVE_TO_xxx_DEG는 모터 제어 명령 0x02에 넣는 위치 명령값
 * ABS_xxx_DEG는 절대엔코더 D9 피드백으로 실제 기구 상태를 판단하는 값
 *
 * 최신 테스트 결과 기준:
 *
 * 실제 기구 DOWN 상태:
 *   제어 명령  : CW, 83.0도
 *   피드백 상태: CW, 약 0.46~0.54도
 *
 * 실제 기구 UP 상태:
 *   제어 명령  : CCW, 0.5도
 *   피드백 상태: CW, 약 82.50~82.55도
 */
#define CMD_MOVE_TO_DOWN_DIR_CW        true     // DOWN 방향, 뒤로 넘어가기
#define CMD_MOVE_TO_DOWN_DEG           83.0f    // -> 기존 90도에서 83도로 변경

#define CMD_MOVE_TO_UP_DIR_CW          false    // UP 방향, 앞으로 올리기
#define CMD_MOVE_TO_UP_DEG             0.5f

#define CMD_MOVE_ARRIVE_SEC            0.9f
#define CMD_CORRECT_ARRIVE_SEC         1.5f

#define MOTOR_MOVE_WAIT_MS             1300u
#define AUTO_HOLD_WAIT_MS              2000u

#define ABS_UP_DIR                     0x01    // CW
#define ABS_UP_CENTER_DEG              82.5f
#define ABS_UP_MIN_DEG                 80.0f
#define ABS_UP_MAX_DEG                 84.0f

#define ABS_DOWN_DIR                   0x01    // CW
#define ABS_DOWN_CENTER_DEG            0.5f     // 기존 8.0f    
#define ABS_DOWN_MIN_DEG               0.3f     // 기존 7.0f
#define ABS_DOWN_MAX_DEG               0.7f     // 기존 9.0f

#define RST_UP_IGNORE_DEG              3.0f

#define MOTOR_PHASE_IDLE                 0
#define MOTOR_PHASE_AUTO_TO_DOWN         1
#define MOTOR_PHASE_AUTO_DOWN_HOLD       2
#define MOTOR_PHASE_AUTO_TO_UP           3
#define MOTOR_PHASE_AUTO_UP_HOLD         4
#define MOTOR_PHASE_TARGET_WAIT_SENSOR   5
#define MOTOR_PHASE_TARGET_TO_DOWN       6

#define TARGET_WAIT_SENSOR_MS            5000u

typedef enum {
    MNQ_POS_UNKNOWN = 0,
    MNQ_POS_UP,
    MNQ_POS_DOWN
} mnq_position_t;

volatile bool acting_auto = false;

static bool prev_control_signal = false;
static uint8_t motor_phase = MOTOR_PHASE_IDLE;
static uint32_t motor_timer_start = 0;
static uint8_t prev_manual_cmd = 255;
static uint8_t prev_mnq_rst_cmd = 0;

static float cur_deg = 0.0f;
static uint8_t cur_abs_dir = 0xFF;
static mnq_position_t cur_mnq_pos = MNQ_POS_UNKNOWN;

static bool abs_report_pending = false;
static uint32_t abs_report_timer_start = 0;
static bool rst_zero_after_move = false;
static bool rst_restore_absolute_mode = false;

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

static bool motor_active = false;

volatile bool first_channel_ban = false;
volatile bool second_channel_ban = false;

static uint8_t default_gain_pwm = 65;
static uint8_t current_gain_pwm = 65;

volatile uint8_t cur_cri_set = 0;
volatile uint8_t cri_hit_count = 0;

volatile uint8_t cur_non_cri_set = 0;
volatile uint8_t non_cri_hit_count = 0;

static int hit_channel = 0;

static uint8_t prev_mask = 0;

// =====================================================
// Signal detect logic - P1/P2 timestamp based
// =====================================================

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
uint8_t calcChecksum(const uint8_t* frame, size_t len);
size_t  buildFrame(uint8_t dev_id, uint8_t mode, const uint8_t* data, size_t data_len, uint8_t* out, size_t out_max);
static void motor_send(uint8_t dev_id, uint8_t mode, const uint8_t* data, size_t data_len);
static void motor_clear_rx(void);

void motor_set_onoff(uint8_t dev_id, bool on);
void sa_set_posmode(uint8_t dev_id, bool absolute);
void motor_reset_pos(uint8_t dev_id);
void safe_stop(uint8_t dev_id);

int  cmd_pos_speed(uint8_t dev_id, bool cw, float pos_deg, float spd_rpm);
int  cmd_acc_pos(uint8_t dev_id, bool cw, float pos_deg, float arrive_s);
int  cmd_acc_speed(uint8_t dev_id, bool cw, float spd_rpm, float arrive_s);
int  cmd_open_loop(uint8_t dev_id, bool cw, float duty_percent);

static bool read_one_motor_frame(uint8_t *frame, size_t *out_len, uint32_t timeout_ms);
static bool motor_request_abs_deg(uint8_t dev_id, uint8_t *dir, float *pos_deg);
static void mnq_rst_move_to_up_by_abs_delta(float arrive_s);

// --------------------- MNQ position function ---------------------
static mnq_position_t mnq_classify(uint8_t dir, float deg);
static bool mnq_read_abs_update(void);
static void mnq_move_to_down(float arrive_s);
static void mnq_move_to_up(float arrive_s);

// --------------------- SBC function ---------------------
void Rx_uart_SBC(void);
void Tx_uart_SBC(int hit_channel, float cur_deg);

static void signal_detector_update(void);
static void signal_detector_reset(void);
static void clear_p2_history_irq_unsafe(uint32_t start_us);

static void detect_channel_ban(void);
void gain_pwm_set(uint8_t gain_pwm);

void start_motor_timer();
bool check_motor_timer(uint32_t duration);

void start_abs_report_timer(void);
void check_abs_report_timer(void);

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

    /*
     * 부팅 직후 절대엔코더 D9 피드백으로 현재 기구 상태 확인.
     * 기존 위치 초기화(0x0C)는 사용하지 않음.
     */
    motor_set_onoff(MOTOR_ID, false);
    sleep_ms(120);

    sa_set_posmode(MOTOR_ID, true);
    sleep_ms(300);

    while (true)
    {
        tight_loop_contents();
        uart_timeout();

        // 제어 신호가 들어오면 신호 감지하기 (수동 상태에서도 신호는 감지하되 모터 동작은 못하게 막기)
        signal_detector_update();

        /*
        Receive_Data[0] → 0 = 제어 off, 1 = on
        Receive_Data[1] → 0 = 자동(자동 모터 제어), 1 = 수동 UP, 2 = 수동 DOWN, 3 = target_mode
        Receive_Data[2] → 0 = 1 채널 on, 1 = 1 채널 off
        Receive_Data[3] → 0 = 2 채널 on, 1 = 2 채널 off
        Receive_Data[4] → 0 = 3 채널 on, 1 = 3 채널 off
        Receive_Data[5] → 0 ~ 254 = Gain PWM 값 적용 (감지 센서 민감도)
        Receive_Data[6] → 0 ~ 9 치명상
        Receive_Data[7] → 0 ~ 9 비치명상
        Receive_Data[8] mnq_rst → 0 = None, 1 = UP 복귀
        Receive_Data[9] jb_cmd → 현재 절대엔코더 기준 제어에서는 사용하지 않음
        */
        
        if (Receive_Done > 0){
            if (Receive_Data[0] == 0){
                control_signal = false;
                cri_hit_count = 0;
                non_cri_hit_count = 0;
                motor_phase = MOTOR_PHASE_IDLE;
                motor_active = false;
                hit_channel = 0;    // 모터 제어는 하지 않고 센서 신호는 보고 싶을 때 제거
                prev_manual_cmd = 255;
                prev_mnq_rst_cmd = 0;
                acting_auto = false;

                signal_detector_reset();
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
                    mnq_read_abs_update();
                }
            }

            detect_channel_ban();

            int report_hit = hit_channel;

            if (control_signal){
                detect_channel_ban();

                uint8_t mnq_rst_cmd = Receive_Data[8];
                bool rst_handled = false;

                if (Receive_Data[5] != current_gain_pwm) {
                    gain_pwm_set(Receive_Data[5]);
                }
                if (Receive_Data[6] != cur_cri_set) {
                    cur_cri_set = Receive_Data[6];
                }
                if (Receive_Data[7] != cur_non_cri_set) {
                    cur_non_cri_set = Receive_Data[7]; 
                }

                /*
                 * RST:
                 * 기존 위치 초기화(0x0C)가 아니라 D9 절대엔코더 기준으로 실제 UP 상태 복귀.
                 * UP 기준각 ±3도 이내면 이동하지 않음.
                 */
                if (mnq_rst_cmd != prev_mnq_rst_cmd) {
                    prev_mnq_rst_cmd = mnq_rst_cmd;

                    if (mnq_rst_cmd == 1) {
                        prev_manual_cmd = 1;
                        acting_auto = false;
                        motor_phase = MOTOR_PHASE_IDLE;
                        motor_active = false;

                        hit_channel = 0;
                        cri_hit_count = 0;
                        non_cri_hit_count = 0;

                        rst_handled = true;

                        if (mnq_read_abs_update()) {
                            Tx_uart_SBC(0, cur_deg);

                            if (cur_mnq_pos != MNQ_POS_UP) {
                                mnq_rst_move_to_up_by_abs_delta(CMD_MOVE_ARRIVE_SEC);
                                rst_zero_after_move = true;
                                start_abs_report_timer();
                            }
                            else {
                                // 이미 UP이면 바로 0도 초기화 가능
                                motor_set_onoff(MOTOR_ID, false);
                                sleep_ms(120);

                                motor_reset_pos(MOTOR_ID);
                                sleep_ms(300);

                                motor_set_onoff(MOTOR_ID, true);
                                sleep_ms(120);
                            }
                        }
                    }
                }

                // 자동
                if (!rst_handled && Receive_Data[1] == 0){
                    acting_auto = true;
                    prev_manual_cmd = 255;

                    // 모터 동작용 hit는 auto일 때만 사용
                    int hit_for_motor = hit_channel;
                    if (hit_for_motor != 0 && motor_phase == MOTOR_PHASE_IDLE) {
                        if ((cur_cri_set > 0 && cri_hit_count >= cur_cri_set) ||
                            (cur_non_cri_set > 0 && non_cri_hit_count >= cur_non_cri_set)) {
                            /*
                            * 자동 모드:
                            * 센서 감지 시 실제 DOWN 상태로 이동.
                            */
                            mnq_move_to_down(CMD_MOVE_ARRIVE_SEC);

                            start_motor_timer();
                            motor_phase = MOTOR_PHASE_AUTO_TO_DOWN;
                            gpio_put(LED, 0);

                            cri_hit_count = 0;
                            non_cri_hit_count = 0;
                        }
                    }
                }
                // 수동
                else if (!rst_handled && (Receive_Data[1] == 1 || Receive_Data[1] == 2)) {
                    if (acting_auto) {
                        motor_phase = MOTOR_PHASE_IDLE;
                        motor_active = false;
                    }

                    acting_auto = false;    // 자동 제어 중 'Receive_Data[2]'값이 들어올 경우 수동 제어에 대한 오동작 방지용

                    if (Receive_Data[1] != prev_manual_cmd){
                        prev_manual_cmd = Receive_Data[1];

                        /*
                         * 수동 UP/DOWN 명령:
                         * 먼저 D9 절대엔코더 피드백을 읽고 현재 위치를 확인.
                         * 이미 목표 상태이면 움직이지 않고 무시.
                         */
                        if (Receive_Data[1] == 1){
                            mnq_move_to_up(CMD_MOVE_ARRIVE_SEC);
                            start_abs_report_timer();
                        }
                        else if (Receive_Data[1] == 2){
                            if (motor_phase == MOTOR_PHASE_IDLE) {
                                mnq_move_to_down(CMD_MOVE_ARRIVE_SEC);
                                start_abs_report_timer();
                            }
                        }
                    }
                }
                else if (!rst_handled && Receive_Data[1] == 3){
                    /*
                    * target mode:
                    * 사용자가 수동 DOWN -> 수동 UP 후,
                    * UP 상태에서 Receive_Data[1] == 3을 보낸다고 가정.
                    *
                    * 5초 동안 센서 감지 대기
                    * 5초 안에 감지되면 DOWN 후 대기
                    * 5초 동안 감지 안 되어도 DOWN 후 대기
                    * 자동모드와 달리 다시 UP으로 복귀하지 않음
                    */
                    if (motor_phase == MOTOR_PHASE_IDLE && prev_manual_cmd != 3) {
                        prev_manual_cmd = 3;
                        acting_auto = false;

                        cri_hit_count = 0;
                        non_cri_hit_count = 0;
                        hit_channel = 0;

                        signal_detector_reset();

                        start_motor_timer();
                        motor_phase = MOTOR_PHASE_TARGET_WAIT_SENSOR;
                    }
                }
            }

            Tx_uart_SBC(report_hit, cur_deg);
            hit_channel = 0;
            last_uart_time = get_absolute_time();
            Receive_Done = 0;
        }

        // 자동: DOWN 이동 완료 -> D9 읽고 SBC 전송 -> 2초 대기 시작
        if (motor_phase == MOTOR_PHASE_AUTO_TO_DOWN) {
            if (check_motor_timer(MOTOR_MOVE_WAIT_MS)) {
                if (mnq_read_abs_update()) {
                    Tx_uart_SBC(0, cur_deg);
                }

                start_motor_timer();
                motor_phase = MOTOR_PHASE_AUTO_DOWN_HOLD;
                gpio_put(LED, 1);
            }
        }
        // 자동: DOWN 상태에서 2초 대기 끝 -> UP 복귀 시작
        else if (motor_phase == MOTOR_PHASE_AUTO_DOWN_HOLD) {
            if (check_motor_timer(AUTO_HOLD_WAIT_MS)){
                mnq_move_to_up(CMD_MOVE_ARRIVE_SEC);

                start_motor_timer();
                motor_phase = MOTOR_PHASE_AUTO_TO_UP;
            }
        }
        // 자동: UP 이동 완료 -> D9 읽고 SBC 전송 -> 2초 대기 시작
        else if (motor_phase == MOTOR_PHASE_AUTO_TO_UP) {
            if (check_motor_timer(MOTOR_MOVE_WAIT_MS)) {
                if (mnq_read_abs_update()) {
                    Tx_uart_SBC(0, cur_deg);
                }

                start_motor_timer();
                motor_phase = MOTOR_PHASE_AUTO_UP_HOLD;
            }
        }
        // 자동: UP 상태에서 2초 대기 끝 -> 종료
        else if (motor_phase == MOTOR_PHASE_AUTO_UP_HOLD) {
            if (check_motor_timer(AUTO_HOLD_WAIT_MS)) {
                motor_phase = MOTOR_PHASE_IDLE;      // normal state
                hit_channel = 0;
                cri_hit_count = 0;
                non_cri_hit_count = 0;
            }
        }
        // target mode: 5초 동안 센서 감지 대기
        else if (motor_phase == MOTOR_PHASE_TARGET_WAIT_SENSOR) {
            bool target_hit = false;

            /*
            * 센서 감지는 자동모드와 같은 카운트 조건 사용
            */
            if ((cur_cri_set > 0 && cri_hit_count >= cur_cri_set) ||
                (cur_non_cri_set > 0 && non_cri_hit_count >= cur_non_cri_set)) {
                target_hit = true;
            }

            bool wait_timeout = check_motor_timer(TARGET_WAIT_SENSOR_MS);

            if (target_hit || wait_timeout) {
                /*
                * 센서 감지됨 또는 5초 타임아웃
                * 둘 다 실제 기구 DOWN으로 이동 후 대기
                */
                mnq_move_to_down(CMD_MOVE_ARRIVE_SEC);

                start_motor_timer();
                motor_phase = MOTOR_PHASE_TARGET_TO_DOWN;

                cri_hit_count = 0;
                non_cri_hit_count = 0;
                hit_channel = 0;
            }
        }
        // target mode: DOWN 이동 완료 후 D9 읽고 종료
        else if (motor_phase == MOTOR_PHASE_TARGET_TO_DOWN) {
            if (check_motor_timer(MOTOR_MOVE_WAIT_MS)) {
                if (mnq_read_abs_update()) {
                    Tx_uart_SBC(0, cur_deg);
                }

                /*
                * DOWN 후 가만히 대기
                * 다시 UP으로 복귀하지 않음
                */
                motor_phase = MOTOR_PHASE_IDLE;
                hit_channel = 0;
                cri_hit_count = 0;
                non_cri_hit_count = 0;
            }
        }

        check_abs_report_timer();
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
            * P1 RISE 순간에 P2 history를 초기화해야 함.
            * main loop에서 나중에 초기화하면 이미 잡힌 P2 pulse를 지워버릴 수 있음.
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
     * 열린 pulse를 하나 만들어둔다.
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
     * first_channel_ban은 기존 채널 무시
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

            if (p2_c1 && p2_c2 && !second_channel_ban) {
                // critical
                hit_channel = 2;
                cri_hit_count += 1;
            }
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

void StartSignal(void)
{
    gpio_put(LED, 1);
    sleep_ms(1000);
    gpio_put(LED, 0);
}

// motor와 통신이 끊길 경우 정지, sbc와 통신이 끊길 경우 control_signal = false;로 변환
void uart_timeout(){
    if (absolute_time_diff_us(last_uart_time, get_absolute_time()) >= check_uart_time_out) {
        if (control_signal || prev_control_signal || motor_phase != MOTOR_PHASE_IDLE) {
            control_signal = false;
            motor_set_onoff(MOTOR_ID, false);

            prev_control_signal = false;
            prev_manual_cmd = 255;
            prev_mnq_rst_cmd = 0;
            acting_auto = false;
            cri_hit_count = 0;
            non_cri_hit_count = 0;

            motor_phase = MOTOR_PHASE_IDLE;
            motor_active = false;
            hit_channel = 0;

            signal_detector_reset();
        }
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

static void motor_clear_rx(void)
{
    while (uart_is_readable(UART_ID_MOTOR)) {
        (void)uart_getc(UART_ID_MOTOR);
    }
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
void safe_stop(uint8_t dev_id)
{
    // 감속 정지 후 OFF
    cmd_acc_speed(dev_id, false, 0.0f, 0.2f);
    motor_set_onoff(dev_id, false);
}

// --------------------- MNQ position ---------------------
static mnq_position_t mnq_classify(uint8_t dir, float deg)
{
    if (dir == ABS_UP_DIR &&
        deg >= ABS_UP_MIN_DEG &&
        deg <= ABS_UP_MAX_DEG) {
        return MNQ_POS_UP;
    }

    if (dir == ABS_DOWN_DIR &&
        deg >= ABS_DOWN_MIN_DEG &&
        deg <= ABS_DOWN_MAX_DEG) {
        return MNQ_POS_DOWN;
    }

    return MNQ_POS_UNKNOWN;
}

static bool mnq_read_abs_update(void)
{
    uint8_t dir;
    float deg;

    if (!motor_request_abs_deg(MOTOR_ID, &dir, &deg)) {
        return false;
    }

    cur_abs_dir = dir;
    cur_deg = deg;
    cur_mnq_pos = mnq_classify(cur_abs_dir, cur_deg);

    return true;
}

static void mnq_move_to_down(float arrive_s)
{
    cmd_acc_pos(MOTOR_ID, CMD_MOVE_TO_DOWN_DIR_CW, CMD_MOVE_TO_DOWN_DEG, arrive_s);
}

static void mnq_move_to_up(float arrive_s)
{
    cmd_acc_pos(MOTOR_ID, CMD_MOVE_TO_UP_DIR_CW, CMD_MOVE_TO_UP_DEG, arrive_s);
}

static void mnq_rst_move_to_up_by_abs_delta(float arrive_s)
{
    float move_deg;
    bool move_dir_cw;

    /*
     * 목표:
     * 절대엔코더 기준 정상 UP 위치(ABS_UP_CENTER_DEG = 82.5도)로 복귀
     *
     * 주의:
     * cmd_acc_pos()의 pos_deg는 절대엔코더 목표각이 아니라
     * 모터 컨트롤러 위치제어 명령값
     * 그래서 현재 절대엔코더와 목표 절대엔코더의 차이를 계산한 뒤,
     * RST 복귀 동작 동안만 상대 위치제어 모드로 바꿔서 이동
     */

    if (cur_abs_dir == ABS_UP_DIR &&
        cur_deg >= ABS_UP_MIN_DEG &&
        cur_deg <= ABS_UP_MAX_DEG) {
        return;
    }

    if (cur_deg < ABS_UP_CENTER_DEG) {
        /*
         * 현재 절대엔코더 각도가 정상 UP 기준보다 낮음
         * 예: DOWN 쪽으로 처져서 0.5도 근처
         * → UP 방향으로 차이만큼 이동
         */
        move_deg = ABS_UP_CENTER_DEG - cur_deg;
        move_dir_cw = CMD_MOVE_TO_UP_DIR_CW;
    }
    else {
        /*
         * 현재 절대엔코더 각도가 정상 UP 기준보다 높음
         * → 반대 방향으로 차이만큼 이동
         */
        move_deg = cur_deg - ABS_UP_CENTER_DEG;
        move_dir_cw = CMD_MOVE_TO_DOWN_DIR_CW;
    }

    if (move_deg < 1.0f) {
        return;
    }

    if (move_deg > 120.0f) {
        move_deg = 120.0f;
    }

    /*
     * 차이값 move_deg는 상대 이동량이므로,
     * RST 보정 이동 중에는 상대 위치제어 모드로 바꿈
     */
    sa_set_posmode(MOTOR_ID, false);   // false = 상대 위치제어 모드
    sleep_ms(300);

    rst_restore_absolute_mode = true;

    cmd_acc_pos(MOTOR_ID, move_dir_cw, move_deg, arrive_s);
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

    mask = 0;
    if (Receive_Data[2] == 1) mask |= 0x01; // ch1 ban
    if (Receive_Data[3] == 1) mask |= 0x02; // ch2 ban
    if (Receive_Data[4] == 1) mask |= 0x04; // ch3 ban

    // bool로도 세팅
    first_channel_ban  = (mask & 0x01) != 0;
    second_channel_ban = (mask & 0x02) != 0;

    // IRQ enable/disable (ban이면 인터럽트 꺼서 핀 신호 무시)
    gpio_set_irq_enabled(DETECT_1_P1, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, !first_channel_ban);
    gpio_set_irq_enabled(DETECT_2_P2, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, !second_channel_ban);
    //gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, !third_channel_ban);

    // 밴 상태가 바뀐 순간, 남아있는 hit/flag 정리
    if (mask != prev_mask) {
        hit_channel = 0;
        signal_detector_reset();
        prev_mask = mask;
    }
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

// 모터 이동 후 절대엔코더 값을 읽어 SBC로 보내기 위한 타이머
void start_abs_report_timer(void)
{
    abs_report_pending = true;
    abs_report_timer_start = to_ms_since_boot(get_absolute_time());
}

void check_abs_report_timer(void)
{
    if (!abs_report_pending) {
        return;
    }

    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    if ((current_time - abs_report_timer_start) >= MOTOR_MOVE_WAIT_MS) {
        abs_report_pending = false;

        if (mnq_read_abs_update()) {
            if (rst_zero_after_move) {
                rst_zero_after_move = false;

                /*
                * RST 복귀 후 절대엔코더가 정상 UP 범위에 들어왔을 때만
                * 현재 위치를 내부 0점으로 초기화
                * 범위 밖이면 잘못된 위치를 원점으로 저장하면 안 되므로
                * motor_reset_pos()를 하지 않음
                */
                if (cur_abs_dir == ABS_UP_DIR &&
                    cur_deg >= ABS_UP_MIN_DEG &&
                    cur_deg <= ABS_UP_MAX_DEG) {

                    motor_set_onoff(MOTOR_ID, false);
                    sleep_ms(120);

                    motor_reset_pos(MOTOR_ID);
                    sleep_ms(300);

                    /*
                    * RST 보정 때 상대모드로 바꿨으므로,
                    * 이후 기존 수동/자동 UP/DOWN 명령을 위해 다시 절대모드로 복귀
                    */
                    sa_set_posmode(MOTOR_ID, true);    // true = 절대 위치제어 모드
                    sleep_ms(300);

                    motor_set_onoff(MOTOR_ID, true);
                    sleep_ms(120);
                }
                else {
                    /*
                    * 복귀 실패 또는 범위 밖이면 원점 초기화 금지
                    * 단, 상대모드로 바꿨을 가능성이 있으므로 절대모드는 복구
                    */
                    if (rst_restore_absolute_mode) {
                        sa_set_posmode(MOTOR_ID, true);
                        sleep_ms(300);
                    }
                }

                rst_restore_absolute_mode = false;
            }

            Tx_uart_SBC(0, cur_deg);
        }
        else {
            rst_zero_after_move = false;
        }
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