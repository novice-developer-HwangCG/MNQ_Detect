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

// 감지 알고리즘 선택: 0=P1 5ms -> 40ms -> P2 5ms, 1=P2 단독 5ms
// 유효 피격 후 중복 감지 방지 200ms
// 모터 운용 모드 3: 착탄 감지만 수행하고 hold_stop으로 현재 위치 유지
// STOP 해제 후 자동 UP 금지: hold_stop 유지, RST 명령으로만 이동 재무장
// 45/90 혼용, ASDD 변경 gain 값 65 -> 0 교체 (SBC/UI도 gain 0 전송 필요)
// P1 추적 및 전달 (주석처리됨)
// NM-4 (NuriMotor-4) / - [제거 됨]

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
#define UART_DATA_LENGTH               (11)

static uint8_t Receive_Data[UART_DATA_LENGTH]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static volatile uint        Receive_Done       = 0;
static volatile uint        Receive_Data_Count = 0;
static volatile uint8_t     Receive_Char       = 0;

static bool control_signal = false;

static absolute_time_t last_uart_time;
static const uint32_t check_uart_time_out = 5000000;

// --------------------- Detect Sensor ---------------------
#define DETECT_1_P1        19
#define DETECT_2_P2        20
//#define DETECT_3        21
#define GAIN               14

#define P1_SAMPLE_INTERVAL_US  1000u      // P1 FALL 이후 1ms 간격으로 LOW 확인
#define P1_SAMPLE_COUNT        5u         // 1ms x 5회 = 약 5ms LOW 확인
#define P1_CONFIRM_US          (P1_SAMPLE_INTERVAL_US * P1_SAMPLE_COUNT)
#define P1_P2_DELAY_US         40000u     // P1 5ms LOW 확인 완료 후 P2 판정 시작까지 40ms 단순 지연
#define P2_SAMPLE_INTERVAL_US  1000u      // P2 확인 시작 후 1ms 간격으로 LOW 확인
#define P2_SAMPLE_COUNT        5u         // 1ms x 5회 = 약 5ms LOW 확인
#define P2_CONFIRM_US          (P2_SAMPLE_INTERVAL_US * P2_SAMPLE_COUNT)
#define DETECT_COOLDOWN_US     200000u    // 유효 판정 직후 200ms 중복 감지 방지

#define P1_HIST_SIZE           64
#define P2_HIST_SIZE           128
#define ACTIVE_IDX_NONE        0xFFu

typedef enum {
    DETECT_MODE_P1_P2 = 0,     // 기존: P1 5ms 확인 -> 40ms 뒤 P2 5ms 확인
    DETECT_MODE_P2_ONLY = 1    // 비상: P2만 5ms 확인
} detect_mode_t;

typedef enum {
    SIG_IDLE = 0,
    SIG_WAIT_P2_DELAY,
    SIG_CONFIRM_P2,
    SIG_CONFIRM_P2_ONLY,
    SIG_COOLDOWN
} signal_detect_state_t;

typedef struct {
    uint32_t start_us;      // active-low 시작: FALL 시각
    uint32_t end_us;        // active-low 종료: RISE 시각
    uint32_t width_us;      // end_us - start_us
    uint32_t start_seq;
    uint32_t end_seq;
    bool valid;
    bool ended;
} active_low_pulse_t;

typedef struct {
    active_low_pulse_t p1_hist[P1_HIST_SIZE];
    active_low_pulse_t p2_hist[P2_HIST_SIZE];
    bool p1_active;        // 현재 P1 LOW 상태인지
    bool p2_active;        // 현재 P2 LOW 상태인지
    uint32_t p1_start_us;
    uint32_t p2_start_us;
    uint32_t latest_p1_seq;
} signal_snapshot_t;

static volatile uint32_t sig_event_seq = 0;

static volatile bool     isr_p1_active = false;
static volatile uint32_t isr_p1_start_us = 0;
static volatile uint32_t isr_p1_start_seq = 0;
static volatile active_low_pulse_t isr_p1_hist[P1_HIST_SIZE];
static volatile uint8_t  isr_p1_write_idx = 0;

// P1 IRQ trace for SBC diagnostics
// active-low 기준: FALL 시 p1_irq_start_us 갱신,
// 1ms x 5회 LOW 확인 성공 시 p1_irq_end_us/width/seq 갱신
// P1 RISE 인터럽트는 사용하지 않으므로 p1_irq_rise_cnt는 0으로 유지된다.
static volatile uint32_t p1_irq_start_us = 0;
static volatile uint32_t p1_irq_end_us = 0;
static volatile uint32_t p1_irq_width_us = 0;
static volatile uint32_t p1_irq_end_seq = 0;
static volatile uint32_t p1_irq_fall_cnt = 0;
static volatile uint32_t p1_irq_rise_cnt = 0;

// P1 FALL 이후 1ms x 5회 샘플링용 상태
static volatile bool     p1_sample_running = false;
static volatile uint8_t  p1_sample_count = 0;
static volatile uint32_t p1_sample_start_us = 0;
static volatile uint32_t p1_sample_start_seq = 0;
static volatile bool     p1_confirm_pending = false;
static volatile uint32_t p1_confirm_start_us = 0;
static volatile uint32_t p1_confirm_end_us = 0;
static volatile uint32_t p1_confirm_width_us = 0;
static volatile uint32_t p1_confirm_seq = 0;

static volatile bool     isr_p2_active = false;
static volatile uint32_t isr_p2_start_us = 0;
static volatile uint32_t isr_p2_start_seq = 0;
static volatile active_low_pulse_t isr_p2_hist[P2_HIST_SIZE];
static volatile uint8_t  isr_p2_write_idx = 0;
static volatile uint8_t  isr_p2_active_idx = ACTIVE_IDX_NONE;

static signal_detect_state_t sig_state = SIG_IDLE;
static volatile detect_mode_t current_detect_mode = DETECT_MODE_P1_P2;

// P2 단독 모드에서 FALL 발생을 main loop로 넘기기 위한 플래그
static volatile bool p2_only_fall_pending = false;
static volatile uint32_t p2_only_fall_us = 0;

static uint32_t active_p1_end_us = 0;
static uint32_t active_p1_seq = 0;
static uint32_t last_used_p1_seq = 0;
static uint32_t sig_cooldown_start_us = 0;

// P1 확인 완료 후 40ms 단순 지연 뒤, P2를 1ms x 5회 샘플링하여 5ms LOW 확인
static uint32_t p2_sample_next_us = 0;
static uint8_t  p2_sample_count = 0;

volatile bool first_channel_ban = false;
volatile bool second_channel_ban = false;

static uint8_t default_gain_pwm = 0;    // 기존 65 -> 0
static uint8_t current_gain_pwm = 0;    // 기존 65 -> 0

volatile uint8_t cur_cri_set = 0;
volatile uint8_t cri_hit_count = 0;

volatile uint8_t cur_non_cri_set = 0;
volatile uint8_t non_cri_hit_count = 0;

static int hit_channel = 0;
static int pending_auto_hit_channel = 0; // 자동 HALF/FULL 판단용
static float cur_deg = 0.0f;
static uint8_t cur_abs_dir = 0x00;

static uint8_t prev_mask = 0;
static uint8_t prev_motor_ctrl = 0xFF;

static uint8_t cur_mnq_rst_cmd = 0;
static uint8_t cur_stop_cmd = 0;
static uint8_t current_motor_mode = 0;

// STOP이 한 번이라도 들어오면 해제 후에도 자동 재이동하지 않는다.
// RST 명령을 받아야 이동 래치가 해제된다.
static bool emergency_stop_active = false;
static bool stop_hold_latched = false;

static bool motor_on_state = false;
static bool hold_stop_done = false;

static absolute_time_t motor_timer_start_time;

// --------------------- Non-blocking motor control / telemetry ---------------------
/*
 0x02 가감속 위치제어
 - D9: 실제 기구 절대각, 소프트리밋, RST, 상대 이동량 계산의 기준
 - D1: 이동 중 속도/통신 상태 확인용으로만 사용
 - 0x02 명령 후에도 main loop를 막지 않고 D9/D1을 비동기로 수신
 - 도달시간 경과 후 D9로 목표 도착을 확인하고, 허용오차 밖이면 1회 보정
 - 정상 완료 시 0x02 위치제어 상태를 유지하여 목표 위치를 계속 유지
 */
#define MOTOR_TELEMETRY_INTERVAL_MS       (100u)  // SBC로 "hit, angle, detect_mode" 송신 주기
#define MOTOR_D9_POLL_INTERVAL_MS         (100u)  // 절대각 갱신 주기
#define MOTOR_D1_POLL_INTERVAL_MS         (150u)  // 이동 중 D1 피드백 주기
#define MOTOR_FEEDBACK_TIMEOUT_MS         (35u)   // D9/D1 응답 1회 대기 한도
#define MOTOR_ENABLE_WAIT_MS              (120u)  // 제어 ON 후 안정화 시간
#define MOTOR_MOVE_FEEDBACK_WAIT_MS       (500u)  // 시작/검증 D9 피드백 대기 한도
#define MOTOR_RX_FRAME_MAX                (32u)

typedef struct {
    bool valid;
    uint8_t dir;          // 0x00=CCW, 0x01=CW
    float pos_deg;        // 0.01 degree, 상대 운전 위치값
    float speed_rpm;      // 0.1 RPM
    float current_a;      // 0.1 A
} motor_pos_feedback_t;

typedef enum {
    MOTOR_FB_NONE = 0,
    MOTOR_FB_D9,
    MOTOR_FB_D1
} motor_feedback_req_t;

typedef enum {
    MOTOR_STATE_DISABLED = 0,
    MOTOR_STATE_ENABLING,
    MOTOR_STATE_READY,
    MOTOR_STATE_WAIT_START_D9,
    MOTOR_STATE_MOVING,
    MOTOR_STATE_WAIT_VERIFY_D9
} motor_state_t;

typedef enum {
    MOTOR_MOVE_NONE = 0,
    MOTOR_MOVE_UP,
    MOTOR_MOVE_DOWN,
    MOTOR_MOVE_HALF_DOWN,
    MOTOR_MOVE_RST
} motor_move_kind_t;

typedef struct {
    motor_move_kind_t kind;
    float target_deg;
    float tol_deg;
    float arrive_s;
    bool allow_outside_limits;     // RST=true, 일반 UP/DOWN=false
    uint8_t correction_count;
    uint32_t wait_d9_seq;
    uint32_t phase_start_ms;
    uint32_t settle_deadline_ms;
} motor_motion_t;

static motor_state_t motor_state = MOTOR_STATE_DISABLED;
static motor_motion_t motor_motion = {0};
static motor_move_kind_t motor_last_completed = MOTOR_MOVE_NONE;
static motor_move_kind_t motor_last_failed = MOTOR_MOVE_NONE;

static uint32_t motor_enable_ready_ms = 0u;

static uint8_t motor_rx_frame[MOTOR_RX_FRAME_MAX];
static size_t motor_rx_idx = 0;
static size_t motor_rx_total_len = 0;
static motor_feedback_req_t motor_fb_pending = MOTOR_FB_NONE;
static absolute_time_t motor_fb_request_time;

static bool motor_d9_valid = false;
static float motor_d9_raw_deg = 0.0f;
static uint32_t motor_d9_seq = 0;
static absolute_time_t motor_d9_update_time;
static absolute_time_t motor_last_d9_request_time;
static bool motor_last_d9_request_valid = false;

static motor_pos_feedback_t motor_d1 = {0};
static uint32_t motor_d1_seq = 0;
static absolute_time_t motor_d1_update_time;
static absolute_time_t motor_last_d1_request_time;
static bool motor_last_d1_request_valid = false;

static absolute_time_t motor_last_telemetry_time;
static bool motor_last_telemetry_time_valid = false;
static uint32_t motor_last_telemetry_d9_seq = 0;
static uint8_t motor_last_telemetry_detect_mode = 0xFFu;

// --------------------- Motor PARAM ---------------------
// [사용자 조정 1] 기구물의 UP/DOWN/HALF 목표각
#define NORMAL_UP_DEG              (132.5f)
#define NORMAL_DOWN_DEG            (50.5f)
#define HALF_DOWN_DEG              (92.0f)

#define UP_TOL_DEG                 (3.0f)
#define DOWN_TOL_DEG               (3.0f)
#define HALF_DOWN_TOL_DEG          (3.0f)

// 일반 UP/DOWN 동작 제한 범위
#define MIN_LIMIT_DEG              (45.0f)
#define MAX_LIMIT_DEG              (140.0f)

#define RST_TOL_DEG                (3.5f)
#define UP_NORMAL_MIN_DEG          (NORMAL_UP_DEG - RST_TOL_DEG)
#define UP_NORMAL_MAX_DEG          (NORMAL_UP_DEG + RST_TOL_DEG)

// 일반 UP/DOWN 이동 시간
#define MOTOR_ARRIVE_TIME_S_UP                 (1.2f)
#define MOTOR_ARRIVE_TIME_S_DOWN               (0.9f)

// RST 복구 이동 시간
#define MOTOR_RST_ARRIVE_TIME_S                (1.2f)

// 명령 도달시간 이후 안정화 및 D9 재확인/보정 설정
#define MOTOR_MOVE_EXTRA_WAIT_MS                (100u)
#define MOTOR_CORRECTION_ARRIVE_TIME_S          (0.3f)
#define MOTOR_CORRECTION_EXTRA_WAIT_MS          (100u)
#define MOTOR_CORRECTION_COUNT                  (1u)

// 장착된 기구 기준 방향 확인
#define INCREASE_CMD_CW            (false)
#define DECREASE_CMD_CW            (true)

//#define MOTOR1_NORMAL_ABS_DIR      (0x00u)

typedef enum {
    AUTO_SEQ_IDLE = 0,
    AUTO_SEQ_ENTRY_UP,     // Auto 진입 전 UP 복귀용
    AUTO_SEQ_MOVE_DOWN,
    AUTO_SEQ_WAIT_DOWN,
    AUTO_SEQ_MOVE_UP
} auto_seq_state_t;

static bool auto_mode_ready = false;
static auto_seq_state_t auto_seq_state = AUTO_SEQ_IDLE;

// --------------------- Common set ---------------------
#define DATA_BITS                       (8)
#define STOP_BITS                       (1)
#define PARITY                          (UART_PARITY_NONE)

static char buffer[128];

#define PICO_SYS_CLK_kHz                (125000)            // 125000 kHz
#define PICO_SYS_CLK                    (125000000)

// --------------------- Control (common) ---------------------
#define MODE_ACC_POS                    0x02u
#define MODE_ACC_SPEED                  0x03u

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
void Tx_uart_SBC(int hit_channel, float cur_deg, uint8_t detect_mode);

// --------------------- Motor function ---------------------
uint8_t calcChecksum(const uint8_t* frame, size_t len);
size_t  buildFrame(uint8_t dev_id, uint8_t mode, const uint8_t* data, size_t data_len, uint8_t* out, size_t out_max);
static void motor_send(uint8_t dev_id, uint8_t mode, const uint8_t* data, size_t data_len);
static void motor_clear_rx(void);
static float d9_to_signed_deg(uint8_t dir, float deg);

void motor_set_onoff(uint8_t dev_id, bool on);
void sa_set_posmode(uint8_t dev_id, bool absolute);
void motor_reset_pos(uint8_t dev_id);
static void motor_hold_stop(uint8_t dev_id);

int cmd_acc_pos(uint8_t dev_id, bool cw, float pos_deg, float arrive_s);
int cmd_acc_speed(uint8_t dev_id, bool cw, float spd_rpm, float arrive_s);

static void motor_feedback_service(void);
static void motor_motion_service(void);
static void motor_telemetry_service(void);
static void motor_request_enable(void);
static void motor_request_hold_stop(void);
static bool motor_request_move(motor_move_kind_t kind);
static bool motor_motion_is_busy(void);
static bool motor_take_completed(motor_move_kind_t kind);
static void motor_clear_motion_request(void);

void start_motor_timer(void);
bool check_motor_timer(uint32_t duration_ms);

static void motor_up(void);
static void motor_down(void);
static bool motor_rst_recover(void);
static bool motor_is_up_position(void);

// --------------------- sensor function ---------------------
static void gpio_irq_callback(uint gpio, uint32_t events);
static int64_t p1_sample_alarm_callback(alarm_id_t id, void *user_data);

static void signal_detector_update(void);
static void signal_detector_reset(void);
static bool signal_take_p2_only_fall(uint32_t *fall_us);
static void register_detected_hit(int report_channel, int auto_channel);

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

    // 0x02에 현재 D9와 목표각의 차이를 전달하므로 상대 위치제어 모드 사용
    // 설정 명령 반영 시간을 확보한 뒤 제어 OFF 초기 상태로 진입한다.
    sa_set_posmode(MOTOR_ID, false);   // false = 상대 위치제어 모드(0x01)
    sleep_ms(300);

    // [수정] 부팅 시 SBC 연결을 기다리지 않고 토크 유지
    motor_request_enable();
    sleep_ms(MOTOR_ENABLE_WAIT_MS);
    motor_motion_service();

    while (true) {
        tight_loop_contents();

        motor_feedback_service();
        motor_motion_service();
        uart_timeout();

        /*
        Receive_Data[0]  -> 0 = 제어 off, 1 = on
        Receive_Data[1]  -> 0 = 자동, 1 = 수동 UP, 2 = 수동 DOWN,
                            3 = 착탄 감지 전용(모터 현재 위치 유지)
        Receive_Data[2]  -> 0 = 1 채널 on, 1 = 1 채널 off
        Receive_Data[3]  -> 0 = 2 채널 on, 1 = 2 채널 off
        Receive_Data[4]  -> 0 = 3 채널 on, 1 = 3 채널 off
        Receive_Data[5]  -> 0 ~ 254 = Gain PWM
        Receive_Data[6]  -> 0 ~ 9 치명상 기준
        Receive_Data[7]  -> 0 ~ 9 비치명상 기준
        Receive_Data[8]  -> 0 = None, 1 = RST/UP 복귀 및 STOP 래치 해제
        Receive_Data[9]  -> 0 = None, 1 = Emergency Stop
        Receive_Data[10] -> 0 = P1+P2 조합 감지, 1 = P2 단독 5ms 감지
        */

        if (Receive_Done > 0) {
            Receive_Done = 0;
            last_uart_time = get_absolute_time();

            current_motor_mode = Receive_Data[1];

            uint8_t requested_detect_mode = Receive_Data[10];
            if (requested_detect_mode > DETECT_MODE_P2_ONLY) {
                requested_detect_mode = DETECT_MODE_P1_P2;
            }

            if ((uint8_t)current_detect_mode != requested_detect_mode) {
                current_detect_mode = (detect_mode_t)requested_detect_mode;
                signal_detector_reset();
                pending_auto_hit_channel = 0;
            }

            cur_stop_cmd = Receive_Data[9];

            if (cur_stop_cmd == 1u) {
                if (!emergency_stop_active) {
                    emergency_stop_active = true;
                    stop_hold_latched = true;

                    auto_mode_ready = false;
                    control_signal = false;
                    auto_seq_state = AUTO_SEQ_IDLE;
                    pending_auto_hit_channel = 0;
                    cri_hit_count = 0;
                    non_cri_hit_count = 0;
                    gpio_put(LED, 0);
                    signal_detector_reset();

                    prev_motor_ctrl = 0xFF;
                    motor_last_completed = MOTOR_MOVE_NONE;

                    hold_stop_done = false;
                    motor_request_hold_stop();
                }
                else {
                    control_signal = false;
                }
            }
            else {
                emergency_stop_active = false;
                bool rst_handled = false;

                if (Receive_Data[0] == 0u) {
                    auto_mode_ready = false;
                    control_signal = false;
                    prev_motor_ctrl = 0xFF;
                    auto_seq_state = AUTO_SEQ_IDLE;
                    pending_auto_hit_channel = 0;
                    signal_detector_reset();

                    motor_request_hold_stop();
                }
                else {
                    if (!motor_on_state && motor_state == MOTOR_STATE_DISABLED) {
                        motor_request_enable();
                    }

                    detect_channel_ban();

                    if (Receive_Data[5] != current_gain_pwm) {
                        gain_pwm_set(Receive_Data[5]);
                    }

                    if (Receive_Data[6] != cur_cri_set) {
                        cur_cri_set = Receive_Data[6];
                        cri_hit_count = 0;
                        pending_auto_hit_channel = 0;
                    }

                    if (Receive_Data[7] != cur_non_cri_set) {
                        cur_non_cri_set = Receive_Data[7];
                        non_cri_hit_count = 0;
                        pending_auto_hit_channel = 0;
                    }

                    if (Receive_Data[8] != cur_mnq_rst_cmd) {
                        cur_mnq_rst_cmd = Receive_Data[8];

                        if (cur_mnq_rst_cmd == 1u) {
                            stop_hold_latched = false;
                            auto_mode_ready = false;
                            control_signal = false;
                            auto_seq_state = AUTO_SEQ_IDLE;
                            pending_auto_hit_channel = 0;
                            gpio_put(LED, 0);
                            signal_detector_reset();

                            hold_stop_done = false;
                            (void)motor_rst_recover();

                            prev_motor_ctrl = current_motor_mode;
                            rst_handled = true;
                        }
                    }

                    if (!rst_handled) {
                        if (stop_hold_latched) {
                            auto_mode_ready = false;
                            auto_seq_state = AUTO_SEQ_IDLE;
                            pending_auto_hit_channel = 0;
                            gpio_put(LED, 0);

                            if (current_motor_mode == 3u) {
                                control_signal = true;
                                prev_motor_ctrl = 3u;
                            }
                            else {
                                control_signal = false;
                                signal_detector_reset();
                            }
                        }
                        else if (current_motor_mode == 3u) {
                            // 착탄 감지 전용: 모터 이동을 즉시 취소하고
                            // hold_stop으로 현재 위치의 정지 토크를 유지한다.
                            // 센서 판정과 SBC hit 전달만 수행한다.
                            if (prev_motor_ctrl != 3u || motor_motion_is_busy()) {
                                auto_mode_ready = false;
                                auto_seq_state = AUTO_SEQ_IDLE;
                                pending_auto_hit_channel = 0;
                                cri_hit_count = 0;
                                non_cri_hit_count = 0;
                                gpio_put(LED, 0);
                                signal_detector_reset();

                                hold_stop_done = false;
                                motor_request_hold_stop();
                            }

                            control_signal = true;
                            prev_motor_ctrl = 3u;
                        }
                        // RST/UP/DOWN 이동 중에는 새 이동 명령을 겹쳐 보내지 않는다.
                        else if (motor_motion_is_busy()) {
                            control_signal = false;
                        }
                        else if (current_motor_mode == 0u) {
                            // 자동 모드
                            hold_stop_done = false;
                            bool skip_auto_switch = false;

                            if (!auto_mode_ready && auto_seq_state == AUTO_SEQ_IDLE) {
                                pending_auto_hit_channel = 0;
                                signal_detector_reset();
                                control_signal = false;
                                gpio_put(LED, 0);

                                if (motor_is_up_position()) {
                                    auto_mode_ready = true;
                                    control_signal = true;
                                }
                                else {
                                    skip_auto_switch = true;

                                    if (motor_request_move(MOTOR_MOVE_UP)) {
                                        auto_mode_ready = true; // 중복 UP 요청 방지
                                        auto_seq_state = AUTO_SEQ_ENTRY_UP;
                                        skip_auto_switch = false;
                                    }
                                }

                                prev_motor_ctrl = 0u;
                            }

                            if (!skip_auto_switch) {
                                switch (auto_seq_state) {
                                case AUTO_SEQ_ENTRY_UP:
                                    control_signal = false;

                                    if (motor_take_completed(MOTOR_MOVE_UP)) {
                                        pending_auto_hit_channel = 0;
                                        cri_hit_count = 0;
                                        non_cri_hit_count = 0;

                                        gpio_put(LED, 0);
                                        signal_detector_reset();

                                        auto_seq_state = AUTO_SEQ_IDLE;
                                        control_signal = true;
                                    }
                                    break;

                                case AUTO_SEQ_IDLE:
                                    control_signal = true;

                                    if (pending_auto_hit_channel == 2 && cur_cri_set > 0) {
                                        control_signal = false;
                                        motor_last_completed = MOTOR_MOVE_NONE;

                                        if (cri_hit_count >= cur_cri_set) {
                                            if (motor_request_move(MOTOR_MOVE_DOWN)) {
                                                pending_auto_hit_channel = 0;
                                                gpio_put(LED, 1);
                                                auto_seq_state = AUTO_SEQ_MOVE_DOWN;
                                            }
                                        }
                                        else {
                                            if (motor_request_move(MOTOR_MOVE_HALF_DOWN)) {
                                                pending_auto_hit_channel = 0;
                                                gpio_put(LED, 1);
                                                auto_seq_state = AUTO_SEQ_MOVE_DOWN;
                                            }
                                        }
                                    }
                                    else if (pending_auto_hit_channel == 1 && cur_non_cri_set > 0) {
                                        control_signal = false;
                                        motor_last_completed = MOTOR_MOVE_NONE;

                                        if (non_cri_hit_count >= cur_non_cri_set) {
                                            if (motor_request_move(MOTOR_MOVE_DOWN)) {
                                                pending_auto_hit_channel = 0;
                                                gpio_put(LED, 1);
                                                auto_seq_state = AUTO_SEQ_MOVE_DOWN;
                                            }
                                        }
                                        else {
                                            if (motor_request_move(MOTOR_MOVE_HALF_DOWN)) {
                                                pending_auto_hit_channel = 0;
                                                gpio_put(LED, 1);
                                                auto_seq_state = AUTO_SEQ_MOVE_DOWN;
                                            }
                                        }
                                    }
                                    break;

                                case AUTO_SEQ_MOVE_DOWN:
                                    control_signal = false;

                                    if (motor_take_completed(MOTOR_MOVE_DOWN) ||
                                        motor_take_completed(MOTOR_MOVE_HALF_DOWN)) {
                                        start_motor_timer();
                                        auto_seq_state = AUTO_SEQ_WAIT_DOWN;
                                    }
                                    break;

                                case AUTO_SEQ_WAIT_DOWN:
                                    control_signal = false;
                                    if (check_motor_timer(500u)) {
                                        if (motor_request_move(MOTOR_MOVE_UP)) {
                                            auto_seq_state = AUTO_SEQ_MOVE_UP;
                                        }
                                    }
                                    break;

                                case AUTO_SEQ_MOVE_UP:
                                    control_signal = false;

                                    if (motor_take_completed(MOTOR_MOVE_UP)) {
                                        if ((cur_cri_set > 0 && cri_hit_count >= cur_cri_set) ||
                                            (cur_non_cri_set > 0 &&
                                             non_cri_hit_count >= cur_non_cri_set)) {
                                            cri_hit_count = 0;
                                            non_cri_hit_count = 0;
                                        }

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
                            }

                            prev_motor_ctrl = 0u;
                        }
                        else if (current_motor_mode == 1u || current_motor_mode == 2u) {
                            hold_stop_done = false;
                            auto_mode_ready = false;
                            signal_detector_reset();
                            auto_seq_state = AUTO_SEQ_IDLE;
                            gpio_put(LED, 0);
                            control_signal = false;

                            if (current_motor_mode == 1u) {
                                if (prev_motor_ctrl != 1u &&
                                    motor_request_move(MOTOR_MOVE_UP)) {
                                    prev_motor_ctrl = 1u;
                                }
                            }
                            else {
                                if (prev_motor_ctrl != 2u &&
                                    motor_request_move(MOTOR_MOVE_DOWN)) {
                                    prev_motor_ctrl = 2u;
                                }
                            }
                        }
                        else {
                            control_signal = false;
                            prev_motor_ctrl = 0xFF;
                        }
                    }
                }
            }
        }

        // 명령/STOP/모드 전환을 먼저 반영한 뒤 센서 상태머신을 실행한다.
        signal_detector_update();
        motor_telemetry_service();

        sleep_us(100);
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

void Tx_uart_SBC(int hit_channel, float cur_deg, uint8_t detect_mode)
{
    // CSV: hit_channel, absolute_angle_deg, active_detection_mode
    snprintf(buffer, sizeof(buffer), "%d, %.2f, %u\n",
             hit_channel, cur_deg, (unsigned int)detect_mode);
    uart_puts(UART_ID_SBC, buffer);

    // 추 후 데이터 확인 필요 시 주석 해제
    // uint32_t p1_start;
    // uint32_t p1_end;
    // uint32_t p1_width;
    // uint32_t p1_seq;
    // uint32_t p1_fall;
    // uint32_t p1_rise;

    // uint32_t irq = save_and_disable_interrupts();
    // p1_start = p1_irq_start_us;
    // p1_end   = p1_irq_end_us;
    // p1_width = p1_irq_width_us;
    // p1_seq   = p1_irq_end_seq;
    // p1_fall  = p1_irq_fall_cnt;
    // p1_rise  = p1_irq_rise_cnt;
    // restore_interrupts(irq);

    // // CSV: shot, angle, p1_start_us, p1_end_us, p1_width_us, p1_seq, p1_fall_cnt, p1_rise_cnt
    // snprintf(buffer, sizeof(buffer), "%d, %.2f, %lu, %lu, %lu, %lu, %lu, %lu\n",
    //          hit_channel, cur_deg,
    //          (unsigned long)p1_start,
    //          (unsigned long)p1_end,
    //          (unsigned long)p1_width,
    //          (unsigned long)p1_seq,
    //          (unsigned long)p1_fall,
    //          (unsigned long)p1_rise);
    // uart_puts(UART_ID_SBC, buffer);
}

// SBC 통신이 끊기면 hold_stop을 호출하고 제어 ON 상태를 유지
void uart_timeout(void)
{
    if (absolute_time_diff_us(last_uart_time, get_absolute_time()) >= check_uart_time_out) {
        control_signal = false;
        auto_mode_ready = false;
        auto_seq_state = AUTO_SEQ_IDLE;
        pending_auto_hit_channel = 0;
        cri_hit_count = 0;
        non_cri_hit_count = 0;
        stop_hold_latched = true;
        signal_detector_reset();
        motor_request_hold_stop();
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

// --------------------- Motor function ---------------------
// UART0는 인터럽트를 쓰지 않고 main loop에서 프레임을 조금씩 파싱
// 따라서 D9/D1 응답 대기 동안에도 1.5초씩 멈추지 않으며, 센서/통신/텔레메트리가 지속
static inline uint32_t motor_now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static inline bool motor_time_reached(uint32_t deadline_ms)
{
    return (int32_t)(motor_now_ms() - deadline_ms) >= 0;
}

static inline uint16_t clamp_u16_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return (uint16_t)lo;
    if (v > hi) return (uint16_t)hi;
    return (uint16_t)v;
}

static inline uint8_t clamp_u8_i32(int32_t v, int32_t lo, int32_t hi)
{
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

size_t buildFrame(uint8_t dev_id, uint8_t mode, const uint8_t* data,
                  size_t data_len, uint8_t* out, size_t out_max)
{
    size_t total = 6u + data_len;
    if (out_max < total) return 0;

    out[0] = 0xFF;
    out[1] = 0xFE;
    out[2] = dev_id;
    out[3] = (uint8_t)(1u + 1u + data_len);
    out[4] = 0x00;
    out[5] = mode;

    if (data_len > 0u && data != NULL) {
        memcpy(&out[6], data, data_len);
    }

    out[4] = calcChecksum(out, total);
    return total;
}

static void motor_send(uint8_t dev_id, uint8_t mode, const uint8_t* data,
                       size_t data_len)
{
    uint8_t frame[32];
    size_t n = buildFrame(dev_id, mode, data, data_len, frame, sizeof(frame));
    if (n == 0u) return;

    uart_write_blocking(UART_ID_MOTOR, frame, n);
}

static void motor_clear_rx(void)
{
    while (uart_is_readable(UART_ID_MOTOR)) {
        (void)uart_getc(UART_ID_MOTOR);
    }
}

// -----추후 모터 데이터 얻어서 방향 값 제대로 확인 후 추가 수정할 것 현재 모터가 양수 영역에 있어서 부호 구분 없이 사용 중-----
// static float d9_to_signed_deg(uint8_t dir, float deg)
// {
//     return (dir == MOTOR1_POSITIVE_ABS_DIR) ? deg : -deg;
// }

static float d9_to_signed_deg(uint8_t dir, float deg)
{
    (void)dir;
    return deg;
}

// 0x02 가감속 위치제어: [dir(1), pos(2), arrive(1)]
// 상대 위치제어 모드에서 현재 위치로부터 pos_deg만큼 이동한다.
int cmd_acc_pos(uint8_t dev_id, bool cw, float pos_deg, float arrive_s)
{
    int32_t pos_raw_i = (int32_t)lroundf(pos_deg * 100.0f);
    int32_t arr_raw_i = (int32_t)lroundf(arrive_s * 10.0f);

    uint16_t pos_raw = clamp_u16_i32(pos_raw_i, 0, 65533);
    uint8_t arr_raw = clamp_u8_i32(arr_raw_i, 1, 254);

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
    uint8_t arr_raw = clamp_u8_i32(arr_raw_i, 1, 255);

    uint8_t d[4];
    d[0] = (uint8_t)(cw ? 0x01 : 0x00);
    d[1] = (uint8_t)((spd_raw >> 8) & 0xFF);
    d[2] = (uint8_t)(spd_raw & 0xFF);
    d[3] = arr_raw;

    motor_send(dev_id, MODE_ACC_SPEED, d, sizeof(d));
    return 0;
}

// ON/OFF: 0x00=On, 0x01=Off
void motor_set_onoff(uint8_t dev_id, bool on)
{
    uint8_t d[1] = { (uint8_t)(on ? 0x00 : 0x01) };
    motor_send(dev_id, SA_MODE_SET_ONOFF, d, 1);
}

// 위치 초기화는 정상 운전 경로에서 사용하지 않음
void motor_reset_pos(uint8_t dev_id)
{
    uint8_t d[1] = { 0x00 };
    motor_send(dev_id, SA_MODE_RESET_POS, d, 1);
}

void sa_set_posmode(uint8_t dev_id, bool absolute)
{
    uint8_t d[1] = { (uint8_t)(absolute ? 0x00 : 0x01) };
    motor_send(dev_id, SA_MODE_SET_POSMODE, d, 1);
}

// 0x03 가감속 속도제어, 0RPM, 도달시간 0.2초
// 비상정지/모드 전환/통신 타임아웃에서만 사용한다.
// 정상 위치 도착 후에는 호출하지 않아 0x02 목표 위치 유지 상태를 보존한다.
static void motor_hold_stop(uint8_t dev_id)
{
    (void)cmd_acc_speed(dev_id, false, 0.0f, 0.2f);
    motor_d1.valid = false;
}

/* --------------------- UART0 asynchronous feedback parser --------------------- */
static void motor_rx_reset(void)
{
    motor_rx_idx = 0u;
    motor_rx_total_len = 0u;
}

static void motor_handle_feedback_frame(const uint8_t *frame, size_t len)
{
    if (len < 6u || frame[2] != MOTOR_ID) {
        return;
    }

    if (frame[5] == SA_FEED_ABS_ENC) {
        /* FF FE ID 05 CHK D9 DIR ENC_H ENC_L */
        if (len == 9u) {
            uint16_t raw = ((uint16_t)frame[7] << 8) | frame[8];
            cur_abs_dir = frame[6];
            motor_d9_raw_deg = (float)raw / 100.0f;
            cur_deg = d9_to_signed_deg(cur_abs_dir, motor_d9_raw_deg);
            motor_d9_valid = true;
            motor_d9_update_time = get_absolute_time();
            motor_d9_seq++;

            if (motor_fb_pending == MOTOR_FB_D9) {
                motor_fb_pending = MOTOR_FB_NONE;
            }
        }
        return;
    }

    if (frame[5] == SA_FEED_POS) {
        /* FF FE ID 08 CHK D1 DIR POS_H POS_L SPD_H SPD_L CURRENT */
        if (len == 12u) {
            uint16_t pos_raw = ((uint16_t)frame[7] << 8) | frame[8];
            uint16_t spd_raw = ((uint16_t)frame[9] << 8) | frame[10];

            motor_d1.dir = frame[6];
            motor_d1.pos_deg = (float)pos_raw / 100.0f;
            motor_d1.speed_rpm = (float)spd_raw / 10.0f;
            motor_d1.current_a = (float)frame[11] / 10.0f;
            motor_d1.valid = true;
            motor_d1_update_time = get_absolute_time();
            motor_d1_seq++;

            if (motor_fb_pending == MOTOR_FB_D1) {
                motor_fb_pending = MOTOR_FB_NONE;
            }
        }
    }
}

static void motor_rx_push_byte(uint8_t b)
{
    if (motor_rx_idx == 0u) {
        if (b == 0xFFu) {
            motor_rx_frame[motor_rx_idx++] = b;
        }
        return;
    }

    if (motor_rx_idx == 1u) {
        if (b == 0xFEu) {
            motor_rx_frame[motor_rx_idx++] = b;
        }
        else if (b == 0xFFu) {
            motor_rx_frame[0] = 0xFFu;
            motor_rx_idx = 1u;
        }
        else {
            motor_rx_reset();
        }
        return;
    }

    if (motor_rx_idx >= MOTOR_RX_FRAME_MAX) {
        motor_rx_reset();
        return;
    }

    motor_rx_frame[motor_rx_idx++] = b;

    if (motor_rx_idx == 4u) {
        motor_rx_total_len = (size_t)motor_rx_frame[3] + 4u;
        if (motor_rx_total_len < 6u || motor_rx_total_len > MOTOR_RX_FRAME_MAX) {
            motor_rx_reset();
            return;
        }
    }

    if (motor_rx_total_len > 0u && motor_rx_idx >= motor_rx_total_len) {
        if (calcChecksum(motor_rx_frame, motor_rx_total_len) == motor_rx_frame[4]) {
            motor_handle_feedback_frame(motor_rx_frame, motor_rx_total_len);
        }
        motor_rx_reset();
    }
}

static bool motor_feedback_begin_request(motor_feedback_req_t req)
{
    if (motor_fb_pending != MOTOR_FB_NONE) {
        return false;
    }

    uint8_t mode = (req == MOTOR_FB_D9) ? SA_REQ_ABS_ENC : SA_REQ_POS;
    uint8_t frame[8];
    size_t n = buildFrame(MOTOR_ID, mode, NULL, 0u, frame, sizeof(frame));
    if (n == 0u) {
        return false;
    }

    motor_clear_rx();
    motor_rx_reset();
    uart_write_blocking(UART_ID_MOTOR, frame, n);

    motor_fb_pending = req;
    motor_fb_request_time = get_absolute_time();

    if (req == MOTOR_FB_D9) {
        motor_last_d9_request_time = motor_fb_request_time;
        motor_last_d9_request_valid = true;
    }
    else {
        motor_last_d1_request_time = motor_fb_request_time;
        motor_last_d1_request_valid = true;
    }

    return true;
}

static bool motor_d9_request_due(absolute_time_t now)
{
    if ((motor_state == MOTOR_STATE_WAIT_START_D9 ||
         motor_state == MOTOR_STATE_WAIT_VERIFY_D9) &&
        motor_d9_seq == motor_motion.wait_d9_seq) {
        return true;
    }

    if (!motor_last_d9_request_valid) {
        return true;
    }

    return absolute_time_diff_us(motor_last_d9_request_time, now) >=
           ((int64_t)MOTOR_D9_POLL_INTERVAL_MS * 1000ll);
}

static bool motor_d1_request_due(absolute_time_t now)
{
    if (motor_state != MOTOR_STATE_MOVING) {
        return false;
    }

    if (!motor_last_d1_request_valid) {
        return true;
    }

    return absolute_time_diff_us(motor_last_d1_request_time, now) >=
           ((int64_t)MOTOR_D1_POLL_INTERVAL_MS * 1000ll);
}

static void motor_feedback_service(void)
{
    while (uart_is_readable(UART_ID_MOTOR)) {
        motor_rx_push_byte(uart_getc(UART_ID_MOTOR));
    }

    absolute_time_t now = get_absolute_time();

    if (motor_fb_pending != MOTOR_FB_NONE &&
        absolute_time_diff_us(motor_fb_request_time, now) >=
            ((int64_t)MOTOR_FEEDBACK_TIMEOUT_MS * 1000ll)) {
        // 응답 1회 누락은 즉시 안전정지하지 않고, 다음 poll에서 재요청
        motor_fb_pending = MOTOR_FB_NONE;
        motor_rx_reset();
    }

    if (motor_fb_pending == MOTOR_FB_NONE) {
        if (motor_d9_request_due(now)) {
            (void)motor_feedback_begin_request(MOTOR_FB_D9);
        }
        else if (motor_d1_request_due(now)) {
            (void)motor_feedback_begin_request(MOTOR_FB_D1);
        }
    }
}

/* --------------------- motion state machine --------------------- */

static bool motor_motion_is_busy(void)
{
    return motor_state == MOTOR_STATE_WAIT_START_D9 ||
           motor_state == MOTOR_STATE_MOVING ||
           motor_state == MOTOR_STATE_WAIT_VERIFY_D9;
}

static void motor_clear_motion_request(void)
{
    motor_motion.kind = MOTOR_MOVE_NONE;
    motor_motion.target_deg = 0.0f;
    motor_motion.tol_deg = 0.0f;
    motor_motion.arrive_s = 0.0f;
    motor_motion.allow_outside_limits = false;
    motor_motion.correction_count = 0u;
    motor_motion.wait_d9_seq = motor_d9_seq;
    motor_motion.phase_start_ms = 0u;
    motor_motion.settle_deadline_ms = 0u;
}

static void motor_begin_wait_start_d9(void)
{
    motor_motion.wait_d9_seq = motor_d9_seq;
    motor_motion.phase_start_ms = motor_now_ms();
    motor_state = MOTOR_STATE_WAIT_START_D9;
}

static void motor_mark_failed(void)
{
    motor_last_failed = motor_motion.kind;
    hold_stop_done = false;
    motor_request_hold_stop();
}

static void motor_mark_completed(void)
{
    motor_last_completed = motor_motion.kind;
    // 정상 완료 후에는 0x03을 보내지 않는다.
    // 마지막 0x02 위치 명령이 목표 위치를 계속 유지하도록 둔다.
    motor_clear_motion_request();
    motor_state = MOTOR_STATE_READY;
}

static bool motor_take_completed(motor_move_kind_t kind)
{
    if (motor_last_completed != kind) {
        return false;
    }

    motor_last_completed = MOTOR_MOVE_NONE;
    return true;
}

static bool motor_calc_d9_move(float current_deg, float target_deg,
                               float *move_deg, bool *move_cw,
                               float *predicted_deg)
{
    if (move_deg == NULL || move_cw == NULL || predicted_deg == NULL) {
        return false;
    }

    float diff = target_deg - current_deg;

    if (diff > 0.0f) {
        *move_cw = INCREASE_CMD_CW;
        *move_deg = diff;
    }
    else {
        *move_cw = DECREASE_CMD_CW;
        *move_deg = -diff;
    }

    *predicted_deg = current_deg + diff;
    return true;
}

static void motor_issue_position_from_latest_d9(void)
{
    if (!motor_d9_valid) {
        motor_mark_failed();
        return;
    }

    float current = cur_deg;
    float target = motor_motion.target_deg;

    if (!motor_motion.allow_outside_limits) {
        if (current < MIN_LIMIT_DEG || current >= MAX_LIMIT_DEG ||
            target < MIN_LIMIT_DEG || target >= MAX_LIMIT_DEG) {
            motor_mark_failed();
            return;
        }

        // UP 정상범위보다 더 아래로 떨어졌다면 DOWN 금지, RST만 허용
        if ((motor_motion.kind == MOTOR_MOVE_DOWN ||
            motor_motion.kind == MOTOR_MOVE_HALF_DOWN) &&
            current <= UP_NORMAL_MIN_DEG) {
            motor_mark_failed();
            return;
        }
    }

    float err = fabsf(target - current);
    if (err <= motor_motion.tol_deg) {
        motor_mark_completed();
        return;
    }

    float move_deg = 0.0f;
    float predicted_deg = 0.0f;
    bool move_cw = false;

    if (!motor_calc_d9_move(current, target, &move_deg, &move_cw,
                            &predicted_deg) ||
        move_deg < 0.01f) {
        motor_mark_completed();
        return;
    }

    if (!motor_motion.allow_outside_limits &&
        (predicted_deg < MIN_LIMIT_DEG || predicted_deg >= MAX_LIMIT_DEG)) {
        motor_mark_failed();
        return;
    }

    bool is_correction = (motor_motion.correction_count > 0u);
    float command_arrive_s = is_correction ?
        MOTOR_CORRECTION_ARRIVE_TIME_S : motor_motion.arrive_s;
    uint32_t extra_wait_ms = is_correction ?
        MOTOR_CORRECTION_EXTRA_WAIT_MS : MOTOR_MOVE_EXTRA_WAIT_MS;

    (void)cmd_acc_pos(MOTOR_ID, move_cw, move_deg, command_arrive_s);

    motor_motion.settle_deadline_ms = motor_now_ms() +
        (uint32_t)(command_arrive_s * 1000.0f) + extra_wait_ms;
    motor_state = MOTOR_STATE_MOVING;
}

static bool motor_request_move(motor_move_kind_t kind)
{
    if (kind == MOTOR_MOVE_NONE || motor_state == MOTOR_STATE_DISABLED) {
        return false;
    }

    if (motor_motion_is_busy()) {
        return false;
    }

    motor_clear_motion_request();
    motor_motion.kind = kind;

    switch (kind) {
    case MOTOR_MOVE_UP:
        motor_motion.target_deg = NORMAL_UP_DEG;
        motor_motion.tol_deg = UP_TOL_DEG;
        motor_motion.arrive_s = MOTOR_ARRIVE_TIME_S_UP;
        motor_motion.allow_outside_limits = false;
        break;

    case MOTOR_MOVE_DOWN:
        motor_motion.target_deg = NORMAL_DOWN_DEG;
        motor_motion.tol_deg = DOWN_TOL_DEG;
        motor_motion.arrive_s = MOTOR_ARRIVE_TIME_S_DOWN;
        motor_motion.allow_outside_limits = false;
        break;

    case MOTOR_MOVE_HALF_DOWN:
        motor_motion.target_deg = HALF_DOWN_DEG;
        motor_motion.tol_deg = HALF_DOWN_TOL_DEG;
        motor_motion.arrive_s = MOTOR_ARRIVE_TIME_S_DOWN;
        motor_motion.allow_outside_limits = false;
        break;

    case MOTOR_MOVE_RST:
        motor_motion.target_deg = NORMAL_UP_DEG;
        motor_motion.tol_deg = RST_TOL_DEG;
        motor_motion.arrive_s = MOTOR_RST_ARRIVE_TIME_S;
        motor_motion.allow_outside_limits = true;
        break;

    default:
        motor_clear_motion_request();
        return false;
    }

    if (motor_state == MOTOR_STATE_ENABLING) {
        // ON 안정화 완료 후 자동으로 D9 확인을 시작
        return true;
    }

    if (motor_state != MOTOR_STATE_READY) {
        motor_clear_motion_request();
        return false;
    }

    motor_begin_wait_start_d9();
    return true;
}

static void motor_request_enable(void)
{
    if (motor_state != MOTOR_STATE_DISABLED) {
        return;
    }

    motor_set_onoff(MOTOR_ID, true);
    motor_enable_ready_ms = motor_now_ms() + MOTOR_ENABLE_WAIT_MS;
    motor_state = MOTOR_STATE_ENABLING;
}

static void motor_request_hold_stop(void)
{
    control_signal = false;
    auto_seq_state = AUTO_SEQ_IDLE;
    signal_detector_reset();

    if (hold_stop_done) {
        return;
    }

    if (motor_on_state && motor_state != MOTOR_STATE_DISABLED) {
        // 비상정지/모드 전환/통신 타임아웃에서는 현재 이동을 취소하고
        // 0x03 0RPM hold_stop으로 제어 ON 상태의 정지 토크를 유지한다.
        motor_hold_stop(MOTOR_ID);
        motor_clear_motion_request();
        motor_state = MOTOR_STATE_READY;
    }
    else {
        motor_clear_motion_request();
    }

    hold_stop_done = true;
}

static void motor_motion_service(void)
{
    uint32_t now_ms = motor_now_ms();

    switch (motor_state) {
    case MOTOR_STATE_ENABLING:
        if (motor_time_reached(motor_enable_ready_ms)) {
            motor_on_state = true;
            motor_state = MOTOR_STATE_READY;

            if (motor_motion.kind != MOTOR_MOVE_NONE) {
                motor_begin_wait_start_d9();
            }
            else {
                // 제어 ON 직후 기존 hold_stop으로 정지 토크를 확보한다.
                motor_hold_stop(MOTOR_ID);
            }
        }
        break;

    case MOTOR_STATE_WAIT_START_D9:
        if (motor_d9_seq != motor_motion.wait_d9_seq) {
            motor_issue_position_from_latest_d9();
        }
        else if ((uint32_t)(now_ms - motor_motion.phase_start_ms) >=
                 MOTOR_MOVE_FEEDBACK_WAIT_MS) {
            motor_mark_failed();
        }
        break;

    case MOTOR_STATE_MOVING:
        if (motor_time_reached(motor_motion.settle_deadline_ms)) {
            motor_motion.wait_d9_seq = motor_d9_seq;
            motor_motion.phase_start_ms = now_ms;
            motor_state = MOTOR_STATE_WAIT_VERIFY_D9;
        }
        break;

    case MOTOR_STATE_WAIT_VERIFY_D9:
        if (motor_d9_seq != motor_motion.wait_d9_seq) {
            float err = fabsf(cur_deg - motor_motion.target_deg);

            if (err <= motor_motion.tol_deg) {
                motor_mark_completed();
            }
            else if (motor_motion.correction_count < MOTOR_CORRECTION_COUNT) {
                motor_motion.correction_count++;
                motor_issue_position_from_latest_d9();
            }
            else {
                motor_mark_failed();
            }
        }
        else if ((uint32_t)(now_ms - motor_motion.phase_start_ms) >=
                 MOTOR_MOVE_FEEDBACK_WAIT_MS) {
            motor_mark_failed();
        }
        break;

    default:
        break;
    }
}

/* --------------------- public UP/DOWN/RST entry --------------------- */

static void motor_down(void)
{
    (void)motor_request_move(MOTOR_MOVE_DOWN);
}

static void motor_up(void)
{
    (void)motor_request_move(MOTOR_MOVE_UP);
}

static bool motor_rst_recover(void)
{
    return motor_request_move(MOTOR_MOVE_RST);
}

static bool motor_is_up_position(void)
{
    return motor_d9_valid &&
           cur_deg >= UP_NORMAL_MIN_DEG &&
           cur_deg <= UP_NORMAL_MAX_DEG;
}

/* --------------------- continuous telemetry --------------------- */
static void motor_telemetry_service(void)
{
    absolute_time_t now = get_absolute_time();
    bool d9_changed = (motor_d9_seq != motor_last_telemetry_d9_seq);
    bool mode_changed = ((uint8_t)current_detect_mode !=
                         motor_last_telemetry_detect_mode);
    bool interval_due = !motor_last_telemetry_time_valid ||
        absolute_time_diff_us(motor_last_telemetry_time, now) >=
            ((int64_t)MOTOR_TELEMETRY_INTERVAL_MS * 1000ll);

    if (hit_channel != 0 || d9_changed || mode_changed || interval_due) {
        Tx_uart_SBC(hit_channel, cur_deg, (uint8_t)current_detect_mode);

        hit_channel = 0;
        motor_last_telemetry_d9_seq = motor_d9_seq;
        motor_last_telemetry_detect_mode = (uint8_t)current_detect_mode;
        motor_last_telemetry_time = now;
        motor_last_telemetry_time_valid = true;
    }
}

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
    gpio_pull_up(DETECT_1_P1);     // active-low: 평상시 HIGH, 감지 시 LOW

    gpio_init(DETECT_2_P2);
    gpio_set_dir(DETECT_2_P2, GPIO_IN);
    gpio_pull_up(DETECT_2_P2);     // active-low: 평상시 HIGH, 감지 시 LOW

    uint32_t now = time_us_32();
    uint32_t irq = save_and_disable_interrupts();

    sig_event_seq = 0;
    isr_p1_write_idx = 0;
    isr_p2_write_idx = 0;
    isr_p2_active_idx = ACTIVE_IDX_NONE;

    p1_irq_start_us = 0;
    p1_irq_end_us = 0;
    p1_irq_width_us = 0;
    p1_irq_end_seq = 0;
    p1_irq_fall_cnt = 0;
    p1_irq_rise_cnt = 0;
    p1_sample_running = false;
    p1_sample_count = 0;
    p1_sample_start_us = 0;
    p1_sample_start_seq = 0;
    p1_confirm_pending = false;
    p1_confirm_start_us = 0;
    p1_confirm_end_us = 0;
    p1_confirm_width_us = 0;
    p1_confirm_seq = 0;
    p2_sample_next_us = 0;
    p2_sample_count = 0;
    p2_only_fall_pending = false;
    p2_only_fall_us = 0;

    for (int i = 0; i < P1_HIST_SIZE; i++) {
        isr_p1_hist[i].start_us = 0;
        isr_p1_hist[i].end_us = 0;
        isr_p1_hist[i].width_us = 0;
        isr_p1_hist[i].start_seq = 0;
        isr_p1_hist[i].end_seq = 0;
        isr_p1_hist[i].valid = false;
        isr_p1_hist[i].ended = false;
    }

    for (int i = 0; i < P2_HIST_SIZE; i++) {
        isr_p2_hist[i].start_us = 0;
        isr_p2_hist[i].end_us = 0;
        isr_p2_hist[i].width_us = 0;
        isr_p2_hist[i].start_seq = 0;
        isr_p2_hist[i].end_seq = 0;
        isr_p2_hist[i].valid = false;
        isr_p2_hist[i].ended = false;
    }

    // 부팅/초기화 시점의 실제 핀 상태와 ISR 내부 상태를 맞춤
    isr_p1_active = (gpio_get(DETECT_1_P1) == 0);
    if (isr_p1_active) {
        isr_p1_start_us = now;
        isr_p1_start_seq = ++sig_event_seq;
    }
    else {
        isr_p1_start_us = 0;
        isr_p1_start_seq = 0;
    }

    isr_p2_active = (gpio_get(DETECT_2_P2) == 0);
    if (isr_p2_active) {
        isr_p2_start_us = now;
        isr_p2_start_seq = ++sig_event_seq;

        uint8_t idx = isr_p2_write_idx;
        isr_p2_write_idx = (uint8_t)((isr_p2_write_idx + 1u) % P2_HIST_SIZE);
        isr_p2_active_idx = idx;

        isr_p2_hist[idx].start_us = now;
        isr_p2_hist[idx].end_us = 0;
        isr_p2_hist[idx].width_us = 0;
        isr_p2_hist[idx].start_seq = isr_p2_start_seq;
        isr_p2_hist[idx].end_seq = 0;
        isr_p2_hist[idx].valid = true;
        isr_p2_hist[idx].ended = false;
    }
    else {
        isr_p2_start_us = 0;
        isr_p2_start_seq = 0;
    }

    restore_interrupts(irq);

    gpio_set_irq_enabled_with_callback(DETECT_1_P1,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &gpio_irq_callback);
    gpio_set_irq_enabled(DETECT_2_P2,
                         GPIO_IRQ_EDGE_FALL,
                         true);

    // gpio_init(DETECT_3);
    // gpio_set_dir(DETECT_3, GPIO_IN);
    // gpio_pull_up(DETECT_3);
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
        // active-low P1: FALL만 사용한다. RISE 인터럽트는 사용하지 않는다.
        if (events & GPIO_IRQ_EDGE_FALL) {
            p1_irq_fall_cnt++;

            // 이미 1ms x 5회 확인 중이면 새 FALL은 무시한다.
            // P1은 active-low라서 HIGH 복귀 후 다시 LOW가 될 때만 FALL이 다시 발생한다.
            if (!p1_sample_running && !p1_confirm_pending) {
                uint32_t seq = ++sig_event_seq;

                isr_p1_active = true;
                isr_p1_start_us = now;
                isr_p1_start_seq = seq;

                p1_sample_running = true;
                p1_sample_count = 0;
                p1_sample_start_us = now;
                p1_sample_start_seq = seq;

                // SBC 진단용: FALL을 Pico가 읽은 순간 갱신.
                // 아직 5회 LOW 확인 전이므로 width는 0으로 초기화한다.
                p1_irq_start_us = now;
                p1_irq_width_us = 0;

                alarm_id_t alarm_id = add_alarm_in_us(P1_SAMPLE_INTERVAL_US,
                                                    p1_sample_alarm_callback,
                                                    NULL,
                                                    true);
                if (alarm_id < 0) {
                    // alarm 등록 실패 시 샘플링 상태가 영구 busy가 되지 않도록 해제
                    p1_sample_running = false;
                    p1_sample_count = 0;
                    isr_p1_active = false;
                }
            }
        }
    }
    else if (gpio == DETECT_2_P2) {
        // active-low P2: FALL만 사용한다. RISE 인터럽트는 사용하지 않는다.
        // 실제 channel 2 판정은 P1 확인 완료 + 40ms 뒤에 P2 핀을 1ms x 5회 샘플링하여 수행한다.
        if (events & GPIO_IRQ_EDGE_FALL) {
            uint32_t seq = ++sig_event_seq;
            isr_p2_start_us = now;
            isr_p2_start_seq = seq;

            if (current_detect_mode == DETECT_MODE_P2_ONLY) {
                p2_only_fall_us = now;
                p2_only_fall_pending = true;
            }

            // FALL 발생 시각만 기록한다. RISE를 쓰지 않으므로 pulse 폭은 계산하지 않는다.
            uint8_t idx = isr_p2_write_idx;
            isr_p2_write_idx = (uint8_t)((isr_p2_write_idx + 1u) % P2_HIST_SIZE);
            isr_p2_active_idx = ACTIVE_IDX_NONE;
            isr_p2_active = false;

            isr_p2_hist[idx].start_us = now;
            isr_p2_hist[idx].end_us = now;
            isr_p2_hist[idx].width_us = 0;
            isr_p2_hist[idx].start_seq = seq;
            isr_p2_hist[idx].end_seq = seq;
            isr_p2_hist[idx].valid = true;
            isr_p2_hist[idx].ended = true;
        }
    }
}

static int64_t p1_sample_alarm_callback(alarm_id_t id, void *user_data)
{
    (void)id;
    (void)user_data;

    if (!p1_sample_running) {
        return 0;
    }

    // active-low: 0이면 LOW 유지, 1이면 HIGH 복귀로 판단하여 실패
    if (gpio_get(DETECT_1_P1) != 0) {
        p1_sample_running = false;
        p1_sample_count = 0;
        isr_p1_active = false;
        return 0;
    }

    p1_sample_count++;

    if (p1_sample_count >= P1_SAMPLE_COUNT) {
        uint32_t end_us = time_us_32();
        uint32_t seq = ++sig_event_seq;
        uint32_t width_us = end_us - p1_sample_start_us;

        p1_confirm_start_us = p1_sample_start_us;
        p1_confirm_end_us = end_us;
        p1_confirm_width_us = width_us;
        p1_confirm_seq = seq;
        p1_confirm_pending = true;

        // SBC 진단용: P1이 1ms x 5회 모두 LOW였을 때만 end/width/seq 갱신
        p1_irq_end_us = end_us;
        p1_irq_width_us = width_us;
        p1_irq_end_seq = seq;

        p1_sample_running = false;
        p1_sample_count = 0;
        isr_p1_active = false;
        return 0;
    }

    return (int64_t)P1_SAMPLE_INTERVAL_US;
}

static inline bool time_after_eq_u32(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) >= 0;
}

static inline bool time_after_u32(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) > 0;
}

static inline bool time_reached_u32(uint32_t now, uint32_t target)
{
    return (int32_t)(now - target) >= 0;
}

static void signal_get_snapshot(signal_snapshot_t *s)
{
    uint32_t irq = save_and_disable_interrupts();

    s->p1_active = isr_p1_active;
    s->p2_active = isr_p2_active;
    s->p1_start_us = isr_p1_start_us;
    s->p2_start_us = isr_p2_start_us;
    s->latest_p1_seq = last_used_p1_seq;

    for (int i = 0; i < P1_HIST_SIZE; i++) {
        s->p1_hist[i].start_us = isr_p1_hist[i].start_us;
        s->p1_hist[i].end_us = isr_p1_hist[i].end_us;
        s->p1_hist[i].width_us = isr_p1_hist[i].width_us;
        s->p1_hist[i].start_seq = isr_p1_hist[i].start_seq;
        s->p1_hist[i].end_seq = isr_p1_hist[i].end_seq;
        s->p1_hist[i].valid = isr_p1_hist[i].valid;
        s->p1_hist[i].ended = isr_p1_hist[i].ended;

        if (s->p1_hist[i].valid && s->p1_hist[i].ended &&
            time_after_u32(s->p1_hist[i].end_seq, s->latest_p1_seq)) {
            s->latest_p1_seq = s->p1_hist[i].end_seq;
        }
    }

    for (int i = 0; i < P2_HIST_SIZE; i++) {
        s->p2_hist[i].start_us = isr_p2_hist[i].start_us;
        s->p2_hist[i].end_us = isr_p2_hist[i].end_us;
        s->p2_hist[i].width_us = isr_p2_hist[i].width_us;
        s->p2_hist[i].start_seq = isr_p2_hist[i].start_seq;
        s->p2_hist[i].end_seq = isr_p2_hist[i].end_seq;
        s->p2_hist[i].valid = isr_p2_hist[i].valid;
        s->p2_hist[i].ended = isr_p2_hist[i].ended;
    }

    restore_interrupts(irq);
}

static bool signal_find_next_p1_pulse(const signal_snapshot_t *s, active_low_pulse_t *out)
{
    bool found = false;
    active_low_pulse_t best = {0};

    for (int i = 0; i < P1_HIST_SIZE; i++) {
        const active_low_pulse_t *p = &s->p1_hist[i];

        if (!p->valid || !p->ended) {
            continue;
        }

        if (!time_after_u32(p->end_seq, last_used_p1_seq)) {
            continue;
        }

        if (!found || time_after_u32(best.end_seq, p->end_seq)) {
            best = *p;
            found = true;
        }
    }

    if (found && out) {
        *out = best;
    }

    return found;
}

static bool signal_take_confirmed_p1(uint32_t *start_us, uint32_t *end_us, uint32_t *width_us, uint32_t *seq)
{
    bool found = false;

    uint32_t irq = save_and_disable_interrupts();

    if (p1_confirm_pending && time_after_u32(p1_confirm_seq, last_used_p1_seq)) {
        if (start_us) {
            *start_us = p1_confirm_start_us;
        }
        if (end_us) {
            *end_us = p1_confirm_end_us;
        }
        if (width_us) {
            *width_us = p1_confirm_width_us;
        }
        if (seq) {
            *seq = p1_confirm_seq;
        }
        p1_confirm_pending = false;
        found = true;
    }
    else if (p1_confirm_pending) {
        p1_confirm_pending = false;
    }

    restore_interrupts(irq);

    return found;
}

static void signal_clear_p1_pending(void)
{
    uint32_t irq = save_and_disable_interrupts();

    p1_sample_running = false;
    p1_sample_count = 0;
    p1_confirm_pending = false;
    isr_p1_active = false;
    last_used_p1_seq = sig_event_seq;

    restore_interrupts(irq);
}

static bool p2_active_low_covers_window(const signal_snapshot_t *s,uint32_t start_us,uint32_t end_us,uint32_t now)
{
    for (int i = 0; i < P2_HIST_SIZE; i++) {
        const active_low_pulse_t *p = &s->p2_hist[i];

        if (!p->valid) {
            continue;
        }

        // P2 LOW 시작이 판정 시작보다 늦으면 해당 pulse는 조건 불만족
        if (!time_after_eq_u32(start_us, p->start_us)) {
            continue;
        }

        // 아직 LOW 유지 중이면 현재 시간이 end_us를 지난 뒤에만 유효 처리
        if (!p->ended) {
            if (time_reached_u32(now, end_us)) {
                return true;
            }
            continue;
        }

        // P2 LOW 종료가 필요한 종료시각보다 같거나 뒤면 유효
        if (time_after_eq_u32(p->end_us, end_us)) {
            return true;
        }
    }

    return false;
}

static bool signal_take_p2_only_fall(uint32_t *fall_us)
{
    bool found = false;
    uint32_t irq = save_and_disable_interrupts();

    if (p2_only_fall_pending) {
        if (fall_us != NULL) {
            *fall_us = p2_only_fall_us;
        }
        p2_only_fall_pending = false;
        found = true;
    }

    restore_interrupts(irq);
    return found;
}

static void register_detected_hit(int report_channel, int auto_channel)
{
    // SBC에는 감지 결과를 항상 1회 전달한다.
    hit_channel = report_channel;

    // 자동 모드에서만 모터 시나리오용 내부 카운트와 pending을 갱신한다.
    // 감지 전용 모드에서는 SBC 전달만 하고 모터는 움직이지 않는다.
    if (current_motor_mode != 0u || stop_hold_latched || emergency_stop_active) {
        pending_auto_hit_channel = 0;
        return;
    }

    if (auto_channel == 2) {
        pending_auto_hit_channel = 2;
        cri_hit_count += 1;
    }
    else if (auto_channel == 1) {
        pending_auto_hit_channel = 1;
        non_cri_hit_count += 1;
    }
    else {
        pending_auto_hit_channel = 0;
    }
}

static void signal_detector_reset(void)
{
    sig_state = SIG_IDLE;
    active_p1_end_us = 0;
    active_p1_seq = 0;
    sig_cooldown_start_us = 0;
    p2_sample_next_us = 0;
    p2_sample_count = 0;

    // reset 전에 이미 들어온 P1/P2 부분 신호는 처리하지 않는다.
    signal_clear_p1_pending();

    uint32_t irq = save_and_disable_interrupts();
    p2_only_fall_pending = false;
    p2_only_fall_us = 0;
    restore_interrupts(irq);
}

static void signal_detector_update(void)
{
    // motor stop, 수동 이동, 자동 동작 중에는 센서 판정을 하지 않는다.
    if (!control_signal) {
        signal_detector_reset();
        return;
    }

    // 채널 ban은 기존 P1+P2 모드에만 적용한다.
    // 비상 P2-only 모드는 채널 ban과 무관하게 피격 여부를 확인한다.
    if (current_detect_mode == DETECT_MODE_P1_P2 && first_channel_ban) {
        signal_detector_reset();
        return;
    }

    uint32_t now = time_us_32();

    switch (sig_state) {
    case SIG_IDLE:
        if (current_detect_mode == DETECT_MODE_P2_ONLY) {
            uint32_t fall_us = 0;

            if (signal_take_p2_only_fall(&fall_us)) {
                p2_sample_count = 0;
                p2_sample_next_us = fall_us + P2_SAMPLE_INTERVAL_US;
                sig_state = SIG_CONFIRM_P2_ONLY;
            }
        }
        else {
            uint32_t p1_start_us = 0;
            uint32_t p1_end_us = 0;
            uint32_t p1_width_us = 0;
            uint32_t p1_seq = 0;

            if (signal_take_confirmed_p1(&p1_start_us,
                                         &p1_end_us,
                                         &p1_width_us,
                                         &p1_seq)) {
                // P1 FALL 이후 1ms 간격으로 5번 모두 LOW였던 경우만 진입.
                (void)p1_start_us;
                (void)p1_width_us;

                last_used_p1_seq = p1_seq;
                active_p1_end_us = p1_end_us;
                active_p1_seq = p1_seq;
                p2_sample_count = 0;
                p2_sample_next_us = active_p1_end_us +
                                    P1_P2_DELAY_US +
                                    P2_SAMPLE_INTERVAL_US;
                sig_state = SIG_WAIT_P2_DELAY;
            }
        }
        break;

    case SIG_WAIT_P2_DELAY: {
        if (current_detect_mode != DETECT_MODE_P1_P2) {
            signal_detector_reset();
            break;
        }

        uint32_t p2_check_start_us = active_p1_end_us + P1_P2_DELAY_US;

        // P1 LOW 확인 완료 후 40ms 단순 지연
        if (time_reached_u32(now, p2_check_start_us)) {
            p2_sample_count = 0;
            p2_sample_next_us = p2_check_start_us + P2_SAMPLE_INTERVAL_US;
            sig_state = SIG_CONFIRM_P2;
        }
        break;
    }

    case SIG_CONFIRM_P2:
        if (current_detect_mode != DETECT_MODE_P1_P2) {
            signal_detector_reset();
            break;
        }

        if (time_reached_u32(now, p2_sample_next_us)) {
            bool p2_low = (gpio_get(DETECT_2_P2) == 0);

            if (!p2_low) {
                // P2 5ms LOW 확인 실패 -> non-critical / channel 1
                register_detected_hit(1, 1);

                sig_cooldown_start_us = now;
                p2_sample_count = 0;
                p2_sample_next_us = 0;
                sig_state = SIG_COOLDOWN;
                break;
            }

            p2_sample_count++;

            if (p2_sample_count >= P2_SAMPLE_COUNT) {
                if (!second_channel_ban) {
                    // P2 1ms x 5회 성공 -> critical / channel 2
                    register_detected_hit(2, 2);
                }
                else {
                    // channel 2 ban이면 기존 로직처럼 channel 1 처리
                    register_detected_hit(1, 1);
                }

                sig_cooldown_start_us = now;
                p2_sample_count = 0;
                p2_sample_next_us = 0;
                sig_state = SIG_COOLDOWN;
            }
            else {
                p2_sample_next_us += P2_SAMPLE_INTERVAL_US;
            }
        }
        break;

    case SIG_CONFIRM_P2_ONLY:
        if (current_detect_mode != DETECT_MODE_P2_ONLY) {
            signal_detector_reset();
            break;
        }

        if (time_reached_u32(now, p2_sample_next_us)) {
            if (gpio_get(DETECT_2_P2) != 0) {
                // 5ms 유지 전 HIGH로 복귀하면 유효 피격이 아니다.
                p2_sample_count = 0;
                p2_sample_next_us = 0;
                sig_state = SIG_IDLE;
                break;
            }

            p2_sample_count++;

            if (p2_sample_count >= P2_SAMPLE_COUNT) {
                // 외부 전달값 3 = P2-only 비상 감지.
                // 자동 모드에서는 기존 critical 시나리오를 사용한다.
                register_detected_hit(3, 2);

                sig_cooldown_start_us = now;
                p2_sample_count = 0;
                p2_sample_next_us = 0;
                sig_state = SIG_COOLDOWN;
            }
            else {
                p2_sample_next_us += P2_SAMPLE_INTERVAL_US;
            }
        }
        break;

    case SIG_COOLDOWN:
        if ((uint32_t)(now - sig_cooldown_start_us) >= DETECT_COOLDOWN_US) {
            // 200ms 동안 들어온 P1/P2 FALL과 부분 확인 결과는 모두 무시.
            signal_detector_reset();
        }
        break;

    default:
        signal_detector_reset();
        break;
    }
}

static void detect_channel_ban(void)
{
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
