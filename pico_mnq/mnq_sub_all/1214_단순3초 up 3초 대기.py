from machine import Pin
import utime

# ------------ pin set ------------

DETECT_1_PIN = 3   # body
DETECT_2_PIN = 4   # head
DETECT_3_PIN = 5   # body

# ACT
MNQ_DIR_PIN   = 8   # Direction: HIGH=down, LOW=up
MNQ_PWM_PIN   = 9

# limit sw (normal state = HIGH, press state = LOW)
LIMIT_SW_UNDER_PIN = 16  # under → 올라갈 때 눌리면 정지
LIMIT_SW_TOP_PIN   = 15  # top   → 내려갈 때 정지

# output (to main pcb)
HIT_1_PIN = 12
HIT_2_PIN = 13
HIT_3_PIN = 14

LED_PIN = 25

MOVE_MS  = 3000
PAUSE_MS = 3000

# ------------ 핀 객체 ------------

led = Pin(LED_PIN, Pin.OUT)

detect1 = Pin(DETECT_1_PIN, Pin.IN, Pin.PULL_DOWN)
detect2 = Pin(DETECT_2_PIN, Pin.IN, Pin.PULL_DOWN)
detect3 = Pin(DETECT_3_PIN, Pin.IN, Pin.PULL_DOWN)

mnq_dir = Pin(MNQ_DIR_PIN, Pin.OUT)
mnq_pwm = Pin(MNQ_PWM_PIN, Pin.OUT)
mnq_pwm.value(0)   # 초기 OFF

limit_under = Pin(LIMIT_SW_UNDER_PIN, Pin.IN, Pin.PULL_UP)
limit_top   = Pin(LIMIT_SW_TOP_PIN, Pin.IN, Pin.PULL_UP)

hit1 = Pin(HIT_1_PIN, Pin.OUT)
hit2 = Pin(HIT_2_PIN, Pin.OUT)
hit3 = Pin(HIT_3_PIN, Pin.OUT)

hit1.value(0)
hit2.value(0)
hit3.value(0)

# ------------ 유틸 ------------

def now_ms():
    return utime.ticks_ms()

def StartSignal():
    led.value(1)
    utime.sleep_ms(3000)
    led.value(0)
    utime.sleep_ms(1000)

# ------------ 메인 동작 루프 ------------

def move_down_for(ms_limit):
    print("MOVE DOWN")
    mnq_dir.value(1)
    mnq_pwm.value(1)

    start = now_ms()
    while utime.ticks_diff(now_ms(), start) < ms_limit:
        if limit_top.value() == 0:
            print("TOP limit, stop down")
            break
        utime.sleep_ms(1)

    mnq_pwm.value(0)

def move_up_for(ms_limit):
    print("MOVE UP")
    mnq_dir.value(0)
    mnq_pwm.value(1)

    start = now_ms()
    while utime.ticks_diff(now_ms(), start) < ms_limit:
        if limit_under.value() == 0:
            print("UNDER limit hit, stop up")
            break
        utime.sleep_ms(1)

    mnq_pwm.value(0)

def pause_for(ms_limit):
    start = now_ms()
    while utime.ticks_diff(now_ms(), start) < ms_limit:
        # print("D1=", detect1.value(), " D2=", detect2.value(), " D3=", detect3.value())
        utime.sleep_ms(10)

def main():
    #StartSignal()
    led.value(1)

    while True:
        move_down_for(MOVE_MS)
        pause_for(PAUSE_MS)

        move_up_for(MOVE_MS)
        pause_for(PAUSE_MS)

# 실행
main()

