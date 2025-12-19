from machine import Pin
import machine
import utime
import micropython

DETECT_1 = 3
DETECT_2 = 4
DETECT_3 = 5

HIT_1 = 12   # DETECT_2용 출력
HIT_2 = 13   # DETECT_1용 출력
HIT_3 = 14   # DETECT_3용 출력

led = Pin("LED", Pin.OUT)

# GPIO Setup
detect1 = Pin(DETECT_1, Pin.IN, Pin.PULL_DOWN)
detect2 = Pin(DETECT_2, Pin.IN, Pin.PULL_DOWN)
detect3 = Pin(DETECT_3, Pin.IN, Pin.PULL_DOWN)

hit1 = Pin(HIT_1, Pin.OUT); hit1.value(0)
hit2 = Pin(HIT_2, Pin.OUT); hit2.value(0)
hit3 = Pin(HIT_3, Pin.OUT); hit3.value(0)

# Interrupt flags
flag1 = False
flag2 = False
flag3 = False

def _irq_detect1(_pin):
    global flag1
    flag1 = True

def _irq_detect2(_pin):
    global flag2
    flag2 = True

def _irq_detect3(_pin):
    global flag3
    flag3 = True

detect1.irq(trigger=Pin.IRQ_RISING, handler=_irq_detect1)
detect2.irq(trigger=Pin.IRQ_RISING, handler=_irq_detect2)
detect3.irq(trigger=Pin.IRQ_RISING, handler=_irq_detect3)

# Helpers (원본 로직 유지)
def start_signal():
    led.value(1)
    utime.sleep_ms(3000)
    led.value(0)
    utime.sleep_ms(1000)

def all_hit_off():
    led.value(0)
    hit1.value(0)
    hit2.value(0)
    hit3.value(0)

# 1ms 간격으로 총 5번 High 유지 확인
def readsignal_detect(pin: Pin) -> bool:
    if pin.value() == 0:
        return False
    for _ in range(5):
        if pin.value() == 0:
            return False
        utime.sleep_ms(1)
    return True

# HIT High를 10ms 동안 출력
def send_pulse_for_detect_1():
    # DETECT_1 → HIT_2
    if readsignal_detect(detect1):
        led.value(1)
        hit1.value(0)
        hit2.value(1)
        hit3.value(0)
        print("D1")
        utime.sleep_ms(10)
    all_hit_off()

def send_pulse_for_detect_2():
    # DETECT_2 → HIT_1
    if readsignal_detect(detect2):
        led.value(1)
        hit1.value(1)
        hit2.value(0)
        hit3.value(0)
        print("D2")
        utime.sleep_ms(10)
    all_hit_off()

def send_pulse_for_detect_3():
    # DETECT_3 → HIT_3
    if readsignal_detect(detect3):
        led.value(1)
        hit1.value(0)
        hit2.value(0)
        hit3.value(1)
        print("D3")
        utime.sleep_ms(10)
    all_hit_off()

def main_loop():
    global flag1, flag2, flag3

    while True:
        irq_state = machine.disable_irq()
        d1 = flag1; flag1 = False
        d2 = flag2; flag2 = False
        d3 = flag3; flag3 = False
        machine.enable_irq(irq_state)

        if d1:
            send_pulse_for_detect_1()
        if d2:
            send_pulse_for_detect_2()
        if d3:
            send_pulse_for_detect_3()

        utime.sleep_ms(1)

start_signal()
main_loop()
