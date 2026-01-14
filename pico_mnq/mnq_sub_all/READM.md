각 파일 로직

1. 테스트 파일 로직
    <--- 1214_단순3초 up 3초 대기.py --->
    - sub pcb, motor driver용 pcb 사용
    - 센서 신호는 읽되 신호 상관 없이 3초 up down, 3초 대기

    <--- sub_pcb_mnq.c 구버전 --->
    - sub pcb, motor driver용 pcb 사용
    - 전체적인 MNQ 로직을 sub pcb가 제어

    <--- test_act_mot.c  --->
    - pwm 없이 단순 high, low 사용 Motor(ACT) 동작 잘 하는지 확인 용도

    <--- test_act_mot.c  --->

2. sub pcb to main pcb 인식
    <--- 의도 --->
    detect 3 → hit 12 → get_hit 10
    detect 4 → hit 13 → get_hit 11
    detect 5 → hit 14 → get_hit 12

    <--- 실제 --->
    detect 3 → hit 12 → get_hit 11
    detect 4 → hit 13 → get_hit 10
    detect 5 → hit 14 → get_hit 12

    <--- 변경 --->
    detect 3 → hit 13 → get_hit 10
    detect 4 → hit 12 → get_hit 11
    detect 5 → hit 14 → get_hit 12


3. P05, P05L limit SW board
    NC ↔ 1k ↔ 3.3V
    NC ↔ gpio pin

    COM ↔ GND

    gpio pin ↔ 0.1uF ↔ GND 

    * SW 로직
    - 넘어갈 때 위 sw, 올라올 때 아래 sw


4. P06 limit SW board
    NC ↔ 3.3k ↔ 3.3V
    NC ↔ gpio pin

    COM ↔ GND

    gpio pin ↔ 0.1uF ↔ GND

    * SW 로직
    - 넘어갈 때 아래 sw, 올라올 때 위 sw


5. MNQ type
    A type = 탄 감지
    E type = 음향 감지

    P05, P05L = A type

    P06 = E type


6. MNQ cable
    A type = 4 pin
     └ 적(HIT_1) 녹(HIT_2) 흰(HIT_3) 검(GND)
     * 방수 cable 4pin 솔더링 시 위치 확인 필요

    E type = 6 pin
     └ 적(12V) 노(HIT_S1) 파(HIT_S2) 흰(HIT_S3) 녹(Gain) 검(GND)
     * 5559-06P, 5559-06R 핀 작업 시 위치 확인 필요


7. SUB pcb  (탄 감지 및 Limit sw, Motor[ACT], Mode)
 7-1) Detect pin
  a) Detect_1 = gpio 3
  b) Detect_2  = gpio 4
  c) Detect_3  = gpio 5

 7-2) Hit pin [send main pcb]
  a) HIT_1 = gpio 12
  b) HIT_2 = gpio 13
  c) HIT_3 = gpio 14

 7-3) Motor[ACT] pin
  a) MNQ_DIR = gpio 10
  b) MNQ_PWM = gpio 9

 7-4) Limit_sw pin
  a) LIMIT_SW_1 = gpio 17
  b) LIMIT_SW_2 = gpio 18

 7-5) Mode pin [get main pcb] / pico to pico 3.3v 로직
  a) Mode = 
   * 자동(탄 감지) / 수동(직접 제어)
  b) Move(?) = 
   * 수동 일 시 Up/Down 명령 받기