# MNQ_Detect
Work in progress — detailed description coming soon

1) MNQ 탄 감지 확인 (신호 3개, 인터럽트)

 1-1) 신호 3개를 통해 헤드샷 또는 몸통샷을 판별함
 
 1-2) DETECT_1, DETECT_3는 몸통샷, DETECT_2는 헤드샷 [low to high 이후 low]
 
    - 몸통샷은 DETECT_1 또는 DETECT_3 신호가 한 번 더 들어오면 내려가기
    
    - 헤드샷 은 DETECT_2 신호 한 번 들어오자 마자 내려가기
    
 1-3) 내려갔을 시 3초 대기(신호 받지 않음/high 신호 들어와도 무시) 후 다시 올라오기
 
 1-4) 올라오고 1초 대기(신호 받지 않음/high 신호 들어와도 무시) 후 다시 신호 받기
 
 1-5) 반복



2) MNQ_MOT 제어신호 송출 (PWM 신호는 HIGH/LOW 아닌 PWM 제어, PWM 주파수는 16kHz)
   
 2-1) 0에서 255/255로 바로 주지말고, 1ms에 PWM 5씩 증가 (51ms면 255/255)
   
 2-2) 255에서 몇 초동안 넘어가고 올라오는지 확인할 것 [내려가기까지 약 1.2초, 올라가기 까지 대략 1.7초]
 
 2-3) 2.5초라면, 1.5초동안 255/255로 제어하고, 그 다음부터는 150/255로 제어할 것
 
    - 단, 255 → 150은 1ms에 PWM 1씩 감소시킬 것 (105ms 소요)
    
    ※ 2초 수준이면 1.2초만 풀파워, 2.5초면 1.5초만 풀파워 등, 내려갈때 올라갈 때 모두 체크 필요
    
 2-4) 엔드스탑 스위치신호 인식되면 150 → 0으로 1ms에 PWM 3씩 감소시킬 것 (스위치 닿고 멈추는데 50ms 소요)
 
 * 번 외 MNQ가 올라가있으면 LED high, 내려가있으면 LED low
   
 * 엔드스탑 스위치로직은 아래 참고 로직으로 반영할 것

```
 /*엔드스탑 스위치 참고 로직*/
    // main 함수 밖에 변수 선언
    static bool up_stop = true;
    static bool down_stop = false;
    static bool up_statue = true;   // 내려갈 시 해당 값은 false 올라갈 시 해당 값은 true여야 함

        if (gpio_get(LIMIT_UP)==0) {
            if (up_stop == false && down_stop == true)
            {
                up_statue = true;
                gpio_put(MNQ_PWM, 0);
                up_stop = true;
                down_stop = false;
            }
        } else if (gpio_get(LIMIT_DOWN)==0) {
            if (up_stop == true && down_stop == false)
            {
                up_statue = false;
                gpio_put(MNQ_PWM, 0);
                up_stop = false;
                down_stop = true;
            }
        }
```



3) MNQ 탄 감지 송출 [단순 high low 신호 보내기 / 기능은 넣어두되 사용은 추후에 할 수 있으니 주석처리만]
   
 3-1) 몸통샷 한 번이면 HIT_1 = high
 
 3-2) 몸통샷 두 번이면 HIT_2 = high
 
 3-3) 헤드샷 이면 HIT_3 = high
 
 3-4) MNQ_MOT가 내려가면 전부 LOW로 초기화
 
    - HIT_1 또는 HIT_2가 high일 때 HIT_3가 high가 되면 LOW로 초기화



4) pico pin
   
 4-1) 신호 IN [풀다운 안해주어도 됨]
 
 gpio 3 = DETECT_1
 
 gpio 4 = DETECT_2
 
 gpio 5 = DETECT_3

 4-2) 모터 [Direction high면 내려감 low면 올라감]
 
 gpio 8 = Direction
 
 gpio 9 = PWM

 4-3) 엔드스탑 스위치 [눌려있지 않으면 high, 눌리면 low]
 
 gpio 16 = limit_sw_under (아래) → 올라갈시 눌리면 정지
 
 gpio 15 = limit_sw_top (위) → 내려갈시 눌리면 정지

 4-4) 신호 OUT
 
 gpio 12 = HIT_1
 
 gpio 13 = HIT_2
 
 gpio 14 = HIT_3



; 12/03 탄 감지 로직

;  1-1) XTGT 피코는 신호 2개를 통해 헤드샷 또는 몸통샷을 판별함(신호 P1, P2).

;  1-2) 먼저 P1이 LOW to HIGH 될 때를 감지하는데, 

;    - P1이 HIGH임을 1ms 간격으로 5회 측정(총 5ms) 후 P1이 HIGH임을 확정.

;  1-3) P1이 HIGH임을 확인하면, P1이 LOW로 변경될 때를 기다림.

;    - P1이 LOW가 되는 즉시 P1이 LOW임을 확정.

;  1-4) P2 신호 확인시작

;    - P1 LOW 확정 후, 1ms delay 후 P2 확인시작 함

;    - P2를 1ms 간격으로 2회 체크해서 P2가 HIGH 인지 LOW인지 확인

;  1-5) P2가 HIGH이면 헤드샷, LOW이면 몸통샷으로 판별함.
