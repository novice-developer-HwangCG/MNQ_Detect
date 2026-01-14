1. sub_pico_mnq_P1_P2.c
- 이전 작성해놨던 센서 2개만 보는 코드 리팩토링 한 것
- P2 값이 High이면 헤드 한 번에 넘어가기 Low이면 몸통샷으로 하여 2번 전달하면 넘어가기
- 0 0 => 정상 / 0, 1 or 1, 0 = 몸통 / 1, 1 = 헤드

2. sub_pico_mnq_P1_P2_P3.c
- sub_pico_mnq_P1_P2.c 코드는 센서 2개만 보았는데 이제 3개 다 봐야 함
- 코딩 안 되어 있음

3. P1_P2_test_1.c
- 인터럽트 신호 확인
- 단순 신호 high로 읽었을 시 펄스 확인 후 main pcb로 신호 전달
- 센서 2개만 사용 (DETECT_3 사용 X)

4. P1_P2_test_2.c
- 인터럽트 신호 확인
- low to high 상승 엣지 확인 펄스 확인 후 main pcb로 신호 전달
- 센서 2개만 사용 (DETECT_3 사용 X)