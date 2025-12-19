각 파일 로직

1. sub_pcb_mnq.c
- sub pcb, motor driver용 pcb 사용
- 전체적인 MNQ 로직을 sub pcb가 제어

2. sub_pico_mnq_1.c
- 인터럽트 신호 확인
- 단순 신호 high로 읽었을 시 펄스 확인 후 main pcb로 신호 전달

3. sub_pico_mnq_2.c
- 인터럽트 신호 확인
- low to high 상승 엣지 확인 펄스 확인 후 main pcb로 신호 전달