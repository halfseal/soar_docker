#!/bin/bash

# px4_sitl gazebo-classic 실행
HEADLESS=1 make px4_sitl gazebo-classic &

# 실행된 프로세스의 PID를 저장
PX4_PID=$!

# 일정 시간(예: 10초) 대기 후 프로세스 종료
sleep 200s

# 프로세스 종료
kill $PX4_PID