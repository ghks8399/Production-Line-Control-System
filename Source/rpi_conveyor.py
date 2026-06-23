"""

rpi_conveyor.py
===============
Conveyor System - Raspberry Pi RS232 communication code
MFC(PC) <-> Raspberry Pi

Hardware:
  - RS232 TX/RX  : GPIO14(TX), GPIO15(RX)  via MAX3232 level converter
  - DHT11 sensor : GPIO26 (BCM) - 3pin module
  - DC Motor IN1 : GPIO12 (BCM) - PWM (speed)
  - DC Motor IN2 : GPIO13 (BCM) - Direction fixed LOW
  - Stepper IN1  : GPIO17 (BCM)
  - Stepper IN2  : GPIO18 (BCM)
  - Stepper IN3  : GPIO27 (BCM)
  - Stepper IN4  : GPIO22 (BCM)
  - Servo Motor  : GPIO25 (BCM) - PWM (defect reject)

Control mapping:
  Conveyor ON/OFF  → DC 모터 ON/OFF
  Speed 1/2/3      → DC 듀티 50/75/100%
  Emergency Stop   → DC 모터 정지
  Fan ON/OFF       → 스텝모터 ON/OFF
  Defect Servo     → 서보모터로 불량품 쳐내기

Run:
  sudo python3 rpi_conveyor.py
"""

import serial
import time
import threading
import lgpio
from collections import deque

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Protocol constants
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PACKET_SIZE = 9
STX = 0x02
ETX = 0x03

IDX_STX  = 0
IDX_CMD  = 1
IDX_LEN  = 2
IDX_TEMP = 3
IDX_HUMI = 4
IDX_ULTR = 5
IDX_MOTR = 6
IDX_CS   = 7
IDX_ETX  = 8

CMD_STATUS_REPORT    = 0x10
CMD_CONVEYOR_CONTROL = 0x20
CMD_SPEED_SET        = 0x21
CMD_FAN_CONTROL      = 0x22
CMD_EMG_STOP         = 0x23
CMD_DEFECT_SERVO     = 0x24    # Defect detected -> activate servo to reject
CMD_ACK  = 0x70
CMD_NACK = 0x71

MOTOR_OFF     = 0x00
MOTOR_SPEED_1 = 0x01
MOTOR_SPEED_2 = 0x02
MOTOR_SPEED_3 = 0x03
MOTOR_EMG     = 0xFF

CTRL_OFF = 0x00
CTRL_ON  = 0x01

SERVO_REJECT = 0x01    # Activate servo to reject defect item


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# GPIO settings
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE   = 9600

DHT_PIN = 26

# DC motor (방향 바꾸려면 두 줄 swap)
DC_PWM_PIN = 12
DC_DIR_PIN = 13

# Stepper motor
STEP_IN1 = 17
STEP_IN2 = 23
STEP_IN3 = 27
STEP_IN4 = 22

# Servo motor (defect reject)
SERVO_PIN = 25


# Button swith
BTN_PIN = 7

# ir
IR_PIN = 6

# led_pin
LED_GREEN = 16
LED_YELLOW = 20
LED_RED = 21

# buzzer
BUZZER_PIN = 5
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Config
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
DC_PWM_FREQ = 1000

DC_SPEED_DUTY = {
    MOTOR_SPEED_1: 50,
    MOTOR_SPEED_2: 75,
    MOTOR_SPEED_3: 100,
}

STEP_SEQUENCE = [
    [1, 0, 1, 0],
    [0, 1, 1, 0],
    [0, 1, 0, 1],
    [1, 0, 0, 1],
]

STEP_DELAY = 0.012

# Servo config
SERVO_PWM_FREQ    = 50       # 서보 PWM 주파수 (50Hz = 20ms period)
SERVO_DUTY_HOME   = 2.5      # 0도 위치 (홈 포지션) 듀티 사이클 %
SERVO_DUTY_REJECT = 12.5     # 180도 위치 (불량 제거) 듀티 사이클 %
SERVO_HOLD_SEC    = 0.8      # 불량 제거 위치 유지 시간 (초)
SERVO_RETURN_SEC  = 0.5      # 원위치 복귀 후 대기 시간 (초)


# servo_delay
SERVO_DELAY = 6.1
# servo time
JUDGEMENT_GRACE = 5.0
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Helper
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def gpio_safe_free(h, gpio):
    try:
        lgpio.gpio_free(h, gpio)
    except lgpio.error:
        pass

def gpio_safe_claim_output(h, gpio, level=0):
    gpio_safe_free(h, gpio)
    lgpio.gpio_claim_output(h, gpio, level)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Packet utilities
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def calc_cs(pkt):
    return (pkt[IDX_CMD] ^ pkt[IDX_LEN] ^
            pkt[IDX_TEMP] ^ pkt[IDX_HUMI] ^
            pkt[IDX_ULTR] ^ pkt[IDX_MOTR]) & 0xFF

def build_packet(cmd, temp_byte, humi_byte, ultr_byte, motr):
    pkt = [0] * PACKET_SIZE
    pkt[IDX_STX]  = STX
    pkt[IDX_CMD]  = cmd        & 0xFF
    pkt[IDX_LEN]  = 5
    pkt[IDX_TEMP] = temp_byte  & 0xFF
    pkt[IDX_HUMI] = humi_byte  & 0xFF
    pkt[IDX_ULTR] = ultr_byte  & 0xFF
    pkt[IDX_MOTR] = motr       & 0xFF
    pkt[IDX_CS]   = calc_cs(pkt)
    pkt[IDX_ETX]  = ETX
    return bytes(pkt)

def validate_packet(data):
    if len(data) != PACKET_SIZE:  return False
    if data[IDX_STX] != STX:     return False
    if data[IDX_ETX] != ETX:     return False
    cs = (data[IDX_CMD] ^ data[IDX_LEN] ^
          data[IDX_TEMP] ^ data[IDX_HUMI] ^
          data[IDX_ULTR] ^ data[IDX_MOTR]) & 0xFF
    return data[IDX_CS] == cs

def send_ack(ser, success):
    cmd = CMD_ACK if success else CMD_NACK
    ser.write(build_packet(cmd, 0, 0, 0, 0))


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Serial packet reader with STX sync
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class PacketReader:
    """
    Byte-level sync: scans for STX, then reads remaining 8 bytes.
    Prevents packet misalignment from causing persistent failures.
    """
    def __init__(self, ser):
        self._ser = ser
        self._buf = bytearray()

    def read_packet(self):
        # Collect available bytes into buffer
        if self._ser.in_waiting > 0:
            self._buf.extend(self._ser.read(self._ser.in_waiting))

        # Scan for STX
        while len(self._buf) >= PACKET_SIZE:
            # Find STX position
            try:
                stx_pos = self._buf.index(STX)
            except ValueError:
                # No STX found, discard all
                self._buf.clear()
                return None

            # Discard bytes before STX
            if stx_pos > 0:
                print(f"[SYNC] Discarded {stx_pos} bytes before STX")
                self._buf = self._buf[stx_pos:]

            # Need at least PACKET_SIZE bytes from STX
            if len(self._buf) < PACKET_SIZE:
                return None

            # Extract candidate packet
            candidate = bytes(self._buf[:PACKET_SIZE])
            self._buf = self._buf[PACKET_SIZE:]

            if validate_packet(candidate):
                return candidate
            else:
                # Bad packet, skip this STX and try next one
                print(f"[SYNC] Invalid packet, re-syncing...")
                # Put back all except first byte (the bad STX)
                # Already removed PACKET_SIZE, just continue loop

        return None


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# DC Motor - conveyor belt (PWM)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DCMotor:
    def __init__(self, h, pwm_pin, dir_pin):
        self._h        = h
        self._pwm_pin  = pwm_pin
        self._dir_pin  = dir_pin
        self._speed    = MOTOR_OFF
        self._duty     = 0
        self._pwm_on   = False
        gpio_safe_claim_output(self._h, self._dir_pin, 0)
        gpio_safe_claim_output(self._h, self._pwm_pin, 0)

    def _stop_pwm(self):
        """PWM 확실히 정지: PWM 끄고 → 핀을 OUTPUT LOW로 재설정"""
        if self._pwm_on:
            lgpio.tx_pwm(self._h, self._pwm_pin, 0, 0)
            time.sleep(0.01)   # PWM 파형이 완전히 끝날 때까지 대기
        # 핀을 OUTPUT LOW로 강제 재설정 (이게 핵심)
        gpio_safe_claim_output(self._h, self._pwm_pin, 0)
        self._pwm_on = False

    def set_speed(self, speed):
        if speed == MOTOR_EMG or speed == MOTOR_OFF:
            self._speed = MOTOR_OFF
            self._duty  = 0
            self._stop_pwm()
            
            lgpio.gpio_write(self._h, self._dir_pin, 0)
        else:
            self._speed = speed
            self._duty  = DC_SPEED_DUTY.get(speed, 50)
            
            lgpio.tx_pwm(self._h, self._pwm_pin, DC_PWM_FREQ, self._duty)
            self._pwm_on = True

    def get_speed(self):  return self._speed
    def get_duty(self):   return self._duty

    def cleanup(self):
        self._stop_pwm()
        lgpio.gpio_write(self._h, self._dir_pin, 0)
        gpio_safe_free(self._h, self._pwm_pin)
        gpio_safe_free(self._h, self._dir_pin)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Stepper Motor - fan (ON/OFF only)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class StepMotor:
    def __init__(self, h, pins):
        self._h          = h
        self._pins       = pins
        self._running    = False
        self._step_idx   = 0
        self._lock       = threading.Lock()
        self._stop_event = threading.Event()
        self._thread     = threading.Thread(target=self._run, daemon=True)
        for pin in self._pins:
            gpio_safe_claim_output(self._h, pin, 0)
        self._thread.start()

    def _set_coils(self, pattern):
        for pin, val in zip(self._pins, pattern):
            lgpio.gpio_write(self._h, pin, val)

    def _all_off(self):
        for pin in self._pins:
            lgpio.gpio_write(self._h, pin, 0)

    def _run(self):
        while not self._stop_event.is_set():
            with self._lock:
                running = self._running
            if not running:
                self._all_off()
                time.sleep(0.05)
                continue
            self._set_coils(STEP_SEQUENCE[self._step_idx])
            self._step_idx = (self._step_idx + 1) % len(STEP_SEQUENCE)
            time.sleep(STEP_DELAY)

    def set_on(self, on):
        with self._lock:
            self._running = on

    def is_on(self):
        with self._lock:
            return self._running

    def cleanup(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        self._all_off()
        for pin in self._pins:
            gpio_safe_free(self._h, pin)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Servo Motor - defect reject (PWM 50Hz)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class ServoMotor:
    """
    SG90 등 표준 서보모터 제어 (PWM 50Hz)
    
    동작 흐름:
      1. 평소: 홈 포지션 (0도, duty 2.5%)
      2. 불량 검출 시: reject() 호출
         → 제거 위치 (180도, duty 12.5%) 로 회전
         → SERVO_HOLD_SEC 유지
         → 홈 포지션으로 복귀
      3. 별도 스레드에서 실행하므로 메인 루프 블로킹 없음
    """
    def __init__(self, h, pin):
        self._h    = h
        self._pin  = pin
        self._lock = threading.Lock()
        self._busy = False
        gpio_safe_claim_output(self._h, self._pin, 0)
        # 초기 위치: 홈 포지션
        self._set_angle(SERVO_DUTY_HOME)
        time.sleep(0.5)
        # PWM 멈춤 (서보 떨림 방지)
        lgpio.tx_pwm(self._h, self._pin, SERVO_PWM_FREQ, 0)

    def _set_angle(self, duty):
        """서보를 지정 듀티(각도)로 이동"""
        lgpio.tx_pwm(self._h, self._pin, SERVO_PWM_FREQ, duty)

    def reject(self):
        with self._lock:
            if self._busy:
                print("[SERVO] Already in reject motion, skipping")
                return
            self._busy = True
        threading.Thread(target=self._do_reject, daemon=True).start()

    def _do_reject(self):
        try:
            self._set_angle(SERVO_DUTY_REJECT)
            print(f"[SERVO] -> Reject position (duty={SERVO_DUTY_REJECT}%)")
            time.sleep(SERVO_HOLD_SEC)

            self._set_angle(SERVO_DUTY_HOME)
            print(f"[SERVO] -> Home position (duty={SERVO_DUTY_HOME}%)")
            time.sleep(SERVO_RETURN_SEC)

            lgpio.tx_pwm(self._h, self._pin, SERVO_PWM_FREQ, 0)
            print("[SERVO] Reject complete, PWM off")
        finally:
            with self._lock:
                self._busy = False

    def is_busy(self):
        with self._lock:
            return self._busy

    def cleanup(self):
        # 홈 포지션으로 복귀 후 PWM 정지
        self._set_angle(SERVO_DUTY_HOME)
        time.sleep(0.3)
        lgpio.tx_pwm(self._h, self._pin, SERVO_PWM_FREQ, 0)
        gpio_safe_free(self._h, self._pin)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# LED 제어 헬퍼 함수
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def set_leds(h, green, yellow, red):
    lgpio.gpio_write(h, LED_GREEN, green)
    lgpio.gpio_write(h, LED_YELLOW, yellow)
    lgpio.gpio_write(h, LED_RED, red)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Button swith - (ON/OFF only)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class ButtonMonitor:
    def __init__(self, h, pin, dc_motor, buzzer, debounce_ms=50):
        self._h = h
        self._pin = pin
        self._dc = dc_motor
        self._buzzer = buzzer
        self._debounce = debounce_ms / 1000.0
        self._stop_event = threading.Event()
        self._last_state = 1
        self._thread = threading.Thread(target=self._run, daemon=True)
        
        gpio_safe_free(self._h, self._pin)
        lgpio.gpio_claim_input(self._h, self._pin, lgpio.SET_PULL_UP)
        self._thread.start()

    def _run(self):
        while not self._stop_event.is_set():
            
            state = lgpio.gpio_read(self._h, self._pin)
            
            if self._last_state == 1 and state == 0:
                time.sleep(self._debounce)
                if lgpio.gpio_read(self._h, self._pin) == 0:
                    print("[BTN] Emergency STOP! -> RED LED")
                    self._dc.set_speed(MOTOR_OFF)

                    set_leds(self._h, 0, 0, 1)
                    self._buzzer.beep(0.1, 3)
            self._last_state = state
            time.sleep(0.02)

    def cleanup(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        gpio_safe_free(self._h, self._pin)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# IR Sensor Monitor - 물체 감지 시 2초 정지 후 재개
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class IRSensorMonitor:
    def __init__(self, h, pin, dc_motor, buzzer, ser, on_resume=None):
        self._h = h
        self._pin = pin
        self._dc = dc_motor
        self._buzzer = buzzer
        self._ser = ser
        self._on_resume = on_resume
        self._last_resume_time = time.time()
        self._stop_event = threading.Event()
        
        
        gpio_safe_free(self._h, self._pin)
        lgpio.gpio_claim_input(self._h, self._pin, lgpio.SET_PULL_UP)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        
        

    def _run(self):
        while not self._stop_event.is_set():
            if lgpio.gpio_read(self._h, self._pin) == 0:
                
                self._buzzer.beep(0.1, 1)
                
                time.sleep(0.05)
                
                current_speed = self._dc.get_speed()
                if current_speed != MOTOR_OFF:
                    self._dc.set_speed(MOTOR_OFF)
                    set_leds(self._h, 0, 1, 0)
                    
                    pkt = build_packet(CMD_STATUS_REPORT, 0, 0, 0xFF, MOTOR_OFF)
                    self._ser.write(pkt)
                    
                    time.sleep(2.0)
                    
                    self._dc.set_speed(current_speed)
                    self._last_resume_time = time.time()
                    if self._on_resume is not None:
                        self._on_resume(self._last_resume_time)
                    set_leds(self._h, 1, 0, 0)

                    time.sleep(3.0)

            time.sleep(0.1)

    def cleanup(self):
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        gpio_safe_free(self._h, self._pin)
        
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Buzzer Controller - 알림음 제어
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class BuzzerController:
    def __init__(self, h, pin):
        self._h = h
        self._pin = pin
        self._freq = 2500
        gpio_safe_claim_output(self._h, self._pin, 0)

    def beep(self, duration=0.05, count=1):
        """별도 스레드에서 부저를 울려 메인 루프 지연 방지"""
        threading.Thread(target=self._do_beep, args=(duration, count), daemon=True).start()

    def _do_beep(self, duration, count):
        for _ in range(count):
            # 50% 듀티 사이클로 주파수를 발생시켜 진동을 만듦
            lgpio.tx_pwm(self._h, self._pin, self._freq, 50) 
            time.sleep(duration)
            # PWM 정지 (소리 끄기)
            lgpio.tx_pwm(self._h, self._pin, self._freq, 0) 
            if count > 1:
                time.sleep(0.05)

    def cleanup(self):
        lgpio.tx_pwm(self._h, self._pin, self._freq, 0)
        gpio_safe_free(self._h, self._pin)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# DHT11 reader
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DHT11Reader:
    def __init__(self, h, gpio_num):
        self._h    = h
        self._gpio = gpio_num
        self._last_temp = 0.0
        self._last_humi = 0.0

    def _read_raw(self):
        g = self._gpio
        h = self._h
        edges = []
        def on_edge(chip, gpio, level, tick):
            edges.append((level, tick))
        gpio_safe_claim_output(h, g, 1)
        lgpio.gpio_write(h, g, 0)
        time.sleep(0.020)
        lgpio.gpio_write(h, g, 1)
        time.sleep(0.00002)
        gpio_safe_free(h, g)
        lgpio.gpio_claim_alert(h, g, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
        cb = lgpio.callback(h, g, lgpio.BOTH_EDGES, on_edge)
        time.sleep(0.1)
        cb.cancel()
        gpio_safe_free(h, g)
        return edges

    def _parse(self, edges):
        high_durations = []
        for i in range(len(edges) - 1):
            if edges[i][0] == 1 and edges[i + 1][0] == 0:
                dur = edges[i + 1][1] - edges[i][1]
                if dur < 0: dur += (1 << 64)
                high_durations.append(dur)
        if len(high_durations) < 38:
            return None, None
        data_pulses = high_durations[-40:] if len(high_durations) >= 40 else high_durations
        threshold = sum(data_pulses) / len(data_pulses)
        bits = [1 if dur > threshold else 0 for dur in data_pulses]
        while len(bits) < 40: bits.insert(0, 0)
        bits = bits[-40:]
        data = []
        byte = 0
        for i, b in enumerate(bits):
            byte = (byte << 1) | b
            if (i + 1) % 8 == 0:
                data.append(byte)
                byte = 0
        if len(data) != 5: return None, None
        if ((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4]:
            return None, None
        humi = data[0] + data[1] * 0.1
        temp = data[2] + (data[3] & 0x7F) * 0.1
        if data[3] & 0x80: temp = -temp
        return temp, humi

    def read(self):
        for attempt in range(5):
            try:
                edges = self._read_raw()
                if len(edges) < 2:
                    time.sleep(1.0); continue
                t, h = self._parse(edges)
                if t is not None and h is not None:
                    self._last_temp = t
                    self._last_humi = h
                    return self._last_temp, self._last_humi
                else:
                    print(f"[DHT11] Parse failed (edges={len(edges)}, attempt {attempt+1})")
            except Exception as e:
                print(f"[DHT11] Error: {e}")
            time.sleep(1.0)
        print("[DHT11] Read failed, using last value")
        return self._last_temp, self._last_humi

    def cleanup(self):
        gpio_safe_free(self._h, self._gpio)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Main
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def main():
    h = lgpio.gpiochip_open(0)
    

    # LED 핀 출력 설정 및 초기화 (전부 끄기)
    gpio_safe_claim_output(h, LED_GREEN, 0)
    gpio_safe_claim_output(h, LED_YELLOW, 0)
    gpio_safe_claim_output(h, LED_RED, 0)
    
    
    buzzer = BuzzerController(h, BUZZER_PIN)
    
    dht     = DHT11Reader(h, DHT_PIN)
    dc      = DCMotor(h, DC_PWM_PIN, DC_DIR_PIN)
    stepper = StepMotor(h, [STEP_IN1, STEP_IN2, STEP_IN3, STEP_IN4])
    servo   = ServoMotor(h, SERVO_PIN)
    btn = ButtonMonitor(h, BTN_PIN, dc, buzzer)
    
    
    
    

    reject_lock = threading.Lock()
    
    items = deque()
    pending_defect = False
    
    scheduler_stop = threading.Event()

    def on_ir_resume(resume_time):
        nonlocal pending_defect
        
        with reject_lock:
            defect = pending_defect
            pending_defect = False

            items.append({
                "target": time.monotonic() + SERVO_DELAY,
                "defect": defect
            })

        print(f"[QUEUE] item added, defect={defect}, count={len(items)}")
    
    def servo_scheduler():
        while not scheduler_stop.is_set():
            item = None
            now_m = time.monotonic()

            with reject_lock:
                if items and now_m >= items[0]["target"]:
                    if items[0]["defect"]:
                        item = items.popleft()
                    elif now_m >= items[0]["target"] + JUDGEMENT_GRACE:
                        print("[SERVO] normal item passed")
                        items.popleft()

            if item is not None:
                print("[SERVO] -> EXECUTE")
                servo.reject()

            time.sleep(0.005)

    scheduler_thread = threading.Thread(target=servo_scheduler, daemon=True)
    scheduler_thread.start()
    

    try:
        ser = serial.Serial(
            port=SERIAL_PORT, baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0.1
        )
        print(f"[SERIAL] {SERIAL_PORT} opened ({BAUD_RATE} bps)")
    except serial.SerialException as e:
        print(f"[ERROR] Serial: {e}")
        dht.cleanup(); dc.cleanup(); stepper.cleanup(); servo.cleanup()
        lgpio.gpiochip_close(h)
        return
    
    ir      = IRSensorMonitor(h, IR_PIN, dc, buzzer, ser, on_resume=on_ir_resume)

    reader = PacketReader(ser)


    fan_manual_on = False

    last_report = 0.0
    print("[SYSTEM] Started")
    print(f"[SYSTEM] DC Motor(conveyor): PWM=GPIO{DC_PWM_PIN}, DIR=GPIO{DC_DIR_PIN}")
    print(f"[SYSTEM] Stepper(fan): GPIO {STEP_IN1}/{STEP_IN2}/{STEP_IN3}/{STEP_IN4}")
    print(f"[SYSTEM] Servo(defect reject): GPIO{SERVO_PIN}")
    print(f"[SYSTEM] DC duty: Speed1={DC_SPEED_DUTY[1]}% Speed2={DC_SPEED_DUTY[2]}% Speed3={DC_SPEED_DUTY[3]}%")

    try:
        while True:
            now = time.time()
            time.sleep(0.05)
            
            if now - last_report >= 2.0:
                temp, humi = dht.read()
                cur_speed = dc.get_speed()
                
                
                if temp >= 60.0:
                    if not stepper.is_on():
                        print(f"[AUTO] ondo warring ({temp:.1f}C)! fan on")
                        stepper.set_on(True)
                else:
                    if not fan_manual_on and stepper.is_on():
                        print(f"[AUTO] ondo Recovery ({temp:.1f}C). fan stop")
                        stepper.set_on(False)

                

                temp_int = int(temp) & 0xFF
                humi_int = int(humi) & 0xFF
                temp_dec = int(round(temp * 10)) % 10
                humi_dec = int(round(humi * 10)) % 10
                ultr_packed = ((temp_dec & 0x0F) << 4) | (humi_dec & 0x0F)

                pkt = build_packet(CMD_STATUS_REPORT,
                                   temp_int, humi_int, ultr_packed, cur_speed)
                ser.write(pkt)
                fan_str = "ON" if stepper.is_on() else "OFF"
                print(f"[TX] Temp={temp:.1f}C Humi={humi:.1f}% "
                      f"DC={cur_speed}({dc.get_duty()}%) Fan={fan_str}")
                last_report = now

            # ── Read with STX sync ──
            raw = reader.read_packet()
            if raw is not None:
                cmd  = raw[IDX_CMD]
                data = raw[IDX_MOTR]
                print(f"[RX] CMD=0x{cmd:02X} DATA=0x{data:02X}")

                # ── Conveyor ON/OFF → DC motor ──
                if cmd == CMD_CONVEYOR_CONTROL:
                    if data == CTRL_ON:
                        dc.set_speed(MOTOR_SPEED_1)
                        set_leds(h, 1, 0, 0) # GREEN LED ON
                        print(f"[DC] Conveyor ON (duty={dc.get_duty()}%)")
                    else:
                        dc.set_speed(MOTOR_OFF)
                        set_leds(h, 0, 0, 0) # LED OFF
                        buzzer.beep(0.2, 1) # buzzer on
                        print("[DC] Conveyor OFF -> STOPPED")
                    send_ack(ser, True)

                # ── Speed 1/2/3 → DC motor duty ──
                elif cmd == CMD_SPEED_SET:
                    if data in (MOTOR_SPEED_1, MOTOR_SPEED_2, MOTOR_SPEED_3):
                        dc.set_speed(data)
                        set_leds(h, 1, 0, 0)
                        
                        print(f"[DC] Speed {data} (duty={dc.get_duty()}%)")
                        send_ack(ser, True)
                    else:
                        print(f"[DC] Invalid speed: 0x{data:02X}")
                        send_ack(ser, False)

                # ── Fan ON/OFF → Stepper motor ──
                elif cmd == CMD_FAN_CONTROL:
                    if data == CTRL_ON:
                        fan_manual_on = True
                        stepper.set_on(True)
                        print("[STEPPER] Fan ON by MFC")
                    else:
                        fan_manual_on = False

                        if temp >= 60.0:
                            stepper.set_on(True)
                            print("[STEPPER] Fan manual OFF, but AUTO temp high -> keep ON")
                        else:
                            stepper.set_on(False)
                            print("[STEPPER] Fan OFF by MFC")

                    send_ack(ser, True)

                # ── Emergency Stop → DC motor only ──
                elif cmd == CMD_EMG_STOP:
                    dc.set_speed(MOTOR_EMG)
                    set_leds(h, 0, 0, 1) # RED ON
                    buzzer.beep(0.5, 2) # buzzer on
                    print("[DC] *** EMERGENCY STOP *** -> STOPPED")
                    send_ack(ser, True)

                # ── Defect Servo → reject defect item ──
                elif cmd == CMD_DEFECT_SERVO:
                    print("[SERVO] *** DEFECT DETECTED *** -> Activating reject servo")
                    #buzzer.beep(0.1, 2)
                    #set_leds(h, 0, 1, 0)  # YELLOW LED during reject
                    
                    now_m = time.monotonic()
                    
                    with reject_lock:
                        if items:
                            items[0]["defect"] = True
                            print("[SERVO] defect marked to queued item")
                        else:
                            pending_defect = True
                            print("[SERVO] defect saved as pending")

                    send_ack(ser, True)

                else:
                    print(f"[RX] Unknown: 0x{cmd:02X}")
                    send_ack(ser, False)
                    
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Shutdown (Ctrl+C)")

    finally:
        dc.set_speed(MOTOR_OFF)
        stepper.set_on(False)
        
        scheduler_stop.set()
        scheduler_thread.join(timeout=1.0)
        
        btn.cleanup()
        ir.cleanup()
        buzzer.cleanup()

        # 그 다음 GPIO 사용하는 애들 정리
        dc.cleanup()
        stepper.cleanup()
        servo.cleanup()
        dht.cleanup()

        # 마지막에 GPIO 닫기
        lgpio.gpiochip_close(h)

        # 마지막 시리얼
        ser.close()
        print("[SYSTEM] Cleanup complete.")


if __name__ == "__main__":
    main()