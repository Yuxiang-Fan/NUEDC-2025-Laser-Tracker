# main.py
import sensor, image, time
from pyb import Servo, LED
from machine import Pin
from pid import PID  # 导入刚才分离出来的PID类

# ================= 1. 配置区 (Configuration) =================
# 视觉识别阈值配置
LAB_THRESHOLD = [(33, 0, -128, 127, -128, 127)]  # 黑框检测 LAB 阈值
# 目标尺寸约束
MIN_L = 15
MAX_L = 250
MIN_W = 5
MAX_W = 200

# 硬件引脚配置
PAN_SERVO_PIN = 1  # 水平舵机，P7引脚
TILT_SERVO_PIN = 2  # 垂直舵机，P8引脚
LASER_PIN_NAME = 'P6'  # 激光笔引脚

# PID 参数配置
PAN_P = 0.07
TILT_P = 0.08
PID_IMAX = 90

# 控制参数
BUFFER_SIZE = 5  # 平均前5帧误差平滑


# ==========================================================

# 寻找最大目标函数
def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob[2] * blob[3] > max_size:
            max_blob = blob
            max_size = blob[2] * blob[3]
    return max_blob


# ================= 2. 硬件与传感器初始化 =================
# 点亮红色LED指示状态
LED(1).on()

# 舵机初始化（恢复参考校准）
pan_servo = Servo(PAN_SERVO_PIN)
tilt_servo = Servo(TILT_SERVO_PIN)
pan_servo.calibration(500, 2500, 500)
tilt_servo.calibration(500, 2500, 500)

# 设置初始角度为90度（中位，强制居中）
pan_servo.angle(90)
tilt_servo.angle(90)

# PID控制器初始化
pan_pid = PID(p=PAN_P, i=0, imax=PID_IMAX)
tilt_pid = PID(p=TILT_P, i=0, imax=PID_IMAX)

# 激光笔控制引脚
laser_pin = Pin(LASER_PIN_NAME, Pin.OUT)

# 摄像头初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(10)
sensor.set_auto_whitebal(False)

clock = time.clock()
time.start_time = time.ticks_ms()

# 误差平滑缓冲区
pan_error_buffer = []
tilt_error_buffer = []

# ================= 3. 主循环 =================
while True:
    clock.tick()
    img = sensor.snapshot()

    # 寻找黑框目标
    blobs = img.find_blobs(LAB_THRESHOLD, pixels_threshold=10, area_threshold=20, merge=True)
    valid_blobs = []

    for b in blobs:
        x, y, w, h = b.rect()
        fill_ratio = b.pixels() / (w * h) if w * h > 0 else 0
        L = max(w, h)
        W = min(w, h)
        aspect_ratio = L / W if W > 0 else float('inf')

        # 筛选有效目标
        if (L >= MIN_L and L <= MAX_L and W >= MIN_W and W <= MAX_W and
                aspect_ratio < 5.0 and 0.01 < fill_ratio < 0.4):
            valid_blobs.append(b)
            img.draw_rectangle(b.rect(), color=(255, 255, 255))

    if valid_blobs:
        max_blob = find_max(valid_blobs)
        # 反转偏差校正
        pan_error = (max_blob.cx() - img.width() / 2) + 8
        tilt_error = (max_blob.cy() - img.height() / 2) - 14

        # 误差平滑更新
        pan_error_buffer.append(pan_error)
        tilt_error_buffer.append(tilt_error)
        if len(pan_error_buffer) > BUFFER_SIZE:
            pan_error_buffer.pop(0)
        if len(tilt_error_buffer) > BUFFER_SIZE:
            tilt_error_buffer.pop(0)

        smoothed_pan_error = sum(pan_error_buffer) / len(pan_error_buffer)
        smoothed_tilt_error = sum(tilt_error_buffer) / len(tilt_error_buffer)

        # 激光笔控制逻辑（两秒强制开启，稳定后两秒开启）
        if not hasattr(time, 'start_time'):
            time.start_time = time.ticks_ms()

        if time.ticks_diff(time.ticks_ms(), time.start_time) >= 2000:
            laser_pin.on()
        elif abs(smoothed_pan_error) <= 10 and abs(smoothed_tilt_error) <= 10:
            if not hasattr(time, 'laser_start_time') or time.ticks_diff(time.ticks_ms(), time.laser_start_time) >= 2000:
                laser_pin.on()
                if not hasattr(time, 'laser_start_time'):
                    time.laser_start_time = time.ticks_ms()
        else:
            laser_pin.off()
            if hasattr(time, 'laser_start_time'):
                del time.laser_start_time

                # 满足条件时绘制十字准星
        if max_blob.w() > 10 and max_blob.h() > 10:
            img.draw_cross(max_blob.cx(), max_blob.cy(), color=(127, 127, 127))

            # 计算PID输出
        pan_output = pan_pid.get_pid(smoothed_pan_error, 1) * 0.9
        tilt_output = tilt_pid.get_pid(smoothed_tilt_error, 1) * 0.9

        # 更新舵机角度（保持方向和±5°步长）
        current_pan = pan_servo.angle()
        pan_change = min(5, max(-5, pan_output))
        new_pan = max(0, min(180, current_pan + pan_change))
        pan_servo.angle(new_pan)

        current_tilt = tilt_servo.angle()
        tilt_change = min(5, max(-5, -tilt_output))
        new_tilt = max(45, min(135, current_tilt + tilt_change))
        tilt_servo.angle(new_tilt)

    else:
        laser_pin.off()
        if hasattr(time, 'laser_start_time'):
            del time.laser_start_time

        pan_pid.reset_I()
        tilt_pid.reset_I()

        current_pan = pan_servo.angle()
        current_tilt = tilt_servo.angle()

        # 目标丢失时的舵机复位逻辑
        if time.ticks_diff(time.ticks_ms(), time.start_time) < 2000:  # 2秒内快速调整
            if abs(current_pan - 90) > 5:
                pan_servo.angle(90)
            if abs(current_tilt - 90) > 5:
                tilt_servo.angle(90)
        else:
            if current_pan < 90:
                pan_servo.angle(min(180, current_pan + 5))
            elif current_pan > 90:
                pan_servo.angle(max(0, current_pan - 5))

            if current_tilt < 90:
                tilt_servo.angle(max(45, min(135, current_tilt + 5)))
            elif current_tilt > 90:
                tilt_servo.angle(max(45, min(135, current_tilt - 5)))

    time.sleep(0.01)  # 加速响应