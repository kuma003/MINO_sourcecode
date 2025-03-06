import RPi.GPIO as GPIO
import pygage
import time
from pygame.locals import *

# pygame setup
pygame.init()
screen = pygame.display.set_mode((400, 330))
pygame.display.set_caption("keyboard event")

# GPIO setup
GPIO.setmode(GPIO.BCM)
left_motor_forward = 19
left_motor_backward = 13
right_motor_forward = 5
right_motor_backward = 6

GPIO.setup(left_motor_forward, GPIO.OUT)
GPIO.setup(left_motor_backward, GPIO.OUT)
GPIO.setup(right_motor_forward, GPIO.OUT)
GPIO.setup(right_motor_backward, GPIO.OUT)

def stop_motors():
    GPIO.output(left_motor_forward, 0)
    GPIO.output(left_motor_backward, 0)
    GPIO.output(right_motor_forward, 0)
    GPIO.output(right_motor_backward, 0)

def move_forward(speed):
    pwm_left_forward.ChangeDutyCycle(speed)
    pwm_right_forward.ChangeDutyCycle(speed*0.5)
    pwm_left_backward.ChangeDutyCycle(0)
    pwm_right_backward.ChangeDutyCycle(0)

def move_backward(speed):
    pwm_left_forward.ChangeDutyCycle(0)
    pwm_right_forward.ChangeDutyCycle(0)
    pwm_left_backward.ChangeDutyCycle(speed)
    pwm_right_backward.ChangeDutyCycle(speed*0.5)

def turn_left(speed):
    pwm_left_forward.ChangeDutyCycle(0)
    pwm_right_forward.ChangeDutyCycle(speed*0.5)
    pwm_left_backward.ChangeDutyCycle(speed)
    pwm_right_backward.ChangeDutyCycle(0)

def turn_right(speed):
    pwm_left_forward.ChangeDutyCycle(speed)
    pwm_right_forward.ChangeDutyCycle(0)
    pwm_left_backward.ChangeDutyCycle(0)
    pwm_right_backward.ChangeDutyCycle(speed*0.5)

try:
    pwm_left_forward = GPIO.PWM(left_motor_forward, 100)
    pwm_left_backward = GPIO.PWM(left_motor_backward, 100)
    pwm_right_forward = GPIO.PWM(right_motor_forward, 100)
    pwm_right_backward = GPIO.PWM(right_motor_backward, 100)

    pwm_left_forward.start(0)
    pwm_left_backward.start(0)
    pwm_right_forward.start(0)
    pwm_right_backward.start(0)

    speed = 0
    while(True):
        for event in pygame.event.get():
            if event.type == KEYDOWN:  # キーを押したとき
                    
                if  event.key == 'q':
                    break
                elif event.key == K_w: # 前進
                    print("前進")
                    while speed < 100:
                        move_forward(speed)
                        speed += 20
                elif event.key == K_s: # 後退
                    print("後退")
                    while speed < 100:
                        move_backward(speed)
                        speed += 20
                elif event.key == K_d: # 右
                    print("右")
                    while speed < 100:
                        turn_left(speed)
                        speed += 20
                elif event.key ==  K_a: # 左
                    print("左")
                    while speed < 100:
                        turn_right(speed)
                        speed += 20
            else:
                stop_motors()
except Exception as e:
    print(f"An error occurred: {e}")

finally:
    GPIO.cleanup()
