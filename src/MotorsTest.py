import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

PWMPin_A1 = 11
PWMPin_A2 = 13
DIRPin_B1 = 12
DIRPin_B2 = 15


GPIO.setup(PWMPin_A1, GPIO.OUT)
GPIO.setup(DIRPin_B1, GPIO.OUT)
GPIO.setup(PWMPin_A2, GPIO.OUT)
GPIO.setup(DIRPin_B2, GPIO.OUT)

Motor1 = GPIO.PWM(PWMPin_A1,1000)
Motor2 = GPIO.PWM(PWMPin_A2,1000)

def controlSignal2PWM(Motor,signal,DIRPin):
	Motor.ChangeDutyCycle(signal)
        if signal > 0:
		GPIO.output(DIRPin, GPIO.HIGH)
                sleep(0.01)
                print "Signal is Positive, Motor is CW"
        elif signal < 0:
		GPIO.output(DIRPin, GPIO.LOW)		
                sleep(0.01)
                print "Signal is Negative, Motor is CCW"
        else:
                print "Signal is Zero"


def testMotors():

	try:
		Motor1.start(1)
		Motor2.start(2)
		while 1:
			for pwm in range(0,100):
				controlSignal2PWM(Motor1,pwm,DIRPin_B1)
				controlSignal2PWM(Motor2,pwm,DIRPin_B2)
				sleep(0.2)
			for pwm2 in range(100,0):
				controlSignal2PWM(Motor1,100-pwm2,DIRPin_B1)
				controlSignal2PWM(Motor2,100-pwm2,DIRPin_B2)
				sleep(0.2)
	except KeyboardInterrupt:
		pass

	Motor1.stop()
	Motor2.stop()
	GPIO.cleanup()

if __name__ == '__main__':
	testMotors()
