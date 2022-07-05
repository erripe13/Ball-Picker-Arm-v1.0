import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
buttonPin = 36
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setup(buttonPin, GPIO.IN)
while True: 
    if GPIO.input(buttonPin) == GPIO.HIGH:
        print("debugmode")
    else :
        print("normalmode")
