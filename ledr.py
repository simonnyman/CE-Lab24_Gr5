import time
import RPi.GPIO as GPIO

GPIO.setwarnings(False)    # Ignore warning for now
GPIO.setmode(GPIO.BOARD)   # Use physical pin numbering
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)   # Set pin 18 to be an output pin and set initial value to low (off)

def collision(): # Run forever
    GPIO.output(18, GPIO.HIGH) # Turn on
    time.sleep(3)                  # Sleep for 1 second
    GPIO.output(18, GPIO.LOW)  # Turn off