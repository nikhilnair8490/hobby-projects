# Import Raspberry Pi I/O library
import RPi.GPIO as GPIO

# Import Python libraries
import time

# GPIO Mode (Board-> use physical numbers on board, BCM-> Broadcom numbering)
GPIO.setmode(GPIO.BOARD)

# Set GPIO pins for Ultrasonic sensor (HC-SR04) pins Trig and Echo
GPIO_TRIGGER = 22
GPIO_ECHO = 36

# Configure GPIO pin direction (Input or Output)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

ON = True
HIGH = True
OFF = False
LOW = False

# Ultrasonic Sensor distance calculation

def distofObj():
    """Function to calculate distance of object from ultrasonic sensor"""

    # Set sensor Trig to HIGH
    GPIO.output(GPIO_TRIGGER, HIGH)

    # Set Trig to LOW after 0.01 ms (Per Sensor spec this has to be 10 us or 0.01ms)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, LOW)

    # Intialisation of timers
    startTime = time.time()
    stopTime = time.time()

    # Store the time when ECHO is LOW (Transition from LOW to HIGH)
    while GPIO.input(GPIO_ECHO) == 0:
        startTime = time.time()

    # Store the time when ECHO is HIGH (Transition from HIGH to LOW)
    while GPIO.input(GPIO_ECHO) == 1:
        stopTime = time.time()

    # Time duration between start and stop
    timeDiff = stopTime - startTime
    # Multiply with the speed of sound (34300 cm/s)
    # and divide by 2, because the wave travels from sensor to object and back
    dist = (timeDiff * 34300) / 2

    return dist

def GPIOCleanup():
    """Run GPIO clean up procedure to release pins for further use"""
    GPIO.cleanup()

# This section will only be executed if run directly as a script
if __name__ == "__main__":

    try:
        print("Program Started")
        while True:
            distance = distofObj()
            print("The measured object distance is = %.2f cm" % distance)
            time.sleep(0.5)

    except KeyboardInterrupt:  # Handling for CTRL + C interrupt
        print("Program Terminated by User")
        GPIO.cleanup()  # Important to exit with GPIO clean up to release pins for further use
