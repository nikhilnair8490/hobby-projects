# Import Raspberry Pi I/O library
import RPi.GPIO as GPIO

# Import Python libraries
import time

# GPIO Mode (Board-> use physical numbers on board, BCM-> Broadcom numbering)
GPIO.setmode(GPIO.BOARD)

# Set GPIO pins to drive the LED
GPIO_LED = 11  # Pin# 11 on the board

# Configure GPIO pin direction (Input or Output)
GPIO.setup(GPIO_LED, GPIO.OUT)

ON = True
OFF = False

# LED Functions

def LED_ON():
    """Function to to switch ON the LED"""
    GPIO.output(GPIO_LED, ON)


def LED_OFF():
    """Function to to switch OFF the LED"""
    GPIO.output(GPIO_LED, OFF)


def GPIOCleanup():
    """Run GPIO clean up procedure to release pins for further use"""
    GPIO.cleanup()


# This section will only be executed if run directly as a script
if __name__ == "__main__":

    try:
        print("Program Started")
        while True:
            LED_ON()
            time.sleep(0.25)
            LED_OFF()
            time.sleep(0.25)

    except KeyboardInterrupt:  # Handling for CTRL + C interrupt
        print("Program Terminated by User")
        GPIO.cleanup()  # Important to exit with GPIO clean up to release pins for further use
