# Karoake-Machine-Raspberry-pi-project-Ongoing 
Goals for the project:
  
  Build a device that allows the user to select a song on the phone and automatically play the song on the speakers in the device
  Automatically browse and generate lyrics for the respective song on the device LCD screen
  Allow volume control of the song using hand motion
  Allow control of the treble and bass of the song being played

Completed:

  Built the required circuitry for the device on a breadboard connecting the following additional components:
        - Potentiometers
        - PCF 8591 module (8 bit)
        - 74H595 8 bit shift register
        - buttons and LEDs (LED bar graph)
        - speakers
        - LCD screen
        - DC Servo Motor (to rotate LEDs)
        - Ultrasonic Distance sensor, Thermistor sensor       
  Set up and programmed the Raspberry pi I2C bus to communicate with multiple devices (PCF8591 and PCF8574).
  Set up the required functions for each component added to the electrical circuitry.
  Set up several GPIO pins to output PWM voltage to provide a varying voltage signal easily to some devices.     

To be completed:

  Write code for the main sequence of tasks (main) using multiple threads and Asynchronous programming.
  
