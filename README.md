# Data Logger (and using cool sensors!)

*A lab report by Natalie B. Abrams.*

## In The Report

Include your responses to the bold questions on your own fork of [this lab report template](https://github.com/FAR-Lab/IDD-Fa18-Lab2). Include snippets of code that explain what you did. Deliverables are due next Tuesday. Post your lab reports as README.md pages on your GitHub, and post a link to that on your main class hub page.

For this lab, we will be experimenting with a variety of sensors, sending the data to the Arduino serial monitor, writing data to the EEPROM of the Arduino, and then playing the data back.

## Part A.  Writing to the Serial Monitor
 
**a. Based on the readings from the serial monitor, what is the range of the analog values being read?**
 
 The range is 0 to 1023
 
**b. How many bits of resolution does the analog to digital converter (ADC) on the Arduino have?**

[RGB color spectrum](./part_b.MOV)

10 bits  (2^10 is 1024). seems like all are being used

## Part B. RGB LED

**How might you use this with only the parts in your kit? Show us your solution.**

By using the Ohm resistors we have in our kit. One for each the blue and green, and two in a row for the red. 

## Part C. Voltage Varying Sensors 
 
### 1. FSR, Flex Sensor, Photo cell, Softpot

**a. What voltage values do you see from your force sensor?**

0-1023 (5v)

**b. What kind of relationship does the voltage have as a function of the force applied? (e.g., linear?)**

FSR- seems to be logorithmic. you push lightly and voltage goes up quickly, but you can squeeze really hard
and the voltage levels off. 

Flex Sensor-Linear. as i tilt it with the same consistent force it goes up and down evenly. 

Photo cell-Linear

Softpot-Logorithmic. Very durastic ups and downs.


**c. Can you change the LED fading code values so that you get the full range of output voltages from the LED when using your FSR?**

Yes

```
int fsrAnalogPin = 0; // FSR is connected to analog 0
int redLEDpin = 8;      // connect Red LED to pin 11 (PWM pin)
int blueLEDpin = 10;      // connect Red LED to pin 11 (PWM pin)
int greenLEDpin = 9;      // connect Red LED to pin 11 (PWM pin)
int fsrReading;      // the analog reading from the FSR resistor divider
int redLEDbrightness;
int greenLEDbrightness;
int blueLEDbrightness;
 
void setup(void) {
  Serial.begin(9600);   // We'll send debugging information via the Serial monitor
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  pinMode(blueLEDpin, OUTPUT);
}
 
void loop(void) {
  fsrReading = analogRead(fsrAnalogPin);
  Serial.print("Analog reading = ");
  Serial.println(fsrReading);
 
  // we'll need to change the range from the analog reading (0-1023) down to the range
  // used by analogWrite (0-255) with map!
  redLEDbrightness = map(100,0, 990, 0, fsrReading%100);
  greenLEDbrightness = map(100, 0, 990, fsrReading%100, 255);
  blueLEDbrightness = map(100,fsrReading%100, 990, 0, 255);
  // LED gets brighter the harder you press
  analogWrite(redLEDpin, redLEDbrightness);
  analogWrite(greenLEDpin, greenLEDbrightness);
  analogWrite(blueLEDpin, blueLEDbrightness);
  delay(100);
}

```


**d. What resistance do you need to have in series to get a reasonable range of voltages from each sensor?**


FSR-

Flex Sensor-

Photo Cell-

Softpot-


**e. What kind of relationship does the resistance have as a function of stimulus? (e.g., linear?)**

### 2. Accelerometer
 
**a. Include your accelerometer read-out code in your write-up.**

### 3. IR Proximity Sensor

**a. Describe the voltage change over the sensing range of the sensor. A sketch of voltage vs. distance would work also. Does it match up with what you expect from the datasheet?**

**b. Upload your merged code to your lab report repository and link to it here.**

## Optional. Graphic Display

**Take a picture of your screen working insert it here!**

## Part D. Logging values to the EEPROM and reading them back
 
### 1. Reading and writing values to the Arduino EEPROM

**a. Does it matter what actions are assigned to which state? Why?**

**b. Why is the code here all in the setup() functions and not in the loop() functions?**

**c. How many byte-sized data samples can you store on the Atmega328?**

**d. How would you get analog data from the Arduino analog pins to be byte-sized? How about analog data from the I2C devices?**

**e. Alternately, how would we store the data if it were bigger than a byte? (hint: take a look at the [EEPROMPut](https://www.arduino.cc/en/Reference/EEPROMPut) example)**

**Upload your modified code that takes in analog values from your sensors and prints them back out to the Arduino Serial Monitor.**

### 2. Design your logger
 
**a. Insert here a copy of your final state diagram.**

### 3. Create your data logger!
 
**a. Record and upload a short demo video of your logger in action.**
