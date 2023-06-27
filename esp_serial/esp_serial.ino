const int outputPin1 = 25; // assign output pin 25 to a variable
const int outputPin2 = 26; // assign output pin 26 to a variable

void setup() {
  dacWrite(outputPin1, 0); // initialize output on pin 25 to 0V
  dacWrite(outputPin2, 0); // initialize output on pin 26 to 0V
  Serial.begin(9600); // initialize serial communication at 9600 bits per second
}

void loop() {
  if (Serial.available() > 0) { // check if there is any incoming data from the serial port
    String inputString = Serial.readStringUntil('\n'); // read the incoming data until a newline character is received
    inputString.trim(); // remove any whitespace characters from the beginning and end of the string
    if (inputString.length() > 0) { // check if the input string is not empty
      int commaIndex = inputString.indexOf(','); // find the index of the comma in the input string
      if (commaIndex > 0 && commaIndex < inputString.length() - 1) { // check if the comma is in a valid position
        String voltageString1 = inputString.substring(0, commaIndex); // extract the first voltage value from the input string
        String voltageString2 = inputString.substring(commaIndex + 1); // extract the second voltage value from the input string
        voltageString1.replace(",", "."); // replace comma with period in voltageString1
        voltageString2.replace(",", "."); // replace comma with period in voltageString2
        float voltage1 = voltageString1.toFloat(); // convert the first voltage value to a float
        float voltage2 = voltageString2.toFloat(); // convert the second voltage value to a float
        voltage1 = map(voltage1, 0, 255, 0, 3.3); // map the first voltage value from 0-255 range to 0-3.3 volts
        voltage2 = map(voltage2, 0, 255, 0, 3.3); // map the second voltage value from 0-255 range to 0-3.3 volts
        dacWrite(outputPin1, voltage1); // output the first voltage on pin 25
        dacWrite(outputPin2, voltage2); // output the second voltage on pin 26
        Serial.print("Voltage 1: "); // print the first voltage value to the serial monitor
        Serial.println(voltageString2);
        Serial.print("Voltage 2: "); // print the second voltage value to the serial monitor
        Serial.println(voltageString2);
      }
    }
  }
}
