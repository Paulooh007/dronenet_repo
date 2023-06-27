void setup() {
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  if (Serial.available() > 0) { // Check if there is any data in the serial buffer
    char incomingData = Serial.read(); // Read the incoming data
    Serial.print(incomingData); // Print the incoming data to the Serial Monitor
  }
}
