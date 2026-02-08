#include <SPI.h>

const int CS_PIN = 5;
const byte POT_LEFT  = 0x10; // Wiper 1 (Left)
const byte POT_RIGHT = 0x00; // Wiper 0 (Right)

const int ADC_LEFT  = 26;    // Monitor for wiper 1 (left)
const int ADC_RIGHT = 27;    // Monitor for wiper 2 (right)

void setup() {

  Serial.begin(115200);
  
  // Specify which SPI pins used on RP2040
  SPI.setSCK(2);
  SPI.setTX(3);
  SPI.setRX(4);
  SPI.begin();

  // Set chip select pin to high at the start
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  // 12 bit ADC on rp2040
  analogReadResolution(12);

  // Blahblah in serial port
  Serial.println("--- Dual-Channel PS5 Controller Tester ---");
  Serial.println("Commands: 'L' or 'R' followed by 0-255 (e.g., L128)");
  Serial.println("Type 'C' to Center both pots.");
  Serial.println("------------------------------------------");
}

void setWiper(byte address, int value) {

  // Failsafe to restrict wiper value commands
  value = constrain(value, 0, 255);

  // Send wiper value to digipot 
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address);
  SPI.transfer((byte)value);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

void displayVoltages() {

  // Read and store measured ADC values as voltages
  float vLeft = (analogRead(ADC_LEFT) * 3.3) / 4095.0;
  float vRight = (analogRead(ADC_RIGHT) * 3.3) / 4095.0;
  
  // Print measured voltages
  Serial.print(">> VOLTAGES | Left (GP26): ");
  Serial.print(vLeft, 3);
  Serial.print("V | Right (GP27): ");
  Serial.print(vRight, 3);
  Serial.println("V");
}

void loop() {

  // If buffer is not emtpy, interpret commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // If C/c, center both wipers
    if (cmd == 'C' || cmd == 'c') {
      Serial.println("Centering both sticks...");
      setWiper(POT_LEFT, 128);
      setWiper(POT_RIGHT, 128);
    } 
    else if (cmd == 'L' || cmd == 'l' || cmd == 'R' || cmd == 'r') {
      int val = Serial.parseInt();
      byte addr = (cmd == 'L' || cmd == 'l') ? POT_LEFT : POT_RIGHT;
      setWiper(addr, val);
      Serial.print("Set "); Serial.print(cmd); Serial.print(" to "); Serial.println(val);
    }

    // Clear buffer
    while(Serial.available() > 0) Serial.read(); 
    
    // Show results of the move
    delay(10); // Small delay for ADC stabilization
    displayVoltages();
  }
}