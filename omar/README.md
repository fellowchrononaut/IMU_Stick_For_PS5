# README

### Objective

To develop a script and the hardware necessary to test the link between a microcontroller and the PS5 expansion port.

### Parts List

- [Knockoff Waveshare RP2040-Zero](https://fr.aliexpress.com/item/1005007650325892.html?spm=a2g0o.order_list.order_list_main.52.76645e5bInMVJV&gatewayAdapt=glo2fra)
- [MCP4251-103E/P 8bit, dual SPI digital potentiometer with volatile memory](https://fr.farnell.com/en-FR/microchip/mcp4251-103e-p/ic-dpot-5-5v-10kr-14-pdip-spi/dp/1578442)
- [3.5mm Male To 3.5 mm Male TRRS Aux Cable](https://fr.aliexpress.com/item/1005009181119614.html?spm=a2g0o.order_list.order_list_main.5.76645e5bInMVJV&gatewayAdapt=glo2fra)
- [3.5mm Female Jack to Bare Wire Open End TRRS](https://fr.aliexpress.com/item/1005008510930978.html?spm=a2g0o.order_list.order_list_main.11.76645e5bInMVJV&gatewayAdapt=glo2fra)
- [USB-A to USB-C Cable](https://fr.aliexpress.com/item/1005008494569058.html?spm=a2g0o.order_list.order_list_main.62.76645e5bInMVJV&gatewayAdapt=glo2fra)
- [Starter Kit For Arduino](https://fr.aliexpress.com/item/1005006480276775.html?spm=a2g0o.order_list.order_list_main.177.76645e5bInMVJV&gatewayAdapt=glo2fra)

### Useful Resources

- [Waveshare RP2040-Zero Pinout](https://www.waveshare.com/rp2040-zero.htm)
- [How to configure Waveshare RP2040-Zero in Arduino IDE](https://www.waveshare.com/wiki/RP2040-Zero)
- [How to create a joystick input for PS5 access controller](https://www.youtube.com/watch?v=6_bBL7czNOw)
- [Digital potentiometers and Arduino](https://www.youtube.com/watch?v=AqeGskH0usY&t=781s)
- [Waveshare RP2040-Zero Fritzing component](https://forum.fritzing.org/t/part-request-waveshare-rp2040-zero/16705)
- [How to download Fritzing for free](https://gist.github.com/RyanLua/fc2457d87641bb39754278b01a647526)
- [SPI wikipedia overview](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#See_also)
- Access™ Controller for PlayStation®5 Expansion Port Specifications PDF
- Microchip MCP413X/415X/423X/425X Datasheet PDF

### Wiring

### MCP4251 Pin Allocation

| MCP4251 Pin | Label | Connection Point | Description |
| :--- | :--- | :--- | :--- |
| **1** | CS | **GP5 (SPI CS)** | SPI Chip Select |
| **2** | SCK | **GP2 (SPI SCK)** | SPI Clock |
| **3** | SDI | **GP3 (SPI TX)** | SPI Data Input |
| **4** | Vss | **GND** | Common Ground |
| **5** | P1B | **GND** | Pot 1 Low Side (GND) |
| **6** | P1W | **TIP (Y-AXIS) + GP26 (ADC)** | Pot 1 Wiper |
| **7** | P1A | **Sleeve (1.8V Ref)** | Pot 1 High Side |
| **8** | P0A | **Sleeve (1.8V Ref)** | Pot 0 High Side |
| **9** | P0W | **RING 1 (X-AXIS) + GP27 (ADC)** | Pot 0 Wiper |
| **10** | P0B | **GND** | Pot 0 Low Side (GND) |
| **11** | WP | **3.3V** | Write Protect (Disabled) |
| **12** | SHDN | **3.3V** | Shutdown (Disabled) |
| **13** | SDO | N/A | Not Used |
| **14** | Vdd | **3.3V** | Chip Logic Power |

![Wiring](https://github.com/fellowchrononaut/IMU_Stick_For_PS5/blob/main/omar/breadboard_wiring.png)

### Code Walkthrough

This program allows the user to send X/Y joystick commands to the PS5 Access Controller by modifying the wiper positions of the corresponding potentiometers via the serial port.

```cpp
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
```

### Test Procedure

1. Turn ON PS5 Access Controller
2. Connect 3.5 mm Male jack to PS5 Access Controller expansion port
3. Verify voltage drop between the sleeve and ring 2 is 1.8V
4. Turn OFF PS5 Access Controller
5. Wire 3.5 mm jack to breadboard according to wiring diagram
6. Turn ON microcontroller and PS5 Access Controller
7. Send X/Y joystick commands from Arduino IDE serial port to PS5 Access Controller


