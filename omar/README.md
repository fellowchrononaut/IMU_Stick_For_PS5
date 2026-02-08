# README

## Objective

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
- Access™ Controller for PlayStation®5 Expansion Port Specifications PDF
- Microchip MCP413X/415X/423X/425X Datasheet PDF

### Wiring
![Wiring](https://github.com/fellowchrononaut/IMU_Stick_For_PS5/blob/main/omar/breadboard_wiring.png)

### Code Walkthrough

This program allows the user to send X/Y joystick commands to the PS5 Access Controller by modifying the wiper positions of the corresponding potentiometers via the serial port.



### Test Procedure

1. Turn ON PS5 Access Controller
2. Connect 3.5 mm Male jack to PS5 Access Controller expansion port
3. Verify voltage drop between the sleeve and ring 2 is 1.8V
4. Turn OFF PS5 Access Controller
5. Wire 3.5 mm jack to breadboard
6. Turn ON microcontroller and PS5 Access Controller
7. Send X/Y joystick commands from Arduino IDE serial port to PS5 Access Controller


