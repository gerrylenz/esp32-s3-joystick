## ESP32 GRBL Joystick Controller man in the middle
Ein ESP32-S3 Projekt zur Steuerung eines GRBL-gesteuerten CNC-Systems über einen analogen Joystick. Das Projekt nutzt den ESP32-S3 USB Host, UART und ADC für die Kommunikation und Eingabe.
Das ganze wird als USB<->USB Bridge verwendet.

## Features
USB CDC-ACM Kommunikation mit GRBL-Geräten
Joystick-basierte Jogging-Steuerung
10-Stufen-Quantisierung für sanfte Bewegungen
Deadzone-Kalibrierung für den Joystick
Automatische GRBL-Konfigurationsabfrage ($$) nach USB-Verbindung
Fehlererkennung (ALARM, ERROR) und automatischer Stop
Unterstützung für LED-Anzeige über WS2812B-LEDs
Blau: Gerät verbunden
Rot: Gerät getrennt

## Hardware Requirements
ESP32-S3 Dev Board (z. B. ESP32-S2 devkit)
USB-Gerät kompatibel mit CDC-ACM (LPC17xx smoothiboard)
Analoger Joystick (X/Y-Achsen)
Optional: WS2812B LED-Strip

## Pin Configuration
Signal	ESP32-S3 Pin
UART0 TX	43
UART0 RX	44
UART1 TX	17
UART1 RX	18
Joystick X (ADC)	GPIO ADC2
Joystick Y (ADC)	GPIO ADC3

Hinweis: ADC-Kanäle werden auf 12 Bit aufgelöst.

## Software Requirements
ESP-IDF v5.x
FreeRTOS (inklusive ESP-IDF)
USB Host Stack für ESP32
GRBL-fähiges Gerät über USB

## Installation

ESP-IDF einrichten und Toolchain installieren:
ESP-IDF Getting Started Guide

## Projekt klonen:
git clone https://github.com/deinusername/esp32-grbl-joystick.git
cd esp32-grbl-joystick

## Projekt konfigurieren (optional):
idf.py menuconfig

## Flashen und Monitor starten:
idf.py flash monitor

## Usage
Joystick zentrieren und ESP32 starten.
USB-Gerät anschließen. Der ESP32 erkennt automatisch die Verbindung.
GRBL-Konfiguration wird automatisch abgerufen ($$) und geparst.
Joystick-Bewegungen werden in Echtzeit in Jogging-Befehle ($J=G91 X.. Y.. F..) umgewandelt.
Fehler wie ALARM oder ERROR werden erkannt und stoppen sofort alle Bewegungen.
LED zeigt den Gerätestatus:
Blau: verbunden
Rot: getrennt

## Code Structure
app_main(): Initialisierung von UART, ADC, LED und Tasks
usb_connect_loop(): USB-Verbindungsmanagement
usb_host_task(): USB Host Event Handler
process_usb_rx_task(): Empfang und Verarbeitung von GRBL-Daten
queue2grbl(): Sendet Befehle aus der Queue an GRBL
joy_task(): Liest Joystick und erzeugt Jogging-Befehle
adc_init() / adc_read(): ADC-Konfiguration und Lesezyklen
grbl_parse_config(): Parsen der GRBL-Konfigurationszeilen
led_control.hpp: Steuerung des LED-Strips

## Notes
Joystick Deadzone wird automatisch auf Basis der Mittelstellung ermittelt.
Maximalgeschwindigkeit für Jogging wird aus der GRBL-Konfiguration gelesen.
USB CDC-ACM wird über Blocksendung (data_tx_blocking) verwendet, um Datenverlust zu vermeiden.

License
free to use @ your own risk
