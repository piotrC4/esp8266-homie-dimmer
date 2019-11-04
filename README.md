
ESP8266 MQTT Homie(https://github.com/homieiot/homie-esp8266) based LED dimmer with light sensor (photo-resistor - build in into Witty module) 
Features:
- dimming   - `homie/_device_ID_/dimmer/percentage/set` (0..100%)
- dimming   - `homie/_device_ID_/dimmer/absolute/set` (0..1000)
- switching - `homie/_device_ID_/dimmer/switch/set` (ON-dim 100%, OFF- dim 0%)
- timer     - `homie/_device_ID_/dimmer/timer/set` (time [s] of 100% dim)
- Manual ON/OFF by momentary turning off/on power
 
USED GPIO (ESP1):
- GPIO-3 - RX Line - boot delay detector
- GPIO-2 - Dimmer output (MOSFET)
- GPIO-0 - Homie reset input (pullup + switch to ground)

USED GPIO (WEMOS-ESP12)
- GPIO-14 - bootdelay detector
- GPIO-12 - dimmer output (MOSFET)
- GPIO-0  - Homie reset input (pullup + switch to ground)
 
RX Line circuit (momentary reboot detector)
```
               |\ |    _____200k     _____2k
    +3,3V _____| >|---|____|---*----|____|-----*--------->GPIO3 (ESP1)
               |/ |            |               |          GPIO14 (WEMOS)
                             -----47uF        |--| 200k
                             -----            |__|
                              _|_             _|_
 ```
