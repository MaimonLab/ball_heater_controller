# Ball Heater Controller Assembly

The parts needed to build the ball heater are listed with links and part numbers in the first sheet of the [Bill of Materials (BOM)](../ball_heater_controller_pcb/ball_heater_controller_bom_V%201.ods) spreadsheet.

## PCB

PCBs can be ordered from one of many PCB supply houses, eg [pcbway](pcbway.com) or [jlcpcb](https://jlcpcb.com), using the [zipped gerber files](../ball_heater_controller_pcb/ball_heater_controller_v1.zip). Order a stencil as well to make the surface mount assembly easier and faster.

Once PCBs and parts have been acquired, apply solder paste using the stencil, and then use the [BOM printout](../ball_heater_controller_pcb/ball_heater_BOM_printout.pdf) to place surface mount parts in the correct locations matching their reference designators.  Then reflow the solder paste using a reflow over or hot air rework system. Lastly, solder the through hole components.

## Enclosure

Laser cut the two enclosure parts files out of 1/8 inch and 1/4 inch acrylic for [ball_heater_box_eighth](../ball_heater_controller_enclosure/tall_enclosure/ball_heater_box_eighth.ai) and [ball_heater_box_quarter](../ball_heater_controller_enclosure/tall_enclosure/ball_heater_box_quarter.ai), respectively. The case is assembled with 1/2 inch long #6-32 button head screws and nuts in the T-slots.

## Putting it all together

* Cut and prepare wires. Strip 1cm on all ends, and crimp 1/4 inch quick disconnect terminals as specified.

| Use |  Colors | Length | End Treatment|
|---|---|---| --- |
|Power in ground|green |5 inches | crimp one end|
|Power in to switch| yellow + white | 7 inches | crimp both ends|
|Switch to PSU| yellow + white | 11 inches | crimp one end|
|PSU to board| black + red | 6 inches | strip only|

* Attach wires to switch, power in on the bottom, power to PSU on the top, whith white on one  side and yellow on the other.
* Install switch into front panel.
* Attach USB panel mount cable to back panel.
* Attach IEC C14 power inlet to back panel with 6-32 by 5/8 flat head screws and nuts.
* Screw PSU to bottom of case with M3 x 5 button head screws.
* Put rubber feet on bottom of case.
* Attach 4 in QWIIC cable to LCD.
* Screw LCD to front of case with #2-56 by 1/2 inch screws, nuts and washers, using acrylic spacer between LCD and front panel.
* Put the 4 walls and on case.
* Attach 4 1.5 inch 6-32 standoffs to board with 6-32 by 3/8 inch button head screws.
* Remove green O ring from the M8 connector on the PCB
* Attach board to case bottom with more  6-32 by 3/8 inch button head screws.
* Attach wires from switch to power inlet. Yellow to L and white to N.
* Attach green ground wire from power inlet to PSU ground terminal (⏚).
* Attach wires from switch to PSU. Yellow to L and white to N.
* Attach wires from PSU to board, red between V+ on PSU and +5V terminal on board, black from V- to GND on board. Trim stripped end slightly shorter before securing to board if bare wire shows outside the screw terminal.
* Make sure that the switch to the left of the microcontroller is in the down position, towards "Normal Reset"
* Attach USB c cable to the microcontroller on the PCB.
* Screw washer and nut onto rotary encoder and nut onto M8 connector on front of box.
* Screw box together with 6-32 by 1/2 inch button head screws and nuts.
* Put knob onto the rotary encoder.
* Program the microcontroller and test!

## Ball Heater

The ball heater itself is made by wrapping the bottom end of an aluminum ball holder in a layer of kapton tape for electrical insulation, and then a helix of nichrome wire for heating as well as a thermistor for temperature sensing.  All of this is secured and made a thermal unit using thermally conductive epoxy.

### Wiring

The heater is wired using a 4-pin M8 cable as follows :

|Wire Color | Attachment |
| --- | ----|
|Black | Nichrome heater|
|White | Nichrome heater|
|Blue | Thermistor|
|Brown | Thermistor|
