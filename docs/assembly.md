# Ball Heater Controller Assembly

The parts needed to build the ball heater are listed with links and part numbers in the first sheet of the [Bill of Materials (BOM)](../ball_heater_controller_pcb/ball_heater_controller_bom_V%201.ods) spreadsheet.

## PCB

PCBs can be ordered from one of many PCB supply houses, eg [pcbway](pcbway.com) or [jlcpcb](https://jlcpcb.com), using the [zipped gerber files](../ball_heater_controller_pcb/ball_heater_controller_v1.zip). Order a stencil as well to make the surface mount assembly easier and faster.

Once PCBs and parts have been acquired, apply solder paste using the stencil, and then use the [BOM printout](../ball_heater_controller_pcb/ball_heater_BOM_printout.pdf) to place surface mount parts in the correct locations matching their reference designators.  Then reflow the solder paste using a reflow over or hot air rework system. Lastly, solder the through hole components.

## Enclosure

Laser cut the two enclosure parts files out of 1/8 inch and 1/4 inch acrylic for [ball_heater_box_eighth](../ball_heater_controller_enclosure/short_enclosure/ball_heater_box_eighth.ai) and [ball_heater_box_quarter](../ball_heater_controller_enclosure/short_enclosure/ball_heater_box_quarter.ai), respectively. The case is assembled with 1/2 inch long #6-32 button head screws and nuts in the T-slots.

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
