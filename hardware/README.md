# Reference shield – hardware files

KiCad 10 design of the Arduino Uno shield used to develop and test the
**Quectel_LCx6G_GNSS** library.

> **Revision 1 is a test prototype, not a final design.** It works (every
> library function was tested on it), but it has the known issues listed
> below. An improved **revision 2** will follow; its files will be added in
> a separate `rev2/` folder.

![Reference shield, revision 1](../extras/images/shield_rev1_top.jpg)

## Folder content

| Path | Content |
|---|---|
| `rev1/Quectel_LC76G.kicad_pro` | KiCad 10 project |
| `rev1/Quectel_LC76G.kicad_sch` | Root schematic sheet |
| `rev1/01_Power_Supply.kicad_sch` | LDO, switched module rail (VCC_GNSS), backup rail (V_BCKP) |
| `rev1/02_MCU_Interface.kicad_sch` | Uno shield connector, UART level shifter (RXD) |
| `rev1/03_GNSS_Module.kicad_sch` | LC76G module, RESET_N driver, I2C pull-ups, test points |
| `rev1/04_Passive_Antenna.kicad_sch` | Molex patch antenna, matching network, optional SAW filter (DNP) |
| `rev1/Quectel_LC76G.kicad_pcb` | PCB layout |
| `rev1/Quectel_LC76G_schematic.pdf` | Schematic, all sheets (readable without KiCad) |
| `rev1/Quectel_LC76G_BOM.csv` | Bill of materials, with LCSC part numbers |

The custom symbols and footprints (LC76G module, Molex antenna, SAW filter)
are embedded in the schematic and PCB files: the project opens in KiCad 10
without any extra library.

## Board summary

| Item | Value |
|---|---|
| Layers | 2, FR4 1.6 mm, 1 oz copper, GND pour on both sides |
| GNSS module | Quectel LC76G (PA), UART + I2C (SPI not used) |
| Antenna | Molex 211624-0001 passive patch, RHCP, 50 Ω, π matching network |
| RF line | Coplanar, 0.92 mm width, no vias; ESD diode RCLAMP0521P |
| Supply | 3.7 V Li-ion/Li-Po cell on BT1 → XC6206 3.3 V LDO |
| Module rail | VCC_GNSS switched by a P-MOSFET with soft start, driven by PWR_EN_VCC (D3) |
| Backup rail | V_BCKP always powered (RTC and satellite data kept with the module off) |
| Reset | RESET_N driven by an open-collector NPN (D6) |
| UART | Module TXD → D4, D5 → module RXD through a BSS138 level shifter |
| I2C | 2.2 kΩ pull-ups to the module rail |
| Test points | RESET_N, TXD, RXD, 1PPS, V_BCKP, VCC_GNSS, I2C_SDA, I2C_SCL, GND |

## Revision 1 – files vs. first batch of boards

The files in `rev1/` include two fixes compared to the first batch of
boards (the one in the pictures):

| Part | Files in `rev1/` | First-batch boards |
|---|---|---|
| U1 (XC6206P332MR, LCSC C5148692) | Footprint with the XC6206 pinout (1 GND, 2 VOUT, 3 VIN) | Footprint with the AP2204R pinout: U1 soldered rotated by hand |
| D1/D2 (TVS on VCC_3.3 and VCC_GNSS) | SMF5.0CA bidirectional (LCSC C908214), no polarity | SMF5.0A unidirectional (C908213), cathode pad on GND: diodes rotated by 180° |

Known issues still present in `rev1/`, to be fixed in revision 2 (see also
[Reference shield, revision 1 – known issues](../README.md#reference-shield-revision-1--known-issues)):

- **Q1/Q4**: no series base resistor and no base pull-down. Use
  `LC76G_DRIVE_WEAK_PULLUP_HIGH` on both control pins.
- **No level shifter** on I2C and on the module TXD line (3.3 V signals
  to a 5 V Arduino).
- **Uno R4**: the hardware UART needs two wires (D4 → D0, D5 → D1).

## Planned for revision 2

- Series base resistors and base pull-downs on Q1/Q4
- Level shifters on I2C and on the module TXD line
- Solder jumpers to choose D4/D5 or D0/D1 for the UART
- LED showing the fix status

## License

These hardware files are released under the same MIT license as the
library, see [LICENSE](../LICENSE). They are provided as they are,
without any warranty: check the design before building it.
