# Hardware Setup

This guide covers the physical connections and hardware requirements for the BNO08x sensor.

## Pin Connections

### I²C Interface (Recommended)

```
MCU              BNO08x
─────────────────────────
3.3V      ────── VIN
GND       ────── GND
SCL       ────── SCL   (with 4.7 kΩ pull-up)
SDA       ────── SDA   (with 4.7 kΩ pull-up)
GPIO      ────── INT   (optional, active-low interrupt)
GPIO      ────── NRST  (optional, active-low reset)
```

**Interface Selection**: PS0=GND, PS1=GND selects I²C mode

**I²C Address**: 
- ADR/SA0 = GND → 0x4A
- ADR/SA0 = VIN → 0x4B

### SPI Interface

```
MCU              BNO08x
─────────────────────────
3.3V      ────── VIN
GND       ────── GND
SCK       ────── SCL
MOSI      ────── SDA
MISO      ────── MISO
CS        ────── CS
GPIO      ────── INT   (optional)
GPIO      ────── NRST  (optional)
GPIO      ────── WAKE  (optional, SPI mode only)
```

**Interface Selection**: PS0=VIN, PS1=VIN selects SPI mode

### UART Interface (RVC Mode)

```
MCU              BNO08x
─────────────────────────
3.3V      ────── VIN
GND       ────── GND
TX        ────── SDA
RX        ────── SCL
GPIO      ────── INT   (optional)
GPIO      ────── NRST  (optional)
```

**Interface Selection**: PS0=GND, PS1=VIN selects UART mode

**Baud Rate**: 115200 bps (8N1)

## Pin Descriptions

| Pin | Name | Description | Required |
|-----|------|-------------|----------|
| VIN | Power | 3.3V power supply | Yes |
| GND | Ground | Ground reference | Yes |
| SCL | Clock | I²C clock (SCL) or SPI clock (SCK) or UART TX | Yes |
| SDA | Data | I²C data (SDA) or SPI data (SDA/MOSI) or UART RX | Yes |
| INT | Interrupt | Active-low interrupt output | No |
| NRST | Reset | Active-low reset input | No |
| WAKE | Wake | SPI mode wake control (active low) | No |
| PS0 | Protocol Select | Interface selection bit 0 | Yes |
| PS1 | Protocol Select | Interface selection bit 1 | Yes |
| ADR/SA0 | Address | I²C address selection | No |

## Power Requirements

- **Supply Voltage**: 3.3V (2.4V - 3.6V typical)
- **Current Consumption**: ~20-30 mA (depends on enabled sensors)
- **Power Supply**: Clean 3.3V supply with decoupling capacitors (100nF ceramic + 10µF tantalum recommended)

## Interface Selection

The BNO08x supports multiple interfaces selected via PS0 and PS1 pins:

| PS1 | PS0 | Interface |
|-----|-----|-----------|
| GND | GND | I²C |
| GND | VIN | UART |
| VIN | GND | UART (RVC mode) |
| VIN | VIN | SPI |

**Note**: Interface selection is sampled at reset. Change pins before resetting the device.

## Physical Layout Recommendations

- Keep I²C/SPI traces short (< 10cm recommended)
- Use ground plane for noise reduction
- Place decoupling capacitors (100nF ceramic + 10µF tantalum) close to VIN pin
- Route clock and data lines away from noise sources
- Use appropriate pull-up resistors for I²C (4.7 kΩ typical)
- Keep sensor away from strong magnetic fields

## Example Wiring Diagram (I²C)

```
                    BNO08x
                    ┌─────────┐
        3.3V ───────┤ VIN     │
        GND  ───────┤ GND     │
        SCL  ───────┤ SCL     │───[4.7kΩ]─── 3.3V
        SDA  ───────┤ SDA     │───[4.7kΩ]─── 3.3V
        INT  ───────┤ INT     │
        NRST ───────┤ NRST    │
        GND  ───────┤ PS0     │
        GND  ───────┤ PS1     │
        GND  ───────┤ ADR     │
                    └─────────┘
```

## Next Steps

- Verify connections with a multimeter
- Proceed to [Quick Start](quickstart.md) to test the connection
- Review [Platform Integration](platform_integration.md) for software setup

---

**Navigation**
⬅️ [Quick Start](quickstart.md) | [Next: Platform Integration ➡️](platform_integration.md) | [Back to Index](index.md)

