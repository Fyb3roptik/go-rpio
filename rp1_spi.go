package rpio

import (
	"errors"
)

// Synopsys DesignWare APB SSI (RP1 SPI) — see Linux spi-dw.h / praktronics rpi5-rp1-spi.
const (
	dwSPI_CTRLR0   = 0x00
	dwSPI_SSIENR   = 0x08
	dwSPI_SER      = 0x10
	dwSPI_BAUDR    = 0x14
	dwSPI_SR       = 0x28
	dwSPI_ICR      = 0x48
	dwSPI_DR       = 0x60
	dwSPI_SCR      = 0xf8 // CS override (optional)

	dwSR_BUSY        = 1 << 0
	dwSR_TF_NOT_FULL = 1 << 1
	dwSR_RF_NOT_EMPT = 1 << 3

	dwCTRLR0_SCPHA = 1 << 6
	dwCTRLR0_SCPOL = 1 << 7

	rp1SPI0Off = 0x050000
	rp1SPI1Off = 0x054000
	rp1SPI2Off = 0x058000

	rp1SPIClockHz = 200_000_000 // input clock to SPI block (see rpi5-rp1-spi README)
)

var (
	rp1SpiActive []uint32 // slice into rp1BarU32 for current controller
)

// ErrRP1GPIOOnly is returned when SPI/PWM need the full RP1 BAR but only
// /dev/gpiomem0 (GPIO-only) is mapped. Run as root so Open() can mmap BAR1.
var ErrRP1GPIOOnly = errors.New("rpio: Pi 5 SPI requires full RP1 memory map (run as root); /dev/gpiomem0 is GPIO-only")

func rp1SPIOffset(dev SpiDev) int {
	switch dev {
	case Spi0:
		return rp1SPI0Off
	case Spi1:
		return rp1SPI1Off
	case Spi2:
		return rp1SPI2Off
	default:
		return -1
	}
}

func rp1SpiBegin(dev SpiDev) error {
	if !isRP1 {
		return nil
	}
	if !rp1FullBar {
		return ErrRP1GPIOOnly
	}
	off := rp1SPIOffset(dev)
	if off < 0 {
		return SpiMapError
	}
	base := rp1BarWord(off)
	if base+64 > len(rp1BarU32) {
		return SpiMapError
	}
	rp1SpiActive = rp1BarU32[base : base+64]

	r := rp1SpiActive
	r[dwSPI_SSIENR/4] = 0
	for _, pin := range getSpiPins(dev) {
		pin.Mode(Spi)
	}
	// 8-bit data, Motorola SPI, transmit+receive
	r[dwSPI_CTRLR0/4] = 0x0007
	r[dwSPI_BAUDR/4] = 128
	r[dwSPI_ICR/4] = 0xffffffff
	r[dwSPI_SER/4] = 0
	r[dwSPI_SSIENR/4] = 1
	if r[dwSPI_SSIENR/4] == 0 {
		return SpiMapError
	}
	return nil
}

func rp1SpiEnd(dev SpiDev) {
	if !isRP1 || len(rp1SpiActive) == 0 {
		return
	}
	rp1SpiActive[dwSPI_SSIENR/4] = 0
	rp1SpiActive = nil
}

func rp1SpiClearFifo() {
	if len(rp1SpiActive) == 0 {
		return
	}
	rp1SpiActive[dwSPI_ICR/4] = 0xffffffff
}

func rp1SpiSpeed(speed int) {
	if !isRP1 || len(rp1SpiActive) == 0 || speed <= 0 {
		return
	}
	div := uint32(rp1SPIClockHz / speed)
	if div < 2 {
		div = 2
	}
	if div&1 != 0 {
		div++
	}
	rp1SpiActive[dwSPI_SSIENR/4] = 0
	rp1SpiActive[dwSPI_BAUDR/4] = div
	rp1SpiActive[dwSPI_SSIENR/4] = 1
}

func rp1SpiChipSelect(chip uint8) {
	if !isRP1 || len(rp1SpiActive) == 0 {
		return
	}
	rp1SpiActive[dwSPI_SER/4] = 1 << (chip & 3)
}

func rp1SpiChipSelectPolarity(chip uint8, polarity uint8) {
	// DW SPI CS polarity is often pad-controlled on RP1; minimal no-op.
	_ = chip
	_ = polarity
}

func rp1SpiMode(polarity uint8, phase uint8) {
	if !isRP1 || len(rp1SpiActive) == 0 {
		return
	}
	rp1SpiActive[dwSPI_SSIENR/4] = 0
	v := rp1SpiActive[dwSPI_CTRLR0/4]
	if polarity != 0 {
		v |= dwCTRLR0_SCPOL
	} else {
		v &^= dwCTRLR0_SCPOL
	}
	if phase != 0 {
		v |= dwCTRLR0_SCPHA
	} else {
		v &^= dwCTRLR0_SCPHA
	}
	rp1SpiActive[dwSPI_CTRLR0/4] = v
	rp1SpiActive[dwSPI_SSIENR/4] = 1
}

func rp1SpiExchange(data []byte) {
	if !isRP1 || len(rp1SpiActive) == 0 {
		return
	}
	r := rp1SpiActive
	dr := dwSPI_DR / 4
	for i := range data {
		for r[dwSPI_SR/4]&dwSR_BUSY != 0 {
		}
		for r[dwSPI_SR/4]&dwSR_TF_NOT_FULL == 0 {
		}
		r[dr] = uint32(data[i])
		for r[dwSPI_SR/4]&dwSR_RF_NOT_EMPT == 0 {
		}
		data[i] = byte(r[dr])
	}
	for r[dwSPI_SR/4]&dwSR_BUSY != 0 {
	}
}
