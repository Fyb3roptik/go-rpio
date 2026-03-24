package rpio

import (
	"reflect"
	"unsafe"
)

// RP1 PCIe BAR1 — see Linux include/dt-bindings/mfd/rp1.h.
const (
	rp1BarPhys   int64 = 0x1f00000000
	rp1BarLength       = 0x400000

	// GPIO controller (IO banks + SYS_RIO + pads + irq status) inside BAR.
	rp1GPIOChipOff   = 0x0d0000
	rp1GPIOChipBytes = 0x30000
)

var (
	// rp1FullBar is true when the full RP1 BAR is mapped (root). SPI and PWM
	// require it; /dev/gpiomem0 only exposes the GPIO chip window.
	rp1FullBar bool
	rp1Bar8    []byte
	rp1BarU32  []uint32
)

func bytesToUint32Slice(mem8 []byte) []uint32 {
	if len(mem8) == 0 {
		return nil
	}
	header := *(*reflect.SliceHeader)(unsafe.Pointer(&mem8))
	header.Len /= 4
	header.Cap /= 4
	return *(*[]uint32)(unsafe.Pointer(&header))
}

func rp1BarWord(off int) int { return off / 4 }
