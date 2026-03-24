package rpio

// GPIO edge detect on RP1 — drivers/pinctrl/pinctrl-rp1.c (Raspberry Pi Ltd.)
const (
	rp1GPIOIRQReset = 1 << 28

	rp1EventsShiftRaw = 20
	rp1IntMask        = 0xf

	rp1IntEdgeFalling = 1 << 0
	rp1IntEdgeRising  = 1 << 1

)

func rp1PinStatusByteOff(pin Pin) int {
	b, o := rp1Bank(pin)
	return rp1IOBankOff[b] + o*8
}

func rp1PinCtrlWordIdx(pin Pin) int {
	return rp1W(rp1PinStatusByteOff(pin) + 4)
}

func rp1PinCtrlSetWordIdx(pin Pin) int {
	return rp1W(rp1PinStatusByteOff(pin) + 4 + rp1SetOffset)
}

func rp1PinCtrlClrWordIdx(pin Pin) int {
	return rp1W(rp1PinStatusByteOff(pin) + 4 + rp1ClrOffset)
}

func rp1BankIntsWordIdx(pin Pin) int {
	b, _ := rp1Bank(pin)
	bankInts := []int{0x124, 0x4124, 0x8124}
	return rp1W(bankInts[b])
}

func rp1DetectEdge(pin Pin, edge Edge) {
	if edge != NoEdge {
		// Do not touch BCM ARM interrupt controller on Pi 5.
	}
	clr := rp1PinCtrlClrWordIdx(pin)
	set := rp1PinCtrlSetWordIdx(pin)
	// Clear raw event enables
	gpioMem[clr] = rp1IntMask << rp1EventsShiftRaw
	// IRQ reset
	gpioMem[set] = rp1GPIOIRQReset

	var flags uint32
	if edge&RiseEdge > 0 {
		flags |= rp1IntEdgeRising
	}
	if edge&FallEdge > 0 {
		flags |= rp1IntEdgeFalling
	}
	gpioMem[set] = flags << rp1EventsShiftRaw
}

func rp1EdgeDetected(pin Pin) bool {
	_, off := rp1Bank(pin)
	intsIdx := rp1BankIntsWordIdx(pin)
	v := gpioMem[intsIdx] & (1 << off)
	if v != 0 {
		gpioMem[rp1PinCtrlSetWordIdx(pin)] = rp1GPIOIRQReset
	}
	return v != 0
}
