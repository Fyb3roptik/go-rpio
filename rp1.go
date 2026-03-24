package rpio

// Raspberry Pi 5 (RP1 southbridge) GPIO. Register layout matches
// raspberrypi/utils pinctrl/gpiochip_rp1.c and the RP1 peripherals datasheet.

const (
	rp1MapBytes = 0x40000 // >= kernel gpiochip size 0x30000

	rp1SetOffset = 0x2000
	rp1ClrOffset = 0x3000

	rp1IOBank0 = 0x00000000
	rp1IOBank1 = 0x00004000
	rp1IOBank2 = 0x00008000

	rp1SysRioBank0 = 0x00010000
	rp1SysRioBank1 = 0x00014000
	rp1SysRioBank2 = 0x00018000

	rp1PadsBank0 = 0x00020000
	rp1PadsBank1 = 0x00024000
	rp1PadsBank2 = 0x00028000

	rp1GpioSysRioOut   = 0x0
	rp1GpioSysRioOE   = 0x4
	rp1GpioSysRioSync = 0x8

	rp1GpioCtrlFselMask = 0x1f << 0
	rp1FselSysRio       = 5

	rp1PadsOD = 1 << 7
	rp1PadsIE = 1 << 6
	rp1PadsPUE = 1 << 3
	rp1PadsPDE = 1 << 2

	// Pi 5: RP1 GPIO block physical address (low 32-bit alias works with /dev/mem on Pi OS).
	rp1GpioPhysBase = 0x1f000d0000
)

var rp1IOBankOff = []int{rp1IOBank0, rp1IOBank1, rp1IOBank2}
var rp1SysRioOff = []int{rp1SysRioBank0, rp1SysRioBank1, rp1SysRioBank2}
var rp1PadsOff = []int{rp1PadsBank0, rp1PadsBank1, rp1PadsBank2}

// rp1BankBase is the first GPIO index in each IO bank (gpiochip_rp1.c).
var rp1BankBase = []int{0, 28, 34}

func rp1Bank(pin Pin) (bank, offset int) {
	p := int(pin)
	switch {
	case p < rp1BankBase[1]:
		return 0, p
	case p < rp1BankBase[2]:
		return 1, p - rp1BankBase[1]
	default:
		return 2, p - rp1BankBase[2]
	}
}

func rp1W(byteOff int) int { return byteOff / 4 }

func rp1IOCtrlOff(bank, offsetInBank int) int {
	// ((offset * 2) + 1) * 4 bytes from IO bank start
	return rp1IOBankOff[bank] + ((offsetInBank*2)+1)*4
}

func rp1PadsRegOff(bank, offsetInBank int) int {
	// sizeof(uint32) + offset * sizeof(uint32) from pads bank start
	return rp1PadsOff[bank] + 4 + offsetInBank*4
}

func rp1SysRioIdx(bank, regByte int) int {
	return rp1W(rp1SysRioOff[bank] + regByte)
}

func rp1SysRioAliasIdx(bank, regByte, alias int) int {
	return rp1W(rp1SysRioOff[bank] + regByte + alias)
}

func rp1CtrlRead(pin Pin) uint32 {
	bank, off := rp1Bank(pin)
	return gpioMem[rp1W(rp1IOCtrlOff(bank, off))]
}

func rp1CtrlWrite(pin Pin, v uint32) {
	bank, off := rp1Bank(pin)
	gpioMem[rp1W(rp1IOCtrlOff(bank, off))] = v
}

func rp1PadsRead(pin Pin) uint32 {
	bank, off := rp1Bank(pin)
	return gpioMem[rp1W(rp1PadsRegOff(bank, off))]
}

func rp1PadsWrite(pin Pin, v uint32) {
	bank, off := rp1Bank(pin)
	gpioMem[rp1W(rp1PadsRegOff(bank, off))] = v
}

func rp1PinMode(pin Pin, mode Mode) {
	switch mode {
	case Input, Output:
		bank, off := rp1Bank(pin)
		input := mode == Input
		// SYS_RIO + pad enables (see gpiochip_rp1 rp1_gpio_set_fsel).
		if input {
			gpioMem[rp1SysRioAliasIdx(bank, rp1GpioSysRioOE, rp1ClrOffset)] = 1 << off
		} else {
			gpioMem[rp1SysRioAliasIdx(bank, rp1GpioSysRioOE, rp1SetOffset)] = 1 << off
		}
		ctrl := rp1CtrlRead(pin) &^ uint32(rp1GpioCtrlFselMask)
		ctrl |= rp1FselSysRio
		rp1CtrlWrite(pin, ctrl)

		pad := rp1PadsRead(pin)
		pad |= rp1PadsIE
		pad &^= rp1PadsOD
		rp1PadsWrite(pin, pad)
	case Alt0, Alt1, Alt2, Alt3, Alt4, Alt5:
		var f uint32
		switch mode {
		case Alt0:
			f = 0
		case Alt1:
			f = 1
		case Alt2:
			f = 2
		case Alt3:
			f = 3
		case Alt4:
			f = 4
		case Alt5:
			f = rp1FselSysRio
		}
		ctrl := rp1CtrlRead(pin) &^ uint32(rp1GpioCtrlFselMask)
		ctrl |= f
		rp1CtrlWrite(pin, ctrl)
		pad := rp1PadsRead(pin)
		if f == 0x1f {
			pad &^= rp1PadsIE
			pad |= rp1PadsOD
		} else {
			pad |= rp1PadsIE
			pad &^= rp1PadsOD
		}
		rp1PadsWrite(pin, pad)
	case Spi:
		var f uint32
		switch pin {
		case 7, 8, 9, 10, 11:
			f = 0
		case 16, 17, 18, 19, 20, 21:
			f = 0
		case 40, 41, 42, 43, 44, 45:
			f = 4
		default:
			return
		}
		ctrl := rp1CtrlRead(pin) &^ uint32(rp1GpioCtrlFselMask)
		ctrl |= f
		rp1CtrlWrite(pin, ctrl)
		pad := rp1PadsRead(pin)
		pad |= rp1PadsIE
		pad &^= rp1PadsOD
		rp1PadsWrite(pin, pad)
	case Pwm:
		rp1PinModePwm(pin)
	case Clock:
		rp1PinModeGpclk(pin)
	default:
		return
	}
}

func rp1PinModeGpclk(pin Pin) {
	var f uint32
	switch pin {
	case 4, 5, 6, 32, 34, 42, 43, 44:
		f = 0
	case 20, 21:
		f = 3 // GPCLK alt; SetFreq for GPCLK not implemented on RP1
	default:
		return
	}
	ctrl := rp1CtrlRead(pin) &^ uint32(rp1GpioCtrlFselMask)
	ctrl |= f
	rp1CtrlWrite(pin, ctrl)
	pad := rp1PadsRead(pin)
	pad |= rp1PadsIE
	pad &^= rp1PadsOD
	rp1PadsWrite(pin, pad)
}

func rp1WritePin(pin Pin, state State) {
	bank, off := rp1Bank(pin)
	if state == Low {
		gpioMem[rp1SysRioAliasIdx(bank, rp1GpioSysRioOut, rp1ClrOffset)] = 1 << off
	} else {
		gpioMem[rp1SysRioAliasIdx(bank, rp1GpioSysRioOut, rp1SetOffset)] = 1 << off
	}
}

func rp1ReadPin(pin Pin) State {
	bank, off := rp1Bank(pin)
	pad := rp1PadsRead(pin)
	if pad&rp1PadsIE == 0 {
		return Low
	}
	v := gpioMem[rp1SysRioIdx(bank, rp1GpioSysRioSync)]
	if v&(1<<off) != 0 {
		return High
	}
	return Low
}

func rp1TogglePin(pin Pin) {
	if rp1ReadPin(pin) == High {
		rp1WritePin(pin, Low)
	} else {
		rp1WritePin(pin, High)
	}
}

func rp1PullMode(pin Pin, pull Pull) {
	pad := rp1PadsRead(pin)
	pad &^= rp1PadsPUE | rp1PadsPDE
	switch pull {
	case PullUp:
		pad |= rp1PadsPUE
	case PullDown:
		pad |= rp1PadsPDE
	}
	rp1PadsWrite(pin, pad)
}

func rp1ReadPull(pin Pin) Pull {
	reg := rp1PadsRead(pin)
	if reg&rp1PadsPUE != 0 {
		return PullUp
	}
	if reg&rp1PadsPDE != 0 {
		return PullDown
	}
	return PullOff
}
