package rpio

import (
	"time"
)

// RP1 PWM — drivers/pwm/pwm-rp1.c (Raspberry Pi Ltd.)
const (
	rp1PWM0Off = 0x098000
	rp1PWM1Off = 0x09c000

	rp1PWMGlobalCtrl  = 0x000 / 4
	rp1PWMChStride    = 16 / 4 // words per channel group
	rp1PWMChCtrl0     = 0x014 / 4
	rp1PWMSETUpdate   = 1 << 31
	rp1PWMChannelDefault = (1 << 8) | (1 << 0)
)

// rp1PwmInputHz is the PWM block input clock (approximate). Linux uses
// clk_get_rate(); ~125 MHz is typical on Pi 5.
var rp1PwmInputHz uint32 = 125_000_000

func rp1PWMRegs(block int) []uint32 {
	if !rp1FullBar || len(rp1BarU32) == 0 {
		return nil
	}
	off := rp1PWM0Off
	if block == 1 {
		off = rp1PWM1Off
	}
	base := rp1BarWord(off)
	if base+32 > len(rp1BarU32) {
		return nil
	}
	return rp1BarU32[base : base+32]
}

func rp1PwmPinChannel(pin Pin) (block, ch int, ok bool) {
	switch pin {
	case 12:
		return 0, 0, true
	case 13:
		return 0, 1, true
	case 18:
		return 0, 2, true
	case 19:
		return 0, 3, true
	case 40:
		return 1, 1, true
	case 41:
		return 1, 2, true
	case 45:
		return 1, 3, true
	default:
		return 0, 0, false
	}
}

func rp1PwmFsel(pin Pin) (uint32, bool) {
	switch pin {
	case 12, 13:
		return 0, true
	case 18, 19:
		return 3, true
	case 40, 41, 45:
		return 0, true
	default:
		return 0, false
	}
}

func rp1PinModePwm(pin Pin) {
	f, ok := rp1PwmFsel(pin)
	if !ok {
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

func rp1SetFreqPWM(pin Pin, freq int) {
	if freq <= 0 {
		return
	}
	rp1PwmInputHz = uint32(freq)
}

func rp1SetDutyCycleWithPwmMode(pin Pin, dutyLen, cycleLen uint32, mode bool) {
	if !rp1FullBar {
		return
	}
	block, ch, ok := rp1PwmPinChannel(pin)
	if !ok || cycleLen == 0 {
		return
	}
	regs := rp1PWMRegs(block)
	if regs == nil {
		return
	}
	ctrlIdx := rp1PWMChCtrl0 + ch*rp1PWMChStride
	rngIdx := ctrlIdx + 1
	dutyIdx := ctrlIdx + 2

	v := uint32(rp1PWMChannelDefault)
	if mode == Balanced {
		v &^= 1 << 0
	}
	regs[ctrlIdx] = v
	regs[dutyIdx] = dutyLen
	regs[rngIdx] = cycleLen

	g := regs[rp1PWMGlobalCtrl]
	g |= rp1PWMSETUpdate
	g |= uint32(1) << ch
	regs[rp1PWMGlobalCtrl] = g
	time.Sleep(time.Microsecond * 10)
}

func rp1StopPwm() {
	for b := 0; b < 2; b++ {
		regs := rp1PWMRegs(b)
		if regs == nil {
			continue
		}
		regs[rp1PWMGlobalCtrl] &^= 0xf
		regs[rp1PWMGlobalCtrl] |= rp1PWMSETUpdate
	}
}

func rp1StartPwm() {
	// Per-channel enables are set in rp1SetDutyCycleWithPwmMode.
}
