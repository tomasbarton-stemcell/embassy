use crate::pac::{PWR, RCC};
use crate::rcc::{set_freqs, Clocks};
use crate::time::Hertz;

/// HSI speed
pub const HSI_FREQ: Hertz = Hertz(16_000_000);

/// LSI speed
pub const LSI_FREQ: Hertz = Hertz(32_000);

#[derive(Clone, Copy)]
pub enum PLLSource {
    HSE(Hertz),
    HSI16,
}

/// System clock mux source
#[derive(Clone, Copy)]
pub enum ClockSrc {
    HSE(Hertz),
    HSI16,
    PLL(PLLSource, Hertz),
}

/// AHB prescaler
#[derive(Clone, Copy, PartialEq)]
pub enum AHBPrescaler {
    NotDivided,
    Div2,
    Div4,
    Div8,
    Div16,
    Div64,
    Div128,
    Div256,
    Div512,
}

/// APB prescaler
#[derive(Clone, Copy)]
pub enum APBPrescaler {
    NotDivided,
    Div2,
    Div4,
    Div8,
    Div16,
}

impl Into<u8> for APBPrescaler {
    fn into(self) -> u8 {
        match self {
            APBPrescaler::NotDivided => 1,
            APBPrescaler::Div2 => 0x04,
            APBPrescaler::Div4 => 0x05,
            APBPrescaler::Div8 => 0x06,
            APBPrescaler::Div16 => 0x07,
        }
    }
}

impl Into<u8> for AHBPrescaler {
    fn into(self) -> u8 {
        match self {
            AHBPrescaler::NotDivided => 1,
            AHBPrescaler::Div2 => 0x08,
            AHBPrescaler::Div4 => 0x09,
            AHBPrescaler::Div8 => 0x0a,
            AHBPrescaler::Div16 => 0x0b,
            AHBPrescaler::Div64 => 0x0c,
            AHBPrescaler::Div128 => 0x0d,
            AHBPrescaler::Div256 => 0x0e,
            AHBPrescaler::Div512 => 0x0f,
        }
    }
}

/// Clocks configutation
pub struct Config {
    pub mux: ClockSrc,
    pub ahb_pre: AHBPrescaler,
    pub apb1_pre: APBPrescaler,
    pub apb2_pre: APBPrescaler,
    pub low_power_run: bool,
}

impl Default for Config {
    #[inline]
    fn default() -> Config {
        Config {
            mux: ClockSrc::HSI16,
            ahb_pre: AHBPrescaler::NotDivided,
            apb1_pre: APBPrescaler::NotDivided,
            apb2_pre: APBPrescaler::NotDivided,
            low_power_run: false,
        }
    }
}

fn pllr_flag(pllr: u32) -> u8 {
    match pllr {
        2 => 0,
        4 => 1,
        6 => 2,
        8 => 3,
        _ => panic!("invalid PLLR"),
    }
}

fn pllq_flag(pllq: u32) -> u8 {
    match pllq {
        2 => 0,
        4 => 1,
        6 => 2,
        8 => 3,
        _ => panic!("invalid PLLQ"),
    }
}

pub(crate) unsafe fn init(config: Config) {
    let (sys_clk, sw) = match config.mux {
        ClockSrc::HSI16 => {
            // Enable HSI16
            RCC.cr().write(|w| w.set_hsion(true));
            while !RCC.cr().read().hsirdy() {}

            (HSI_FREQ.0, 0x01)
        }
        ClockSrc::HSE(freq) => {
            // Enable HSE
            RCC.cr().write(|w| w.set_hseon(true));
            while !RCC.cr().read().hserdy() {}

            (freq.0, 0x02)
        }
        ClockSrc::PLL(source, target_freq) => {
            let pll_input = match source {
                PLLSource::HSE(freq) => freq,
                PLLSource::HSI16 => Hertz::mhz(16),
            };

            let pllm = 4;

            // VCO Input 2.66-16 MHz
            let vco_input = Hertz(pll_input.0 / pllm);

            // TODO should be configurable across devices?
            if vco_input > Hertz::mhz(16) {
                panic!("VCO Input frequency too high");
            }
            if vco_input < Hertz(2_660_000) {
                panic!("VCO Input frequency too low")
            }

            // VCO Output range 96-344Mhz or 96-128MHz (depends on voltage scaling range)
            // Assume voltage scaling range 1

            let pllr = 2;
            let plln = target_freq.0 * pllr / vco_input.0;

            let vco_output = Hertz(vco_input.0 * plln);
            let r_output = Hertz(vco_output.0 / pllr);

            if vco_output > Hertz::mhz(344) {
                panic!("vco output frequency too high");
            }
            if vco_output < Hertz::mhz(96) {
                panic!("vco output frequency too low");
            }

            let q_freq = Hertz::mhz(48);
            if vco_output.0 % q_freq.0 != 0 {
                panic!("cannot divide vco output to 48MHz");
            }

            let pllq = vco_output.0 / q_freq.0;

            // Configure PLL
            RCC.pllcfgr().write(|w| {
                w.set_pllr(pllr_flag(pllr));
                w.set_pllren(true);
                w.set_pllq(pllq_flag(pllq));
                w.set_pllqen(true);
                w.set_plln(plln.try_into().unwrap());
                w.set_pllm(u8::try_from(pllm - 1).unwrap());
                w.set_pllsrc(match source {
                    PLLSource::HSE(_) => 0b11,
                    PLLSource::HSI16 => 0b10,
                })
            });

            if let PLLSource::HSE(_) = source {
                // Start HSE if it is required as the source for PLL
                RCC.cr().write(|w| w.set_hseon(true));
                while !RCC.cr().read().hserdy() {}
            }

            // Start PLL
            RCC.cr().write(|w| {
                w.set_pllon(true);
                if let PLLSource::HSE(_) = source {
                    w.set_hseon(true);
                };
            });

            // Wait for HSE
            if let PLLSource::HSE(_) = source {
                while !RCC.cr().read().hserdy() {}
            }

            // wait for the PLL
            while !RCC.cr().read().pllrdy() {}

            // use PLLQ for the USB
            RCC.ccipr().modify(|q| {
                q.set_clk48sel(0b10);
            });

            (r_output.0, 0x03)
        }
    };

    // let latency = if sys_clk <= 30 {
    //     0
    // } else if sys_clk <= 60 {
    //     1
    // } else if sys_clk <= 90 {
    //     2
    // } else if sys_clk <= 120 {
    //     3
    // } else {
    //     4
    // };

    // TODO need to change memory wait cycles for faster system clock

    RCC.cfgr().modify(|w| {
        w.set_sw(sw.into());
        w.set_hpre(config.ahb_pre.into());
        w.set_ppre1(config.apb1_pre.into());
        w.set_ppre2(config.apb2_pre.into());
    });

    // wait for the switch
    while RCC.cfgr().read().sws() != sw {}

    let ahb_freq: u32 = match config.ahb_pre {
        AHBPrescaler::NotDivided => sys_clk,
        pre => {
            let pre: u8 = pre.into();
            let pre = 1 << (pre as u32 - 7);
            sys_clk / pre
        }
    };

    let (apb1_freq, apb1_tim_freq) = match config.apb1_pre {
        APBPrescaler::NotDivided => (ahb_freq, ahb_freq),
        pre => {
            let pre: u8 = pre.into();
            let pre: u8 = 1 << (pre - 3);
            let freq = ahb_freq / pre as u32;
            (freq, freq * 2)
        }
    };

    let (apb2_freq, apb2_tim_freq) = match config.apb2_pre {
        APBPrescaler::NotDivided => (ahb_freq, ahb_freq),
        pre => {
            let pre: u8 = pre.into();
            let pre: u8 = 1 << (pre - 3);
            let freq = ahb_freq / pre as u32;
            (freq, freq * 2)
        }
    };

    if config.low_power_run {
        assert!(sys_clk <= 2_000_000);
        PWR.cr1().modify(|w| w.set_lpr(true));
    }

    set_freqs(Clocks {
        sys: Hertz(sys_clk),
        ahb1: Hertz(ahb_freq),
        ahb2: Hertz(ahb_freq),
        apb1: Hertz(apb1_freq),
        apb1_tim: Hertz(apb1_tim_freq),
        apb2: Hertz(apb2_freq),
        apb2_tim: Hertz(apb2_tim_freq),
    });
}
