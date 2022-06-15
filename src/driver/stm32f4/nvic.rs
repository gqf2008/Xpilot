use xtask::arch::cortex_m;
use xtask::arch::cortex_m::interrupt::InterruptNumber;

pub trait NVICExt {
    unsafe fn priority<I>(i: I, pro: u8)
    where
        I: InterruptNumber;
}

impl NVICExt for cortex_m::peripheral::NVIC {
    unsafe fn priority<I>(interrupt: I, prio: u8)
    where
        I: InterruptNumber,
    {
        #[cfg(not(armv6m))]
        {
            let nr = interrupt.number();
            (*Self::PTR).ipr[usize::from(nr)].write(prio)
        }

        #[cfg(armv6m)]
        {
            (*Self::PTR).ipr[Self::ipr_index(interrupt)].modify(|value| {
                let mask = 0x0000_00ff << Self::ipr_shift(interrupt);
                let prio = u32::from(prio) << Self::ipr_shift(interrupt);

                (value & !mask) | prio
            })
        }
    }
}
