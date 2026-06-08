//! Top-priority current-loop ISR (~20 kHz).
//! Read MT6816 + 2 phase currents -> Clarke/Park -> current PI ->
//! inverse Park/SVM -> PWM compares. Uninterruptible by anything below.
//! Reads target d/q current from the boundary; on decimation tick, pends
//! the cascade interrupt. ~3200 cycle budget @ 64MHz / 50us.
