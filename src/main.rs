#![no_std]
#![no_main]

const DEVICE_ADRS_FOR_WRITE: u8 = 0xa0;
const DEVICE_ADRS_FOR_READ: u8 = 0xa1;
const UPPER_ADRS: u8 = 0x00;
const LOWER_ADRS: u8 = 0x01;

use core::convert::TryInto;
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use cortex_m_rt::entry;
use cortex_m::delay;    // Delayを使う
use stm32f4::stm32f401;

#[entry]
fn main() -> ! {

    let dp = stm32f401::Peripherals::take().unwrap();   // デバイス用Peripheralsの取得
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();    // cortex-m Peripheralsの取得
    let mut delay = delay::Delay::new(cp.SYST, 84000000_u32);   // Delayの生成
    clock_init(&dp);    // クロック関連の初期化
    gpiob89_init(&dp);  // GPIOBの初期化
    i2c1_init(&dp);     // I2C1の初期化

    i2c_set_ack(&dp, true); // ACKを返す指示
    i2c_start(&dp);
    i2c_control_byte_write(&dp, DEVICE_ADRS_FOR_WRITE);

    i2c_write(&dp, UPPER_ADRS);
    i2c_write(&dp, LOWER_ADRS);

    i2c_write(&dp, 18);   // 0x12
    i2c_write(&dp, 52);   // 0x34
    i2c_write(&dp, 86);   // 0x56

    i2c_stop_after_writing(&dp);

    // 書き込んだアドレス(01番地)から値を読む

    delay.delay_ms(5_u32);  // 休憩する

    i2c_start(&dp);
    i2c_control_byte_write(&dp, DEVICE_ADRS_FOR_WRITE);

    i2c_write(&dp, UPPER_ADRS);
    i2c_write(&dp, LOWER_ADRS);

    i2c_start(&dp);
    i2c_control_byte_write(&dp, DEVICE_ADRS_FOR_READ);

    let _ul = i2c_read(&dp);
    let _um = i2c_read(&dp);
    i2c_set_ack(&dp, false); // ACKを返さない(NAK)指示
    let _uh = i2c_read(&dp);

    i2c_stop_after_reading(&dp);
    loop {
    }
}

fn clock_init(dp: &stm32f401::Peripherals) {

    // PLLSRC = HSI: 16MHz (default)
    dp.RCC.pllcfgr.modify(|_, w| w.pllp().div4());      // P=4
    dp.RCC.pllcfgr.modify(|_, w| unsafe { w.plln().bits(336) });    // N=336
    // PLLM = 16 (default)

    dp.RCC.cfgr.modify(|_, w| w.ppre1().div2());        // APB1 PSC = 1/2
    dp.RCC.cr.modify(|_, w| w.pllon().on());            // PLL On
    while dp.RCC.cr.read().pllrdy().is_not_ready() {    // 安定するまで待つ
        // PLLがロックするまで待つ (PLLRDY)
    }

    // データシートのテーブル15より
    dp.FLASH.acr.modify(|_,w| w.latency().bits(2));    // レイテンシの設定: 2ウェイト

    dp.RCC.cfgr.modify(|_,w| w.sw().pll());     // sysclk = PLL
    while !dp.RCC.cfgr.read().sws().is_pll() {  // SWS システムクロックソースがPLLになるまで待つ
    }
//  SYSCLK = 16MHz * 1/M * N * 1/P
//  SYSCLK = 16MHz * 1/16 * 336 * 1/4 = 84MHz
//  APB2 = 84MHz
//  APB1 = 42MHz (I2C1 pclk)
}

fn gpiob89_init(dp: &stm32f401::Peripherals) {

    dp.RCC.ahb1enr.modify(|_, w| w.gpioben().enabled());            // GPIOBのクロックを有効にする

    dp.GPIOB.otyper.modify(|_, w| w.ot8().open_drain());            // GPIOB8をオープンドレイン出力にする
    dp.GPIOB.otyper.modify(|_, w| w.ot9().open_drain());            // GPIOB9をオープンドレイン出力にする
    dp.GPIOB.ospeedr.modify(|_, w| w.ospeedr8().very_high_speed()); // GPIOB8を高速動作にする
    dp.GPIOB.ospeedr.modify(|_, w| w.ospeedr9().very_high_speed()); // GPIOB9を高速動作にする

    dp.GPIOB.moder.modify(|_, w| w.moder8().alternate());           // GPIOB8をオルタネートに設定    
    dp.GPIOB.afrh.modify(|_, w| w.afrh8().af4());                   // GPIOB8をAF4に設定    
    dp.GPIOB.moder.modify(|_, w| w.moder9().alternate());           // GPIOB9をオルタネートに設定    
    dp.GPIOB.afrh.modify(|_, w| w.afrh9().af4());                   // GPIOB9をAF4に設定    

}

fn i2c1_init(dp: &stm32f401::Peripherals) {

    // I2C1のクロックイネーブル機能は APB1 にある
    dp.RCC.apb1enr.modify(|_,w| w.i2c1en().enabled());      // I2C1のクロックを有効にする

    dp.I2C1.cr2.modify(|_, w| unsafe { w.freq().bits(42) });// クロック=42MHz (APB1)
    dp.I2C1.ccr.modify(|_, w| unsafe { w.bits(210) });
    dp.I2C1.trise.modify(|_, w| unsafe { w.bits(43) }); 
    dp.I2C1.cr1.modify(|_, w| w.pe().enabled());            // ペリフェラルイネーブル

    // f = 100kHz として計算
    // CCR:210の計算
    // Thigh = 1/200kHz, CCR = 42M / 200k = 210
    // TRISE:43の計算
    // 42M * 1000n = 42, 42 + 1 = 43

}

fn i2c_set_ack(dp: &stm32f401::Peripherals, ack: bool) {
    if ack {
        dp.I2C1.cr1.modify(|_, w| w.ack().ack());   // ACKを返すように指示
    } else {
        dp.I2C1.cr1.modify(|_, w| w.ack().nak()); // ACKを返さないように指示
    }
}

fn i2c_start(dp: &stm32f401::Peripherals) {
    dp.I2C1.cr1.modify(|_, w| w.start().set_bit()); // スタートコンディション
    while dp.I2C1.sr1.read().sb().is_no_start() {    // 生成されるまで待つ
    }
}

fn i2c_stop_after_writing(dp: &stm32f401::Peripherals) {
    while dp.I2C1.sr1.read().tx_e().is_not_empty() {    // エンプティでない間待つ
    }
    dp.I2C1.cr1.modify(|_, w| w.stop().set_bit());  // ストップコンディション
}

fn i2c_stop_after_reading(dp: &stm32f401::Peripherals) {
    dp.I2C1.cr1.modify(|_, w| w.stop().set_bit());  // ストップコンディション
}

fn i2c_write(dp: &stm32f401::Peripherals, data: u8) {
    while dp.I2C1.sr1.read().tx_e().is_not_empty() {   // エンプティでない間待つ
    }
    dp.I2C1.dr.write(|w| w.dr().bits(data));    // データをライト
    while dp.I2C1.sr1.read().btf().is_not_finished() {    // byte transfer finished
    }
}

fn i2c_read(dp: &stm32f401::Peripherals) -> u8 {
    while dp.I2C1.sr1.read().rx_ne().is_empty() {   // エンプティの間待つ
    }
    dp.I2C1.dr.read().bits().try_into().unwrap()    // データをリード
}

fn i2c_control_byte_write(dp: &stm32f401::Peripherals, data: u8) {
    dp.I2C1.dr.write(|w| w.dr().bits(data));    // コントロールバイトをライト
    while dp.I2C1.sr1.read().addr().is_not_match() {    // 一致するまで待つ
    }
    let _ = dp.I2C1.sr2.read().busy();  // ダミーリードする
}
