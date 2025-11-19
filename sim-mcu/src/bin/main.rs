#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_time::{Duration as EDuration, Timer as ETimer};

use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::peripherals::Peripherals;
use esp_hal::system::Stack;
use esp_hal::time::Duration as HDuration;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::timer::PeriodicTimer;
use esp_hal::Blocking;
use esp_hal::{clock::CpuClock, peripherals};
use heapless::spsc::{Consumer, Producer, Queue};

use libm::{cosf, sinf, sqrtf};
use rtt_target::rprintln;
use static_cell::StaticCell;

use hello_embassy::*;

// if false -> DTSS integrator will be used.
const USE_QSS1: bool = false;

static TELE_Q: StaticCell<Queue<Sample, 256>> = StaticCell::new();

// Core1 stack via StaticCell (so no unsafe static mut)
static CORE1_STACK: StaticCell<Stack<8192>> = StaticCell::new();

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// Push to the producer queue (does not drop packets if full, may block)
fn push_blocking(prod: &mut Producer<'static, Sample>, sample: Sample) {
    let mut s = sample;
    loop {
        if prod.enqueue(s).is_ok() {
            break;
        }
        // Wait a bit to let consumer catch up (spin or small delay)
        core::hint::spin_loop();
    }
}

fn run_dtss<'d>(
    mut prod: Producer<'static, Sample>,
    mut periodic: PeriodicTimer<'d, Blocking>,
) -> ! {
    let (mut state, params, controller, q_target) = default_problem();

    let dt = 0.02f32;
    let mut t = 0.0f32;
    let t_end = 100.0;

    loop {
        let q_error = state.q.multiply(&q_target.conjugate());
        let torque = controller.compute_torque(&q_error, &state.w);

        let energies = compute_energies(&state, &torque);
        step_dtss_euler(&mut state, &params, &torque, dt);

        let sample = Sample {
            t,
            q: [state.q.w, state.q.x, state.q.y, state.q.z],
            w: state.w,
            rw: state.rw,
            p_mech: energies.p_body,
            p_wheels: energies.p_wheels,
        };
        let _ = prod.enqueue(sample);
        //push_blocking(&mut prod, sample);

        t += dt;
        if t >= t_end {
            rprintln!("# DTSS control loop finished at t={:.2}s", t);
            loop {
                core::hint::spin_loop();
            }
        }

        periodic.wait();
    }
}

// ---- QSS1 path ----

fn run_qss1(mut prod: Producer<'static, Sample>) -> ! {
    let (state0, params, controller, q_target) = default_problem();

    let x0 = state_to_vec(&state0);
    let delta_q: [f32; 10] = [
        1e-4, 1e-4, 1e-4, 1e-4, // q
        1e-4, 1e-4, 1e-4, // w
        1e-2, 1e-2, 1e-2, // rw
    ];

    let t0 = 0.0;
    let t_final = 100.0;

    solve_qss1_embedded(
        t0,
        t_final,
        x0,
        delta_q,
        &params,
        &controller,
        &q_target,
        |t, x| {
            let state = vec_to_state(x);

            let q_err = state.q.multiply(&q_target.conjugate());
            let torque = controller.compute_torque(&q_err, &state.w);
            let energies = compute_energies(&state, &torque);

            let sample = Sample {
                t,
                q: [state.q.w, state.q.x, state.q.y, state.q.z],
                w: state.w,
                rw: state.rw,
                p_mech: energies.p_body,
                p_wheels: energies.p_wheels,
            };

            let _ = prod.enqueue(sample);
            //push_blocking(&mut prod, sample);
        },
    );

    rprintln!("# QSS1 simulation finished at t={:.2}s", t_final);
    loop {
        core::hint::spin_loop();
    }
}

/// Control loop entry on core1: produces samples.
fn control_core_entry<'d>(
    prod: Producer<'static, Sample>,
    periodic: PeriodicTimer<'d, Blocking>,
) -> ! {
    if USE_QSS1 {
        run_qss1(prod);
    } else {
        run_dtss(prod, periodic);
    }
}

/// Embassy task: consumes samples and prints csv lines over RTT.
#[embassy_executor::task]
async fn telemetry_task(mut cons: Consumer<'static, Sample>) {
    //Print header once
    rprintln!("time,q_w,q_x,q_y,q_z,w_x,w_y,w_z,rw1,rw2,rw3");

    // Use small stack string to format, no heap.
    use heapless::String;
    let mut line: String<192> = String::new();

    loop {
        if let Some(s) = cons.dequeue() {
            line.clear();
            use core::fmt::Write as _;
            let _ = write!(
                &mut line,
                "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                s.t,
                s.q[0],
                s.q[1],
                s.q[2],
                s.q[3],
                s.w[0],
                s.w[1],
                s.w[2],
                s.rw[0],
                s.rw[1],
                s.rw[2]
            );
            rprintln!("{}", line.as_str());
        } else {
            //avoid burning cpu.
            ETimer::after(EDuration::from_millis(2)).await;
        }
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1

    rtt_target::rtt_init_print!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut timg1 = TimerGroup::new(peripherals.TIMG1); // for core1 pacing
    let mut periodic = PeriodicTimer::new(timg1.timer0);

    let sic = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    // Init telemetry queue and split ends.
    let q = TELE_Q.init(Queue::new());
    let (prod, cons) = q.split();

    // Start scheduler on core0
    esp_rtos::start(timg0.timer0);

    // Initialize stack for core 1
    let stack = CORE1_STACK.init(Stack::new());

    //wait 3 secs
    ETimer::after(EDuration::from_secs(3)).await;

    // Start second core: control loop runs there.
    // start the timer at 20 ms
    periodic.start(HDuration::from_millis(20)).unwrap();
    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        sic.software_interrupt0,
        sic.software_interrupt1,
        stack,
        move || {
            control_core_entry(prod, periodic);
        },
    );

    rprintln!("Embassy initialized!");
    rprintln!("Core 1 started - running control loop");
    rprintln!("Core 0 - telemetry streaming via RTT");

    // Spawn Consumer : telemetry
    spawner.must_spawn(telemetry_task(cons));

    loop {
        ETimer::after(EDuration::from_secs(5)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
}
