use vexide::prelude::*;

use std::time::Duration;
use evian::{
    control::loops::{AngularPid, Pid},
    drivetrain::model::Differential,
    motion::Seeking,
    prelude::*,
};

use evian::tracking::RotarySensor;
use vexide::{
    devices::PortError,
    prelude::{AdiEncoder, AdiPort, Direction, Position},
};

type Amt102V = AdiEncoder<8192>;

const LINEAR_PID: Pid = Pid::new(1.0, 0.0, 0.125, None);
const ANGULAR_PID: AngularPid = AngularPid::new(16.0, 0.0, 1.0, None);
const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
    .error(4.0)
    .velocity(0.25)
    .duration(Duration::from_millis(15));
const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
    .error(f64::to_radians(8.0))
    .velocity(0.09)
    .duration(Duration::from_millis(15));

struct Robot {
    drivetrain: Drivetrain<Differential, WheeledTracking>,
    controller: Controller,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let dt = &mut self.drivetrain;
        let mut seeking = Seeking {
            linear_controller: Pid::new(0.0, 0.0, 0.0, None),
            lateral_controller: Pid::new(0.0, 0.0, 0.0, None),
            tolerances: LINEAR_TOLERANCES,
            timeout: Some(Duration::from_secs(10)),
        };

        seeking.move_to_point(dt, (0.0, 12.0)).await;
        // // Drive forwards at 60% speed.
        // basic
        //     .drive_distance(dt, 24.0)
        //     .with_linear_output_limit(6.0)
        //     .await;

        // // Turn to 0 degrees heading.
        // basic.turn_to_heading(dt, 0.0.deg()).await;

        // // Move to point (24, 24) on the field.

        // // Having fun with modifiers.
        // basic
        //     .drive_distance_at_heading(dt, 8.0, 45.0.deg())
        //     .with_linear_kd(1.2)
        //     .with_angular_tolerance_duration(Duration::from_millis(5))
        //     .with_angular_error_tolerance(f64::to_radians(10.0))
        //     .with_linear_error_tolerance(12.0)
        //     .await;
    }

    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();
            
            _ = self
                .drivetrain
                .model
                .drive_arcade(state.left_stick.y(), state.left_stick.x());
            println!("{}", self.drivetrain.tracking.position());

            sleep(Duration::from_millis(50)).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let forwards_enc = Amt102V::new(peripherals.adi_a, peripherals.adi_b);
    let sideways_enc = Amt102V::new(peripherals.adi_c, peripherals.adi_d);

    let mut imu = InertialSensor::new(peripherals.port_15);
    imu.calibrate().await.unwrap();

    let mut drivetrain = Drivetrain::new(
        Differential::new(
            [
                Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
            ],
            [
                Motor::new(peripherals.port_17, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
                Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_20, Gearset::Blue, Direction::Forward),
            ],
        ),
        WheeledTracking::new(
            Vec2::default(),
            90.0.deg(),
            [TrackingWheel::new(forwards_enc, 2.0, 0.0, None)],
            [TrackingWheel::new(sideways_enc, 2.0, 0.0, None)],
            Some(imu),
        ),
    );

    let mut seeking = Seeking {
        linear_controller: Pid::new(0.08, 0.0, 0.0125, None),
        lateral_controller: Pid::new(0.05, 0.0, 0.0, None),
        tolerances: LINEAR_TOLERANCES,
        timeout: Some(Duration::from_secs(10)),
    };

    seeking.move_to_point(&mut drivetrain, (-6.0, 6.0)).await;

    Robot {
        drivetrain,
        controller: peripherals.primary_controller,
    }
    .compete()
    .await;
}
