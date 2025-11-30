pub use vexide_devices::math::Angle;

// MARK: Extension Trait

/// Extension trait for easily creating [`Angle`]s from floating-point
/// number literals.
pub trait IntoAngle {
    /// Creates an [`Angle`] of `self` degrees.
    fn deg(self) -> Angle;

    /// Creates an [`Angle`] of `self` gradians.
    fn grad(self) -> Angle;

    /// Creates an [`Angle`] of `self` radians.
    fn rad(self) -> Angle;

    /// Creates an [`Angle`] of `self` turns (revolutions).
    fn turns(self) -> Angle;
}

impl IntoAngle for f64 {
    fn deg(self) -> Angle {
        Angle::from_degrees(self)
    }

    fn rad(self) -> Angle {
        Angle::from_radians(self)
    }

    fn grad(self) -> Angle {
        Angle::from_gradians(self)
    }

    fn turns(self) -> Angle {
        Angle::from_turns(self)
    }
}
