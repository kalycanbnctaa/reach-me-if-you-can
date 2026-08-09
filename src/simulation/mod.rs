use crate::{
    collision::{obstacle::Obstacle, potential_field::PotentialFieldConfig},
    kinematics::{
        inverse::{IkConfig, IkSolver},
        solver,
    },
    math::vector2::Vector2,
    robot::arm::RobotArm,
};

#[derive(Debug, Clone, Copy)]
pub struct SimulationConfig {
    pub enabled: bool,
    pub target: Vector2,
    pub paused: bool,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            target: Vector2::ZERO,
            paused: false,
        }
    }
}

pub struct Simulation {
    pub config: SimulationConfig,
}

impl Simulation {
    pub fn new() -> Self {
        Self {
            config: SimulationConfig::default(),
        }
    }

    pub fn update(
        &mut self,
        robot: &mut RobotArm,
        ik_solver: &dyn IkSolver,
        ik_config: &IkConfig,
        steps_per_frame: usize,
        obstacles: &[Obstacle],
        field_config: &PotentialFieldConfig,
    ) {
        if !self.config.enabled || self.config.paused {
            return;
        }

        for _ in 0..steps_per_frame.max(1) {
            if robot.state.converged || robot.state.unreachable || robot.state.stalled {
                break;
            }

            solver::step(
                robot,
                self.config.target,
                ik_solver,
                ik_config,
                obstacles,
                field_config,
            );
        }
    }

    pub fn set_target(&mut self, target: Vector2) {
        if target.is_finite() {
            self.config.target = target;
        }
    }

    pub fn target(&self) -> Vector2 {
        self.config.target
    }

    pub fn toggle_pause(&mut self) {
        self.config.paused = !self.config.paused;
    }

    pub fn set_paused(&mut self, paused: bool) {
        self.config.paused = paused;
    }

    pub fn reset(&mut self, robot: &mut RobotArm) {
        robot.reset();
        self.config.target = robot.end_effector();
        self.config.paused = false;
    }
}

impl Default for Simulation {
    fn default() -> Self {
        Self::new()
    }
}