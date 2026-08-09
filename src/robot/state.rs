#[derive(Debug, Clone)]
pub struct RobotState {
    pub joint_angles: Vec<f32>,
    pub current_error: f32,
    pub iteration: usize,
    pub converged: bool,
    pub singular: bool,
    pub near_singular: bool,
    pub is_singular_now: bool,
    pub manipulability: f32,
    pub unreachable: bool,
    pub stalled: bool,
    pub constraint_blocked: bool,
    pub colliding: bool,
}

impl RobotState {
    pub fn new(joint_count: usize) -> Self {
        Self {
            joint_angles: vec![0.0; joint_count],
            current_error: 0.0,
            iteration: 0,
            converged: false,
            singular: false,
            near_singular: false,
            is_singular_now: false,
            manipulability: 0.0,
            unreachable: false,
            stalled: false,
            constraint_blocked: false,
            colliding: false,
        }
    }

    pub fn reset(&mut self) {
        self.joint_angles.fill(0.0);
        self.clear_solver_status();
    }

    pub fn joint_count(&self) -> usize {
        self.joint_angles.len()
    }

    pub fn angle(&self, index: usize) -> Option<f32> {
        self.joint_angles.get(index).copied()
    }

    pub fn set_angle(&mut self, index: usize, angle: f32) -> bool {
        if !angle.is_finite() {
            return false;
        }

        if let Some(current_angle) = self.joint_angles.get_mut(index) {
            *current_angle = angle;
            true
        } else {
            false
        }
    }

    pub fn set_angles(&mut self, angles: &[f32]) -> bool {
        if angles.len() != self.joint_angles.len()
            || !angles.iter().all(|angle| angle.is_finite())
        {
            return false;
        }

        self.joint_angles.copy_from_slice(angles);
        true
    }

    pub fn clear_solver_status(&mut self) {
        self.current_error = 0.0;
        self.iteration = 0;
        self.converged = false;
        self.singular = false;
        self.near_singular = false;
        self.is_singular_now = false;
        self.manipulability = 0.0;
        self.unreachable = false;
        self.stalled = false;
        self.constraint_blocked = false;
        self.colliding = false;
    }

    pub fn mark_converged(&mut self, error: f32, iteration: usize) {
        self.current_error = error.max(0.0);
        self.iteration = iteration;
        self.converged = true;
        self.singular = false;
        self.unreachable = false;
        self.stalled = false;
        self.constraint_blocked = false;
    }

    pub fn mark_singular(&mut self, error: f32, iteration: usize) {
        self.current_error = error.max(0.0);
        self.iteration = iteration;
        self.converged = false;
        self.singular = true;
    }

    pub fn mark_unreachable(&mut self, error: f32, iteration: usize) {
        self.current_error = error.max(0.0);
        self.iteration = iteration;
        self.converged = false;
        self.unreachable = true;
        self.constraint_blocked = false;
    }

    pub fn mark_stalled(&mut self, error: f32, iteration: usize) {
        self.current_error = error.max(0.0);
        self.iteration = iteration;
        self.converged = false;
        self.stalled = true;
    }

    pub fn set_constraint_blocked(&mut self, blocked: bool) {
        self.constraint_blocked = blocked;
    }

    pub fn is_valid(&self) -> bool {
        self.joint_angles.iter().all(|angle| angle.is_finite())
            && self.current_error.is_finite()
    }
}