use crate::math::vector2::Vector2;

#[derive(Debug, Clone, PartialEq)]
pub struct RobotPose {
    pub joint_positions: Vec<Vector2>,
    pub end_effector: Vector2,
}

impl RobotPose {
    pub fn new(base_position: Vector2, joint_count: usize) -> Self {
        assert!(
            joint_count > 0,
            "Robot pose must contain at least one joint"
        );

        assert!(
            base_position.is_finite(),
            "Base position must be finite"
        );

        Self {
            joint_positions: vec![base_position; joint_count],
            end_effector: base_position,
        }
    }

    pub fn from_positions(positions: Vec<Vector2>) -> Self {
        assert!(
            !positions.is_empty(),
            "Robot pose must contain at least one joint"
        );

        assert!(
            positions.iter().all(|position| position.is_finite()),
            "Robot pose positions must be finite"
        );

        let end_effector = *positions
            .last()
            .expect("Robot pose positions cannot be empty");

        Self {
            joint_positions: positions,
            end_effector,
        }
    }

    pub fn joint_count(&self) -> usize {
        self.joint_positions.len()
    }

    pub fn joint_position(&self, index: usize) -> Option<Vector2> {
        self.joint_positions.get(index).copied()
    }

    pub fn end_effector(&self) -> Vector2 {
        self.end_effector
    }

    pub fn set_joint_position(
        &mut self,
        index: usize,
        position: Vector2,
    ) -> bool {
        if !position.is_finite() {
            return false;
        }

        if let Some(joint_position) =
            self.joint_positions.get_mut(index)
        {
            *joint_position = position;

            if index + 1 == self.joint_positions.len() {
                self.end_effector = position;
            }

            true
        } else {
            false
        }
    }

    pub fn set_positions(
        &mut self,
        positions: &[Vector2],
    ) -> bool {
        if positions.len() != self.joint_positions.len()
            || !positions.iter().all(|position| position.is_finite())
        {
            return false;
        }

        self.joint_positions.copy_from_slice(positions);

        if let Some(end_effector) = positions.last().copied() {
            self.end_effector = end_effector;
        }

        true
    }

    pub fn set_end_effector(&mut self, position: Vector2) {
        if !position.is_finite() {
            return;
        }

        self.end_effector = position;

        if let Some(last) = self.joint_positions.last_mut() {
            *last = position;
        }
    }

    pub fn reset(&mut self, base_position: Vector2) {
        if !base_position.is_finite() {
            return;
        }

        self.joint_positions.fill(base_position);
        self.end_effector = base_position;
    }

    pub fn is_finite(&self) -> bool {
        self.joint_positions
            .iter()
            .all(|position| position.is_finite())
            && self.end_effector.is_finite()
    }
}

impl Default for RobotPose {
    fn default() -> Self {
        Self {
            joint_positions: vec![Vector2::ZERO],
            end_effector: Vector2::ZERO,
        }
    }
}