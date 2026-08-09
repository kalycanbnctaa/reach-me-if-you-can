use crate::{
    kinematics::{
        forward,
        jacobian::{self, Jacobian},
    },
    math::vector2::Vector2,
};

use super::{
    joint::Joint,
    limits::JointLimit,
    pose::RobotPose,
    segment::Segment,
    state::RobotState,
};

pub struct RobotArm {
    pub base_position: Vector2,
    pub joints: Vec<Joint>,
    pub segments: Vec<Segment>,
    pub limits: Vec<JointLimit>,
    pub state: RobotState,
    pub pose: RobotPose,
}

impl RobotArm {
    pub fn new(base_position: Vector2, lengths: Vec<f32>) -> Self {
        assert!(
            base_position.is_finite(),
            "Robot base position must be finite"
        );

        let segment_count = lengths.len();

        assert!(
            (2..=5).contains(&segment_count),
            "Robot arm must have between 2 and 5 segments"
        );

        assert!(
            lengths
                .iter()
                .all(|&length| length.is_finite() && length > 0.0),
            "All segment lengths must be finite and greater than zero"
        );

        let joints = (0..=segment_count)
            .map(Joint::new)
            .collect::<Vec<_>>();

        let segments = lengths
            .into_iter()
            .map(Segment::new)
            .collect::<Vec<_>>();

        let limits = (0..segment_count)
            .map(|_| JointLimit::unrestricted())
            .collect::<Vec<_>>();

        let state = RobotState::new(segment_count);

        let pose = forward::forward_kinematics(
            base_position,
            &segments
                .iter()
                .map(|segment| segment.length)
                .collect::<Vec<_>>(),
            &state.joint_angles,
        );

        Self {
            base_position,
            joints,
            segments,
            limits,
            state,
            pose,
        }
    }

    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    pub fn actuated_joint_count(&self) -> usize {
        self.segments.len()
    }

    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }

    pub fn total_length(&self) -> f32 {
        self.segments
            .iter()
            .map(|segment| segment.length)
            .sum()
    }

    pub fn longest_segment(&self) -> f32 {
        self.segments
            .iter()
            .map(|segment| segment.length)
            .fold(0.0, f32::max)
    }

    pub fn minimum_reach(&self) -> f32 {
        let total = self.total_length();
        let longest = self.longest_segment();

        (2.0 * longest - total).max(0.0)
    }

    pub fn maximum_reach(&self) -> f32 {
        self.total_length()
    }

    pub fn is_within_reach(&self, target: Vector2) -> bool {
        if !target.is_finite() {
            return false;
        }

        let distance =
            Vector2::distance(self.base_position, target);

        distance >= self.minimum_reach()
            && distance <= self.maximum_reach()
    }

    pub fn set_joint_angle(
        &mut self,
        index: usize,
        angle: f32,
    ) -> bool {
        let Some(limit) = self.limits.get(index).copied() else {
            return false;
        };

        if !angle.is_finite() {
            return false;
        }

        let clamped_angle = limit.clamp_normalized(angle);

        if !self.state.set_angle(index, clamped_angle) {
            return false;
        }

        self.update_pose();
        true
    }

    pub fn set_joint_angles(&mut self, angles: &[f32]) -> bool {
        if angles.len() != self.actuated_joint_count()
            || !angles.iter().all(|angle| angle.is_finite())
        {
            return false;
        }

        let clamped_angles: Vec<f32> = angles
            .iter()
            .enumerate()
            .map(|(index, &angle)| {
                self.limits[index].clamp_normalized(angle)
            })
            .collect();

        if !self.state.set_angles(&clamped_angles) {
            return false;
        }

        self.update_pose();
        true
    }

    pub fn joint_angle(&self, index: usize) -> Option<f32> {
        self.state.angle(index)
    }

    pub fn joint_limit(&self, index: usize) -> Option<JointLimit> {
        self.limits.get(index).copied()
    }

    pub fn set_joint_limit(&mut self, index: usize, limit: JointLimit) -> bool {
        let Some(current) = self.limits.get_mut(index) else {
            return false;
        };

        *current = limit;

        if let Some(angle) = self.state.angle(index) {
            let clamped = limit.clamp_normalized(angle);
            self.state.set_angle(index, clamped);
        }

        self.update_pose();
        true
    }

    pub fn end_effector(&self) -> Vector2 {
        self.pose.end_effector()
    }

    pub fn jacobian(&self) -> Jacobian {
        let lengths: Vec<f32> = self
            .segments
            .iter()
            .map(|segment| segment.length)
            .collect();

        jacobian::calculate(
            self.base_position,
            &lengths,
            &self.state.joint_angles,
        )
    }

    pub fn set_segment_length(&mut self, index: usize, length: f32) -> bool {
        if !length.is_finite() || length <= 0.0 {
            return false;
        }

        let Some(segment) = self.segments.get_mut(index) else {
            return false;
        };

        segment.length = length;
        self.update_pose();
        true
    }

    pub fn update_pose(&mut self) {
        forward::update_pose(self);
    }

    pub fn reset(&mut self) {
        self.state.reset();
        self.update_pose();
    }

    pub fn validate(&self) -> bool {
        self.base_position.is_finite()
            && self
                .segments
                .iter()
                .all(|segment| {
                    segment.length.is_finite()
                        && segment.length > 0.0
                })
            && self.state.is_valid()
            && self.pose.is_finite()
            && self.state.joint_count()
                == self.actuated_joint_count()
            && self.limits.len() == self.actuated_joint_count()
            && self.joints.len() == self.segment_count() + 1
            && self.pose.joint_count()
                == self.segment_count() + 1
    }
}