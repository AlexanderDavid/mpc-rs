use nalgebra::{Vector2, Vector3, Vector5};
use rand::Rng;
use serde::{Deserialize, Serialize};

use crate::agent::Agent;

#[derive(Serialize, Deserialize)]
pub struct AccelDiffDriveAgent {
    state: Vector5<f64>, // x, y, theta, l, w
    goal: Vector3<f64>,  // x, y, theta
}

impl AccelDiffDriveAgent {
    const ACTION_LB: Vector2<f64> = Vector2::<f64>::new(-0.1, -0.1);
    const ACTION_UB: Vector2<f64> = Vector2::<f64>::new(0.1, 0.1);

    const STATE_LB: Vector5<f64> =
        Vector5::<f64>::new(f64::INFINITY, f64::INFINITY, f64::INFINITY, -1.0, -1.0);
    const STATE_UB: Vector5<f64> =
        Vector5::<f64>::new(-f64::INFINITY, -f64::INFINITY, -f64::INFINITY, 1.0, 1.0);

    pub fn clamp_state(&self, state: &mut Vector5<f64>) {
        state[3] = nalgebra::clamp(state[3], Self::STATE_LB[3], Self::STATE_UB[3]);
        state[4] = nalgebra::clamp(state[4], Self::STATE_LB[4], Self::STATE_UB[4]);
    }
}

impl Agent<Vector5<f64>, Vector2<f64>> for AccelDiffDriveAgent {
    fn query(&self, state: &Vector5<f64>, action: &Vector2<f64>, dt: f64) -> Vector5<f64> {
        let mut potential_state = Vector5::<f64>::new(
            state[0] + state[3] * f64::cos(state[4]) * dt,
            state[1] + state[3] * f64::sin(state[4]) * dt,
            state[2] + state[4] * dt,
            state[3] + action[0] * dt,
            state[4] + action[1] * dt,
        );
        self.clamp_state(&mut potential_state);
        potential_state
    }

    fn take(&mut self, action: &Vector2<f64>, dt: f64) {
        self.state[0] += self.state[3] * f64::cos(self.state[4]) * dt;
        self.state[1] += self.state[3] * f64::sin(self.state[4]) * dt;
        self.state[2] += self.state[4] * dt;
        self.state[3] += action[0] * dt;
        self.state[4] += action[1] * dt;

        self.state[3] = nalgebra::clamp(self.state[3], Self::STATE_LB[3], Self::STATE_UB[3]);
        self.state[3] = nalgebra::clamp(self.state[3], Self::STATE_LB[3], Self::STATE_UB[3]);

        println!("state: {}", self.state);
    }

    fn get_goal_pose(&self) -> Vector3<f64> {
        self.goal
    }

    fn get_state(&self) -> Vector5<f64> {
        self.state
    }

    fn pose_from_state(&self, state: &Vector5<f64>) -> Vector3<f64> {
        Vector3::<f64>::new(state[0], state[1], state[2])
    }

    fn get_random_action(&self) -> Vector2<f64> {
        let mut rng = rand::thread_rng();

        Vector2::<f64>::new(
            rng.gen::<f64>()
                * (AccelDiffDriveAgent::ACTION_UB[0] - AccelDiffDriveAgent::ACTION_LB[0])
                + AccelDiffDriveAgent::ACTION_LB[0],
            rng.gen::<f64>()
                * (AccelDiffDriveAgent::ACTION_UB[1] - AccelDiffDriveAgent::ACTION_LB[1])
                + AccelDiffDriveAgent::ACTION_LB[1],
        )
    }
}
