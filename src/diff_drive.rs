use nalgebra::{Vector2, Vector3};
use rand::Rng;
use serde::{Deserialize, Serialize};

use crate::agent::Agent;

#[derive(Serialize, Deserialize)]
pub struct DiffDriveAgent {
    pub state: Vector3<f64>, // x, y, theta
    pub goal: Vector3<f64>,  // x, y, theta
}

impl DiffDriveAgent {
    const ACTION_LB: Vector2<f64> = Vector2::<f64>::new(-1.0, -1.0);
    const ACTION_UB: Vector2<f64> = Vector2::<f64>::new(1.0, 1.0);
}

impl Agent<Vector3<f64>, Vector2<f64>> for DiffDriveAgent {
    fn query(&self, state: &Vector3<f64>, action: &Vector2<f64>, dt: f64) -> Vector3<f64> {
        Vector3::<f64>::new(
            state[0] + action[0] * f64::cos(action[1]) * dt,
            state[1] + action[0] * f64::sin(action[1]) * dt,
            state[2] + action[1] * dt,
        )
    }

    fn take(&mut self, action: &Vector2<f64>, dt: f64) {
        self.state[0] += action[0] * f64::cos(action[1]) * dt;
        self.state[1] += action[0] * f64::sin(action[1]) * dt;
        self.state[2] += action[1] * dt;
    }

    fn get_goal_pose(&self) -> Vector3<f64> {
        self.goal
    }

    fn get_state(&self) -> Vector3<f64> {
        self.state
    }

    fn pose_from_state(&self, state: &Vector3<f64>) -> Vector3<f64> {
        Vector3::<f64>::new(state[0], state[1], state[2])
    }

    fn get_random_action(&self) -> Vector2<f64> {
        let mut rng = rand::thread_rng();

        Vector2::<f64>::new(
            rng.gen::<f64>() * (DiffDriveAgent::ACTION_UB[0] - DiffDriveAgent::ACTION_LB[0])
                + DiffDriveAgent::ACTION_LB[0],
            rng.gen::<f64>() * (DiffDriveAgent::ACTION_UB[1] - DiffDriveAgent::ACTION_LB[1])
                + DiffDriveAgent::ACTION_LB[1],
        )
    }
}
