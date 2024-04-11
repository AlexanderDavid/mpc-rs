use nalgebra::Vector2;
use nalgebra::Vector3;
use rand::Rng;
use serde_derive::Deserialize;
use serde_derive::Serialize;

#[derive(Serialize, Deserialize)]
pub struct DiffDriveAgent {
    state: Vector3<f64>, // x, y, theta
    goal: Vector3<f64>, // x, y, theta

    history: Vec<Vector3<f64>>
}

impl DiffDriveAgent {
    const ACTION_LB: Vector2<f64> = Vector2::<f64>::new(-1.0, -1.0);
    const ACTION_UB: Vector2<f64> = Vector2::<f64>::new(1.0, 1.0);

    fn goal_dist(&self) -> f64 {
        (self.state - self.goal).norm()
    }

    pub fn at_goal(&self, eps: f64) -> bool {
        self.goal_dist() <= eps
    }

    fn query(&self, state: &Vector3<f64>, action: &Vector2<f64>, dt: f64) -> Vector3<f64> {
        Vector3::<f64>::new(
            state[0] + action[0] * f64::cos(action[1]) * dt,
            state[1] + action[0] * f64::sin(action[1]) * dt,
            state[2] + action[1] * dt,
        )
    }

    fn cost(&self, action: &Vector2<f64>, rollout_iters: i32, dt: f64) -> f64 {
        let mut rollout_state = self.state;
        for _ in 0..rollout_iters {
            rollout_state = self.query(&rollout_state, action, dt);
        }

        (self.goal - rollout_state).norm()
    }

    pub fn get_next_best_action(
        &self,
        control_iters: i32,
        rollout_iters: i32,
        dt: f64,
    ) -> (Vector2<f64>, f64) {
        let mut curr_best_action = Vector2::<f64>::zeros();
        let mut curr_best_cost = f64::INFINITY;
        let mut rng = rand::thread_rng();
        for _ in 0..control_iters {
            let candidate_action = Vector2::<f64>::new(
                rng.gen::<f64>() * (DiffDriveAgent::ACTION_UB[0] - DiffDriveAgent::ACTION_LB[0])
                    + DiffDriveAgent::ACTION_LB[0],
                rng.gen::<f64>() * (DiffDriveAgent::ACTION_UB[1] - DiffDriveAgent::ACTION_LB[1])
                    + DiffDriveAgent::ACTION_LB[1],
            );
            let candidate_cost = self.cost(&candidate_action, rollout_iters, dt);

            if candidate_cost < curr_best_cost {
                curr_best_action = candidate_action;
                curr_best_cost = candidate_cost;
            }
        }

        (curr_best_action, curr_best_cost)
    }

    pub fn take(&mut self, action: &Vector2<f64>, dt: f64) {
        self.history.push(self.state);

        self.state[0] += action[0] * f64::cos(action[1]) * dt;
        self.state[1] += action[0] * f64::sin(action[1]) * dt;
        self.state[2] += action[1] * dt;
    }
}

impl std::fmt::Display for DiffDriveAgent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "(x: {}, y: {}, theta: {})",
            self.state[0], self.state[1], self.state[2]
        )
    }
}
