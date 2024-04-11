use nalgebra::Vector2;
use nalgebra::Vector3;
use rand::Rng;
use serde_derive::Deserialize;
use serde_derive::Serialize;

pub trait Agent<State, Action> {
    fn get_goal_pose(&self) -> Vector3<f64>;
    
    fn get_state(&self) -> State;

    fn pose_from_state(&self, state: &State) -> Vector3<f64>;

    fn get_random_action(&self) -> Action;

    fn query(&self, state: &State, action: &Action, dt: f64) -> State;

    fn take(&mut self, action: &Action, dt: f64);

    fn cost(&self, action: &Action, rollout_iters: i32, dt: f64) -> f64 {
        let mut rollout_state = self.get_state();
        for _ in 0..rollout_iters {
            rollout_state = self.query(&rollout_state, action, dt);
        }

        (self.get_goal_pose() - self.pose_from_state(&rollout_state)).norm()
    }

    fn get_next_best_action(
        &self,
        control_iters: i32,
        rollout_iters: i32,
        dt: f64,
    ) -> (Action, f64) {
        let mut curr_best_action = self.get_random_action();
        let mut curr_best_cost = self.cost(&curr_best_action, rollout_iters, dt);
        for _ in 0..control_iters {
            let candidate_action = self.get_random_action();
            let candidate_cost = self.cost(&candidate_action, rollout_iters, dt);

            if candidate_cost < curr_best_cost {
                curr_best_action = candidate_action;
                curr_best_cost = candidate_cost;
            }
        }

        (curr_best_action, curr_best_cost)
    }

    fn get_pose(&self) -> Vector3<f64> {
        self.pose_from_state(&self.get_state())
    }

    fn goal_dist(&self) -> f64 {
        (self.get_pose() - self.get_goal_pose()).norm()
    }

    fn at_goal(&self, eps: f64) -> bool {
        self.goal_dist() <= eps
    }
}

#[derive(Serialize, Deserialize)]
pub struct DiffDriveAgent {
    state: Vector3<f64>, // x, y, theta
    goal: Vector3<f64>,  // x, y, theta
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
        Vector3::<f64>::new(
            state[0],
            state[1],
            state[2],
        )
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
