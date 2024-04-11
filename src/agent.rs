use nalgebra::Vector3;

pub trait Agent<State, Action: std::fmt::Display> {
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

        println!("action: {}", curr_best_action);
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

