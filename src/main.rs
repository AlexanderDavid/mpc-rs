use nalgebra::Vector3;

mod agent;

fn main() {
    static DT: f64 = 0.01 as f64;
    static GOAL_EPS: f64 = 0.1 as f64;

    let mut agents = Vec::new();
    let state = Vector3::<f64>::new(0.0, 0.0, 0.0); 
    let goal = Vector3::<f64>::new(1.0, 0.0, 0.0); 
    let agent = agent::DiffDriveAgent::new(state, goal);
    agents.push(agent);

    while !agents.iter().fold(false, |acc, x| acc || x.at_goal(GOAL_EPS)) {
        println!("step: {}", agents[0]);
        agents.iter_mut().for_each(|x| x.take(&x.get_next_best_action(100, DT).0, DT));
    }
    println!("after: {}", agents[0]);
}
