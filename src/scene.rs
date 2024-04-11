use std::{
    fs::{File, OpenOptions},
    io::{Read, Write},
};

use serde::{Deserialize, Serialize};

use crate::agent::Agent;
// use crate::diff_drive::DiffDriveAgent;
use crate::accel_diff_drive::AccelDiffDriveAgent;

#[derive(Serialize, Deserialize)]
pub struct Scene {
    agents: Vec<AccelDiffDriveAgent>,
}

impl Scene {
    const DT: f64 = 0.01 as f64;
    const GOAL_EPS: f64 = 0.1 as f64;
    const CONTROL_SAMPLES: i32 = 10_000 as i32;
    const ROLLOUT_ITERS: i32 = 10 as i32;

    pub fn run_once(&mut self) {
        self.agents.iter_mut().for_each(|x| {
            x.take(
                &x.get_next_best_action(Self::CONTROL_SAMPLES, Self::ROLLOUT_ITERS, Self::DT)
                    .0,
                Self::DT,
            )
        });
    }

    pub fn done(&self) -> bool {
        self.agents
            .iter()
            .fold(false, |acc, x| acc || x.at_goal(Self::GOAL_EPS))
    }

    pub fn run(&mut self) {
        while !self.done() {
            self.run_once();
        }
    }

    pub fn load(filename: &String) -> Option<Scene> {
        let mut file = File::open(filename).ok()?;
        let mut contents = String::new();
        file.read_to_string(&mut contents).ok()?;

        Some(toml::from_str(&contents).unwrap())
    }

    pub fn save(&self, filename: &String) -> bool {
        let toml = toml::to_string(self).unwrap();

        let mut file = match OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(&format!("{}", filename))
        {
            Ok(file) => file,
            Err(_) => {
                return false;
            }
        };

        match file.write_all(toml.as_str().as_bytes()) {
            Ok(_) => true,
            Err(_) => false,
        }
    }
}
