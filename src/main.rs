use crate::scene::Scene;

mod scene;
mod agent;
mod diff_drive;
mod accel_diff_drive;

fn main() {
    let filename: String = "scenes/simple.toml".to_string();
    let mut scene = match Scene::load(&filename) {
        Some(s) => s,
        None => panic!("Couldn't load {}", filename)
    };

    scene.run();

    scene.save(&"scenes/simple_done.toml".to_string());
}
