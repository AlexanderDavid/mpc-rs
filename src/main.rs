use crate::scene::Scene;

mod scene;
mod agent;

fn main() {
    let mut scene = match Scene::load(&"scenes/simple.toml".to_string()) {
        Some(s) => s,
        None => panic!()
    };

    scene.run();

    if scene.save(&"scenes/simple_done.toml".to_string()) {
        println!("saved");
    } else {
        println!("error during saving")
    }
}
