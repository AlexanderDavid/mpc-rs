use criterion::{criterion_group, criterion_main, Criterion};
use nalgebra::Vector3;

use mpc::agent::Agent;
use mpc::diff_drive::DiffDriveAgent;

pub fn criterion_benchmark(c: &mut Criterion) {
    let dd = DiffDriveAgent {
        state: Vector3::<f64>::new(0.0, 0.0, 0.0),
        goal: Vector3::<f64>::new(1.0, 0.0, 0.0),
    };

    c.bench_function("dd best action 10000 50 0.01", |b| {
        b.iter(|| dd.get_next_best_action(10000, 50, 0.01))
    });

    c.bench_function("dd query random action", |b| {
        b.iter(|| dd.query(&dd.state, &dd.get_random_action(), 0.01))
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
