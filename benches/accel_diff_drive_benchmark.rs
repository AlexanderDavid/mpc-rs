use criterion::{criterion_group, criterion_main, Criterion};
use nalgebra::{Vector3, Vector5};

use mpc::agent::Agent;
use mpc::accel_diff_drive::AccelDiffDriveAgent;

pub fn criterion_benchmark(c: &mut Criterion) {
    let dd = AccelDiffDriveAgent {
        state: Vector5::<f64>::new(0.0, 0.0, 0.0, 0.0, 0.0),
        goal: Vector3::<f64>::new(1.0, 0.0, 0.0),
    };

    c.bench_function("add best action 10000 50 0.01", |b| {
        b.iter(|| dd.get_next_best_action(10000, 50, 0.01))
    });

    c.bench_function("add query random action", |b| {
        b.iter(|| dd.query(&dd.state, &dd.get_random_action(), 0.01))
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
