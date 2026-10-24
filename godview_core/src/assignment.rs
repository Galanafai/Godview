//! Optimal one-to-one assignment (Kuhn-Munkres / Hungarian algorithm).
//!
//! Used for detection-to-track association within one agent and for scoring
//! estimates against ground truth. Replaces the per-packet greedy nearest
//! neighbor of the legacy tracker, which was order dependent and could give
//! two detections from one scan to the same track (audit finding F-12).

/// Minimum-cost one-to-one assignment between rows and columns.
///
/// Pairs whose cost is not finite or is `>= gate` are forbidden and never
/// returned. Among assignments, the solver first maximizes the number of
/// permitted pairs and then minimizes their total cost. Returns
/// `(row, column)` pairs sorted by row. Deterministic for identical input.
pub fn assign(cost: &[Vec<f64>], gate: f64) -> Vec<(usize, usize)> {
    let rows = cost.len();
    let cols = cost.first().map_or(0, |r| r.len());
    if rows == 0 || cols == 0 {
        return Vec::new();
    }
    let permitted = |c: f64| c.is_finite() && c < gate;
    let transpose = rows > cols;
    let (n, m) = if transpose {
        (cols, rows)
    } else {
        (rows, cols)
    };
    // Forbidden pairs cost more than any complete set of permitted pairs, so the
    // solver maximizes permitted pairs first. Keeping it proportional to the real
    // costs (rather than a huge constant) keeps the potentials well conditioned.
    let max_permitted = cost
        .iter()
        .flatten()
        .copied()
        .filter(|c| permitted(*c))
        .fold(0.0f64, |a, c| a.max(c.abs()));
    let forbidden = (max_permitted.max(1.0)) * (n as f64 + 1.0) * 4.0;
    let a = |i: usize, j: usize| {
        let c = if transpose { cost[j][i] } else { cost[i][j] };
        if permitted(c) {
            c
        } else {
            forbidden
        }
    };
    let matched = hungarian(n, m, a);
    let mut out: Vec<(usize, usize)> = matched
        .into_iter()
        .enumerate()
        .filter_map(|(i, j)| {
            let (r, c) = if transpose { (j, i) } else { (i, j) };
            permitted(cost[r][c]).then_some((r, c))
        })
        .collect();
    out.sort_unstable();
    out
}

/// Classic O(n² m) Hungarian algorithm with potentials for `n <= m`.
/// Returns the column assigned to each row.
fn hungarian(n: usize, m: usize, a: impl Fn(usize, usize) -> f64) -> Vec<usize> {
    let inf = f64::INFINITY;
    let mut u = vec![0.0; n + 1];
    let mut v = vec![0.0; m + 1];
    let mut p = vec![0usize; m + 1];
    let mut way = vec![0usize; m + 1];
    for i in 1..=n {
        p[0] = i;
        let mut j0 = 0;
        let mut minv = vec![inf; m + 1];
        let mut used = vec![false; m + 1];
        loop {
            used[j0] = true;
            let i0 = p[j0];
            let mut delta = inf;
            let mut j1 = 0;
            for j in 1..=m {
                if !used[j] {
                    let cur = a(i0 - 1, j - 1) - u[i0] - v[j];
                    if cur < minv[j] {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if minv[j] < delta {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for j in 0..=m {
                if used[j] {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
            if p[j0] == 0 {
                break;
            }
        }
        loop {
            let j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
            if j0 == 0 {
                break;
            }
        }
    }
    let mut ans = vec![0usize; n];
    for j in 1..=m {
        if p[j] != 0 {
            ans[p[j] - 1] = j - 1;
        }
    }
    ans
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{Rng, SeedableRng};

    fn brute_best(cost: &[Vec<f64>], gate: f64) -> (usize, f64) {
        // Enumerate all partial injections rows -> cols; maximize count then minimize cost.
        fn rec(
            i: usize,
            cost: &[Vec<f64>],
            gate: f64,
            used: &mut Vec<bool>,
            acc: (usize, f64),
            best: &mut (usize, f64),
        ) {
            if i == cost.len() {
                if acc.0 > best.0 || (acc.0 == best.0 && acc.1 < best.1 - 1e-12) {
                    *best = acc;
                }
                return;
            }
            rec(i + 1, cost, gate, used, acc, best);
            for j in 0..cost[i].len() {
                if !used[j] && cost[i][j] < gate {
                    used[j] = true;
                    rec(
                        i + 1,
                        cost,
                        gate,
                        used,
                        (acc.0 + 1, acc.1 + cost[i][j]),
                        best,
                    );
                    used[j] = false;
                }
            }
        }
        let mut best = (0, 0.0);
        rec(
            0,
            cost,
            gate,
            &mut vec![false; cost[0].len()],
            (0, 0.0),
            &mut best,
        );
        best
    }

    #[test]
    fn matches_brute_force_on_random_gated_problems() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(5);
        for _ in 0..400 {
            let n = rng.gen_range(1..6);
            let m = rng.gen_range(1..6);
            let cost: Vec<Vec<f64>> = (0..n)
                .map(|_| (0..m).map(|_| rng.gen_range(0.0..20.0)).collect())
                .collect();
            let gate = 11.0;
            let got = assign(&cost, gate);
            let mut rows = std::collections::BTreeSet::new();
            let mut cols = std::collections::BTreeSet::new();
            for (r, c) in &got {
                assert!(rows.insert(*r) && cols.insert(*c), "one-to-one");
                assert!(cost[*r][*c] < gate, "gated");
            }
            let total: f64 = got.iter().map(|(r, c)| cost[*r][*c]).sum();
            let (bn, bc) = brute_best(&cost, gate);
            assert_eq!(got.len(), bn, "maximum number of permitted pairs");
            assert!((total - bc).abs() < 1e-9, "minimum cost {total} vs {bc}");
        }
    }

    #[test]
    fn one_detection_cannot_feed_two_tracks() {
        // Two tracks both close to one detection: only one gets it.
        let cost = vec![vec![1.0], vec![1.5]];
        assert_eq!(assign(&cost, 10.0), vec![(0, 0)]);
    }

    #[test]
    fn handles_empty_and_non_finite() {
        assert!(assign(&[], 1.0).is_empty());
        let cost = vec![vec![f64::NAN, 2.0], vec![f64::INFINITY, 3.0]];
        assert_eq!(assign(&cost, 10.0), vec![(0, 1)]);
    }
}
