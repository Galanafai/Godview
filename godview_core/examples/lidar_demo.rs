//! LiDAR and 3D Assets Demo
//! ==========================
//!
//! Demonstrates:
//! - LiDAR point cloud visualization (height-colored)
//! - 3D detection bounding boxes with class colors
//! - Simulated sensor data
//!
//! Run:
//! ```bash
//! cargo run --example lidar_demo --features visualization -- --save lidar_demo.rrd
//! ```

use godview_core::visualization::RerunVisualizer;
use rand::Rng;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🔦 LiDAR and 3D Assets Demo");
    println!("============================\n");

    // Create visualizer
    let args: Vec<String> = std::env::args().collect();
    let viz = if args.len() > 1 && args[1] == "--save" {
        let path = args.get(2).map(|s| s.as_str()).unwrap_or("lidar_demo.rrd");
        println!("📹 Saving to: {}\n", path);
        RerunVisualizer::new_to_file("LiDAR Demo", path)?
    } else {
        println!("📹 Opening Rerun viewer...\n");
        RerunVisualizer::new("LiDAR Demo")?
    };

    // Ground plane
    viz.log_ground_plane(100.0, 20)?;

    // Create a simulated scene
    let mut rng = rand::thread_rng();
    
    // Simulate 60 frames (2 seconds at 30fps)
    println!("▶️  Generating LiDAR frames...\n");
    
    for frame in 0..60 {
        viz.set_time("frame", frame as u64);
        let t = frame as f64 / 30.0;
        
        // Generate simulated LiDAR point cloud
        // Simulate a 64-beam LiDAR scanning in a circle
        let mut points: Vec<[f32; 3]> = Vec::new();
        let mut intensities: Vec<f32> = Vec::new();
        
        // Ground plane points
        for i in 0..1000 {
            let angle = (i as f32 / 1000.0) * std::f32::consts::TAU;
            let dist = rng.gen_range(5.0..50.0);
            let x = angle.cos() * dist;
            let y = angle.sin() * dist;
            let z = rng.gen_range(-0.2..0.1); // Ground with noise
            
            points.push([x, y, z]);
            intensities.push(rng.gen_range(0.1..0.3)); // Low intensity for ground
        }
        
        // Car 1 - stationary
        add_car_points(&mut points, &mut intensities, [15.0, 10.0, 0.0], &mut rng);
        
        // Car 2 - moving
        let car2_x = 20.0 * (t * 0.5).cos();
        let car2_y = 20.0 * (t * 0.5).sin();
        add_car_points(&mut points, &mut intensities, [car2_x as f32, car2_y as f32, 0.0], &mut rng);
        
        // Pedestrians
        let ped_x = -5.0 + t as f32 * 2.0;
        add_pedestrian_points(&mut points, &mut intensities, [ped_x, -10.0, 0.0], &mut rng);
        
        // Log the point cloud (colored by height)
        viz.log_lidar_pointcloud("world/lidar", &points, Some(&intensities), false)?;
        
        // Log 3D detection boxes
        viz.log_class_bbox("car", "car_1", [15.0, 10.0, 0.8], [4.5, 2.0, 1.6], 0.0, 0.95)?;
        viz.log_class_bbox("car", "car_2", [car2_x, car2_y, 0.8], [4.5, 2.0, 1.6], t as f32 * 0.5 + std::f32::consts::FRAC_PI_2, 0.88)?;
        viz.log_class_bbox("pedestrian", "ped_1", [ped_x as f64, -10.0, 0.9], [0.6, 0.6, 1.8], 0.0, 0.75)?;
        
        // Log ego vehicle position (sensor origin)
        viz.log_agent("EGO_VEHICLE", [0.0, 0.0, 1.5], [4.5, 2.0, 1.6], [0, 255, 200, 255], false)?;
        
        if frame % 20 == 0 {
            println!("  Frame {}/60 - {} points", frame + 1, points.len());
        }
    }

    println!("\n✅ Done!");
    println!("\n📖 What you should see:");
    println!("   • Colored point cloud (height-based coloring)");
    println!("   • Cyan box: EGO vehicle (sensor origin)");
    println!("   • Cyan boxes: Detected cars");
    println!("   • Orange box: Detected pedestrian");
    println!("   • Points are brighter for objects, darker for ground");

    Ok(())
}

/// Add simulated LiDAR points for a car shape
fn add_car_points(points: &mut Vec<[f32; 3]>, intensities: &mut Vec<f32>, center: [f32; 3], rng: &mut impl Rng) {
    let (cx, cy, cz) = (center[0], center[1], center[2]);
    
    // Car dimensions
    let length = 4.5;
    let width = 2.0;
    let height = 1.6;
    
    // Generate points on car surface
    for _ in 0..200 {
        let side = rng.gen_range(0..6);
        let (x, y, z) = match side {
            0 => (cx + rng.gen_range(-length/2.0..length/2.0), cy + width/2.0, cz + rng.gen_range(0.0..height)), // Right
            1 => (cx + rng.gen_range(-length/2.0..length/2.0), cy - width/2.0, cz + rng.gen_range(0.0..height)), // Left
            2 => (cx + length/2.0, cy + rng.gen_range(-width/2.0..width/2.0), cz + rng.gen_range(0.0..height)), // Front
            3 => (cx - length/2.0, cy + rng.gen_range(-width/2.0..width/2.0), cz + rng.gen_range(0.0..height)), // Back
            4 => (cx + rng.gen_range(-length/2.0..length/2.0), cy + rng.gen_range(-width/2.0..width/2.0), cz + height), // Top
            _ => (cx + rng.gen_range(-length/2.0..length/2.0), cy + rng.gen_range(-width/2.0..width/2.0), cz), // Bottom
        };
        
        points.push([x, y, z]);
        intensities.push(rng.gen_range(0.7..1.0)); // High intensity for objects
    }
}

/// Add simulated LiDAR points for a pedestrian
fn add_pedestrian_points(points: &mut Vec<[f32; 3]>, intensities: &mut Vec<f32>, center: [f32; 3], rng: &mut impl Rng) {
    let (cx, cy, cz) = (center[0], center[1], center[2]);
    
    // Pedestrian as vertical cylinder
    for _ in 0..50 {
        let angle = rng.gen_range(0.0..std::f32::consts::TAU);
        let radius = 0.3;
        let z = cz + rng.gen_range(0.0..1.8);
        
        points.push([
            cx + angle.cos() * radius,
            cy + angle.sin() * radius,
            z,
        ]);
        intensities.push(rng.gen_range(0.5..0.8));
    }
}
