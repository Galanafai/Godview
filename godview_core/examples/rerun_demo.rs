//! GodView Synthetic Demo
//! 
//! Basic synthetic demo for testing visualization pipeline.
//! For the full demo with real data, see nuscenes_fusion_demo.rs

use godview_core::visualization::RerunVisualizer;
use nalgebra::Matrix6;
use uuid::Uuid;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🌐 GodView Synthetic Demo");
    
    let viz = RerunVisualizer::new("GodView Synthetic Demo")?;
    viz.log_ground_plane(100.0, 10)?;
    
    // Simple animation
    for frame in 0..100 {
        viz.set_time("frame", frame);
        
        let x = (frame as f64 * 0.1).sin() * 20.0;
        let y = (frame as f64 * 0.05).cos() * 10.0;
        
        viz.log_track(
            Uuid::nil(),
            [x, y, 1.0],
            [1.0, 0.5, 0.0],
            &(Matrix6::identity() * 0.5),
            "test_track",
        )?;
    }
    
    println!("✅ Done");
    Ok(())
}
