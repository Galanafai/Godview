//! Simple Visual Demo - "See What's Happening"
//! =============================================
//!
//! A minimal demo showing clearly:
//! - 4 GIANT agents (impossible to miss)
//! - 5 moving cars
//! - Detection lines from each agent to what it sees
//! - Ghost tracks forming when multiple agents see the same car
//!
//! Run:
//! ```bash
//! cargo run --example simple_visual_demo --features visualization
//! ```

use godview_core::visualization::RerunVisualizer;
use nalgebra::Vector3;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🎬 Simple Visual Demo - See What's Happening!");
    println!("==============================================\n");

    // Create visualizer
    let args: Vec<String> = std::env::args().collect();
    let viz = if args.len() > 1 && args[1] == "--save" {
        let path = args.get(2).map(|s| s.as_str()).unwrap_or("simple_demo.rrd");
        println!("📹 Saving to: {}\n", path);
        RerunVisualizer::new_to_file("Simple Visual Demo", path)?
    } else {
        println!("📹 Opening Rerun viewer...\n");
        RerunVisualizer::new("Simple Visual Demo")?
    };

    // ========================================================================
    // SCENE SETUP - Simple grid, easy to understand
    // ========================================================================
    
    // Ground plane
    viz.log_ground_plane(100.0, 10)?;
    
    // 4 Agents at corners of a square (VERY VISIBLE)
    let agents = [
        ("ALPHA", [-30.0, -30.0, 10.0], [255, 100, 100, 255], 50.0),  // Red - bottom-left
        ("BETA",  [30.0, -30.0, 10.0],  [100, 150, 255, 255], 50.0),  // Blue - bottom-right
        ("GAMMA", [-30.0, 30.0, 10.0],  [100, 255, 150, 255], 50.0),  // Green - top-left
        ("DRONE", [0.0, 0.0, 40.0],     [255, 215, 0, 255],   80.0),  // Gold - center, high up
    ];
    
    // Log agents with sensor ranges (STATIC - logged once)
    for (name, pos, color, range) in &agents {
        viz.log_agent(name, *pos, [8.0, 8.0, 8.0], *color, name == &"DRONE")?;
        viz.log_sensor_range(name, *pos, *range, *color)?;
    }
    
    // 5 Cars - simple positions, will move in circles
    let cars = [
        ("Car_A", [0.0, 0.0]),      // Center
        ("Car_B", [15.0, 10.0]),    // Right of center
        ("Car_C", [-15.0, 10.0]),   // Left of center
        ("Car_D", [10.0, -15.0]),   // Below center right
        ("Car_E", [-10.0, -15.0]),  // Below center left
    ];
    
    println!("📍 Scene:");
    println!("   4 Agents at corners (Red/Blue/Green) + Gold Drone overhead");
    println!("   5 Cars moving in circles in the center\n");
    
    // ========================================================================
    // SIMULATION LOOP - 30 frames, 1 second of "real time"
    // ========================================================================
    
    println!("▶️  Running simulation...\n");
    
    for frame in 0..30 {
        viz.set_time("frame", frame as u64);
        
        let t = frame as f64 * 0.1; // Time in seconds
        
        // Move drone in a circle
        let drone_x = (t * 0.5).cos() * 20.0;
        let drone_y = (t * 0.5).sin() * 20.0;
        viz.log_agent("DRONE", [drone_x, drone_y, 40.0], [8.0, 8.0, 4.0], [255, 215, 0, 255], true)?;
        
        // Move each car and log detection lines
        for (car_name, base_pos) in &cars {
            // Car position (moves in small circle)
            let car_x = base_pos[0] + (t * 2.0).cos() * 5.0;
            let car_y = base_pos[1] + (t * 2.0).sin() * 5.0;
            let car_pos = [car_x, car_y, 1.0];
            
            // Log car as a white box (ground truth)
            viz.log_agent(&format!("CAR_{}", car_name), car_pos, [4.0, 2.0, 1.5], [255, 255, 255, 200], false)?;
            
            // Each agent "sees" the car and draws a detection line
            for (agent_name, agent_pos, color, range) in &agents {
                let dist = ((car_x - agent_pos[0]).powi(2) + (car_y - agent_pos[1]).powi(2)).sqrt();
                
                if dist <= *range as f64 {
                    // Agent can see the car - draw detection line
                    viz.log_detection_line(
                        &format!("{}_{}", agent_name, car_name),
                        *agent_pos,
                        car_pos,
                        *color,
                    )?;
                    
                    // Log the agent's noisy detection (colored dot near the car)
                    let noise = ((frame as f64 * 0.3).sin() * 2.0, (frame as f64 * 0.5).cos() * 2.0);
                    let detection_pos = [car_x + noise.0, car_y + noise.1, 1.0];
                    
                    viz.log_packet_detection(
                        &godview_core::GlobalHazardPacket {
                            entity_id: uuid::Uuid::new_v4(),
                            position: detection_pos,
                            velocity: [0.0, 0.0, 0.0],
                            class_id: 1,
                            timestamp: t,
                            confidence_score: 0.9,
                        },
                        1.5,
                    )?;
                }
            }
        }
        
        // Simple progress indicator
        if frame % 10 == 0 {
            println!("  Frame {}/30", frame + 1);
        }
    }
    
    println!("\n✅ Done!");
    println!("\n📖 What you should see:");
    println!("   • 4 big colored boxes at corners = Agents");
    println!("   • 5 white boxes moving in center = Cars (ground truth)");
    println!("   • Colored dots near cars = Agent detections (noisy!)");
    println!("   • Lines from agents to cars = Detection links");
    println!("   • Gold box flying circles = Drone");
    
    Ok(())
}
