//! GodView TUI Dashboard Module
//! ==============================
//!
//! Real-time terminal dashboard for monitoring fusion health metrics.
//! Uses Ratatui for rendering and Crossbeam for async metric delivery.
//!
//! Enable with the `dashboard` feature flag.
//!
//! Features:
//! - System health gauge (Healthy/Degraded/Critical)
//! - Active ghost count with threshold coloring
//! - Entropy reduction sparkline (last 100 values)
//! - Ghost watch table (sorted by ghost score)

use std::collections::VecDeque;
use std::io::{self, Stdout};
use std::time::Duration;

use crossbeam::channel::Receiver;
use crossterm::{
    event::{self, Event, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Gauge, Paragraph, Row, Sparkline, Table},
    Frame, Terminal,
};
use uuid::Uuid;

// =============================================================================
// METRIC PACKET (Sent from Fusion Engine to Dashboard)
// =============================================================================

/// Lightweight metric packet sent from the fusion engine to the TUI.
#[derive(Debug, Clone)]
pub struct MetricPacket {
    /// Current simulation timestamp
    pub timestamp: f64,
    /// Number of active tracks
    pub active_tracks: usize,
    /// Number of tracks with ghost score > 0.7
    pub active_ghosts: usize,
    /// Current entropy reduction rate (bits/step)
    pub entropy_reduction_rate: f64,
    /// Number of measurements with conflicting associations
    pub conflicting_associations: usize,
    /// Overall system health status
    pub system_status: SystemStatus,
}

/// System health classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemStatus {
    Healthy,
    Degraded,
    Critical,
}

impl Default for MetricPacket {
    fn default() -> Self {
        Self {
            timestamp: 0.0,
            active_tracks: 0,
            active_ghosts: 0,
            entropy_reduction_rate: 0.0,
            conflicting_associations: 0,
            system_status: SystemStatus::Healthy,
        }
    }
}

/// Ghost watch entry for the table widget
#[derive(Debug, Clone)]
pub struct GhostWatchEntry {
    pub track_id: Uuid,
    pub ghost_score: f64,
    pub nearest_neighbor: Option<Uuid>,
    pub velocity_delta: f64,
}

// =============================================================================
// FUSION DASHBOARD
// =============================================================================

/// TUI Dashboard for real-time fusion monitoring.
pub struct FusionDashboard {
    rx: Receiver<MetricPacket>,
    entropy_history: VecDeque<u64>,
    ghost_history: VecDeque<usize>,
    ghost_watch: Vec<GhostWatchEntry>,
    latest_packet: MetricPacket,
    frame_count: usize,
}

impl FusionDashboard {
    /// Create a new dashboard with the metric receiver channel.
    pub fn new(rx: Receiver<MetricPacket>) -> Self {
        Self {
            rx,
            entropy_history: VecDeque::with_capacity(100),
            ghost_history: VecDeque::with_capacity(100),
            ghost_watch: Vec::new(),
            latest_packet: MetricPacket::default(),
            frame_count: 0,
        }
    }

    /// Add a ghost watch entry (called from main loop)
    pub fn update_ghost_watch(&mut self, entries: Vec<GhostWatchEntry>) {
        self.ghost_watch = entries;
        // Sort by ghost score descending
        self.ghost_watch.sort_by(|a, b| {
            b.ghost_score.partial_cmp(&a.ghost_score).unwrap_or(std::cmp::Ordering::Equal)
        });
        // Keep top 10
        self.ghost_watch.truncate(10);
    }

    /// Run the TUI main loop (blocks until 'q' pressed)
    pub fn run(&mut self) -> io::Result<()> {
        // Setup terminal
        enable_raw_mode()?;
        let mut stdout = io::stdout();
        execute!(stdout, EnterAlternateScreen)?;
        let backend = CrosstermBackend::new(stdout);
        let mut terminal = Terminal::new(backend)?;

        loop {
            // Non-blocking receive of metrics
            while let Ok(packet) = self.rx.try_recv() {
                self.latest_packet = packet.clone();
                
                // Update history buffers
                let entropy_val = (packet.entropy_reduction_rate.abs() * 100.0) as u64;
                self.entropy_history.push_back(entropy_val);
                if self.entropy_history.len() > 100 {
                    self.entropy_history.pop_front();
                }
                
                self.ghost_history.push_back(packet.active_ghosts);
                if self.ghost_history.len() > 100 {
                    self.ghost_history.pop_front();
                }
            }

            // Draw UI
            terminal.draw(|f| self.ui(f))?;
            self.frame_count += 1;

            // Handle input (non-blocking with 50ms timeout)
            if event::poll(Duration::from_millis(50))? {
                if let Event::Key(key) = event::read()? {
                    if key.code == KeyCode::Char('q') || key.code == KeyCode::Esc {
                        break;
                    }
                }
            }
        }

        // Restore terminal
        disable_raw_mode()?;
        execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
        terminal.show_cursor()?;

        Ok(())
    }

    /// Render the UI
    fn ui(&self, f: &mut Frame) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .margin(1)
            .constraints([
                Constraint::Length(3),  // Header
                Constraint::Length(5),  // Health gauges
                Constraint::Length(6),  // Sparkline
                Constraint::Min(5),     // Ghost watch table
                Constraint::Length(1),  // Footer
            ])
            .split(f.area());

        // === HEADER ===
        let header = Paragraph::new(Line::from(vec![
            Span::styled("👁️ GodView Deep Inspection Dashboard", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw("  |  "),
            Span::styled(format!("t={:.2}s", self.latest_packet.timestamp), Style::default().fg(Color::Cyan)),
            Span::raw("  |  "),
            Span::raw(format!("Frame: {}", self.frame_count)),
        ]))
        .block(Block::default().borders(Borders::BOTTOM));
        f.render_widget(header, chunks[0]);

        // === HEALTH GAUGES ===
        let gauge_chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(33),
                Constraint::Percentage(34),
                Constraint::Percentage(33),
            ])
            .split(chunks[1]);

        // System Status
        let (status_text, status_color) = match self.latest_packet.system_status {
            SystemStatus::Healthy => ("HEALTHY", Color::Green),
            SystemStatus::Degraded => ("DEGRADED", Color::Yellow),
            SystemStatus::Critical => ("CRITICAL", Color::Red),
        };
        let status = Paragraph::new(format!("■ {}", status_text))
            .style(Style::default().fg(status_color).add_modifier(Modifier::BOLD))
            .block(Block::default().title("System").borders(Borders::ALL));
        f.render_widget(status, gauge_chunks[0]);

        // Ghost Count Gauge
        let ghost_ratio = (self.latest_packet.active_ghosts as f64 / 10.0).min(1.0);
        let ghost_color = if self.latest_packet.active_ghosts > 10 {
            Color::Red
        } else if self.latest_packet.active_ghosts > 5 {
            Color::Yellow
        } else {
            Color::Green
        };
        let ghost_gauge = Gauge::default()
            .block(Block::default().title("Active Ghosts").borders(Borders::ALL))
            .gauge_style(Style::default().fg(ghost_color))
            .percent((ghost_ratio * 100.0) as u16)
            .label(format!("{}/10", self.latest_packet.active_ghosts));
        f.render_widget(ghost_gauge, gauge_chunks[1]);

        // Track Count
        let tracks = Paragraph::new(format!("{} tracks", self.latest_packet.active_tracks))
            .style(Style::default().fg(Color::Cyan))
            .block(Block::default().title("Active Tracks").borders(Borders::ALL));
        f.render_widget(tracks, gauge_chunks[2]);

        // === ENTROPY SPARKLINE ===
        let entropy_data: Vec<u64> = self.entropy_history.iter().cloned().collect();
        let sparkline = Sparkline::default()
            .block(Block::default().title("Entropy Reduction Rate (last 100 steps)").borders(Borders::ALL))
            .data(&entropy_data)
            .style(Style::default().fg(Color::Cyan));
        f.render_widget(sparkline, chunks[2]);

        // === GHOST WATCH TABLE ===
        let header_cells = ["ID", "Score", "Nearest", "Δ Velocity"]
            .iter()
            .map(|h| Span::styled(*h, Style::default().add_modifier(Modifier::BOLD)));
        let header = Row::new(header_cells).height(1);

        let rows: Vec<Row> = self.ghost_watch.iter().map(|entry| {
            let score_color = if entry.ghost_score > 0.7 {
                Color::Red
            } else if entry.ghost_score > 0.3 {
                Color::Yellow
            } else {
                Color::Green
            };
            
            Row::new(vec![
                Span::raw(format!("{}", &entry.track_id.to_string()[..8])),
                Span::styled(format!("{:.2}", entry.ghost_score), Style::default().fg(score_color)),
                Span::raw(entry.nearest_neighbor.map(|n| n.to_string()[..8].to_string()).unwrap_or_else(|| "-".to_string())),
                Span::raw(format!("{:.1} m/s", entry.velocity_delta)),
            ])
        }).collect();

        let table = Table::new(
            rows,
            [
                Constraint::Length(10),
                Constraint::Length(8),
                Constraint::Length(10),
                Constraint::Length(12),
            ]
        )
        .header(header)
        .block(Block::default().title("👻 Ghost Watch (Top 10)").borders(Borders::ALL));
        f.render_widget(table, chunks[3]);

        // === FOOTER ===
        let footer = Paragraph::new("Press 'q' to quit")
            .style(Style::default().fg(Color::DarkGray));
        f.render_widget(footer, chunks[4]);
    }
}

// =============================================================================
// TESTS
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_metric_packet_default() {
        let packet = MetricPacket::default();
        assert_eq!(packet.active_tracks, 0);
        assert_eq!(packet.system_status, SystemStatus::Healthy);
    }

    #[test]
    fn test_ghost_watch_sorting() {
        let entries = vec![
            GhostWatchEntry {
                track_id: Uuid::new_v4(),
                ghost_score: 0.3,
                nearest_neighbor: None,
                velocity_delta: 1.0,
            },
            GhostWatchEntry {
                track_id: Uuid::new_v4(),
                ghost_score: 0.9,
                nearest_neighbor: None,
                velocity_delta: 2.0,
            },
        ];

        let (tx, rx) = crossbeam::channel::unbounded();
        let mut dashboard = FusionDashboard::new(rx);
        dashboard.update_ghost_watch(entries);

        assert_eq!(dashboard.ghost_watch[0].ghost_score, 0.9);
    }
}
