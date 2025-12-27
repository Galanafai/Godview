#!/usr/bin/env python3
"""
Three.js CARLA Bridge - WebSocket Server for 3D Web Visualization

Streams CARLA vehicle data over WebSocket to a Three.js frontend.
Simple, clean, and gives complete control over visualization.

Usage:
    # Terminal 1: Start CARLA  
    cd /data/CARLA_0.9.16 && ./CarlaUE4.sh -RenderOffScreen

    # Terminal 2: Run this server
    python3 threejs_bridge.py

    # Terminal 3: Open browser
    open http://localhost:8080
"""

import carla
import numpy as np
import time
import json
import math
import asyncio
import http.server
import socketserver
import threading
from pathlib import Path
from typing import Dict, List
import websockets

# =============================================================================
# HTTP SERVER (serves the Three.js HTML)
# =============================================================================

HTML_CONTENT = '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>GodView - CARLA Visualization</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            overflow: hidden;
        }
        #container { width: 100vw; height: 100vh; }
        
        #hud {
            position: absolute;
            top: 20px;
            left: 20px;
            color: #00ff88;
            font-size: 14px;
            z-index: 100;
            background: rgba(0,0,0,0.7);
            padding: 20px;
            border-radius: 12px;
            border: 1px solid #00ff8855;
            backdrop-filter: blur(10px);
            min-width: 280px;
        }
        #hud h1 { 
            font-size: 24px; 
            margin-bottom: 15px;
            background: linear-gradient(90deg, #00ff88, #00ccff);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        #hud .stat { 
            margin: 8px 0; 
            display: flex;
            justify-content: space-between;
        }
        #hud .label { color: #888; }
        #hud .value { color: #00ff88; font-weight: bold; }
        #hud .status { 
            margin-top: 15px;
            padding-top: 15px;
            border-top: 1px solid #333;
        }
        #hud .connected { color: #00ff88; }
        #hud .disconnected { color: #ff4444; }
        
        #legend {
            position: absolute;
            bottom: 20px;
            left: 20px;
            color: white;
            font-size: 12px;
            z-index: 100;
            background: rgba(0,0,0,0.7);
            padding: 15px;
            border-radius: 8px;
        }
        #legend h3 { margin-bottom: 10px; color: #00ff88; }
        .legend-item { display: flex; align-items: center; margin: 5px 0; }
        .legend-color { 
            width: 20px; height: 20px; 
            border-radius: 4px; 
            margin-right: 10px; 
        }
    </style>
</head>
<body>
    <div id="container"></div>
    
    <div id="hud">
        <h1>🚗 GodView</h1>
        <div class="stat">
            <span class="label">Vehicles:</span>
            <span class="value" id="vehicle-count">0</span>
        </div>
        <div class="stat">
            <span class="label">Avg Speed:</span>
            <span class="value" id="avg-speed">0 m/s</span>
        </div>
        <div class="stat">
            <span class="label">Max Speed:</span>
            <span class="value" id="max-speed">0 m/s</span>
        </div>
        <div class="stat">
            <span class="label">Frame:</span>
            <span class="value" id="frame-id">0</span>
        </div>
        <div class="stat">
            <span class="label">Sim Time:</span>
            <span class="value" id="sim-time">0.0s</span>
        </div>
        <div class="status">
            <span class="label">Status: </span>
            <span id="connection-status" class="disconnected">Connecting...</span>
        </div>
    </div>
    
    <div id="legend">
        <h3>Legend</h3>
        <div class="legend-item">
            <div class="legend-color" style="background: #4488ff;"></div>
            <span>Vehicle</span>
        </div>
        <div class="legend-item">
            <div class="legend-color" style="background: #ffaa00;"></div>
            <span>Velocity Vector</span>
        </div>
    </div>
    
    <script src="https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.160.0/examples/js/controls/OrbitControls.js"></script>
    
    <script>
        // Three.js Setup
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x1a1a2e);
        scene.fog = new THREE.Fog(0x1a1a2e, 100, 500);
        
        const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 2000);
        camera.position.set(50, 80, 100);
        camera.lookAt(0, 0, 0);
        
        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        renderer.shadowMap.enabled = true;
        document.getElementById('container').appendChild(renderer.domElement);
        
        // Controls
        const controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        controls.maxPolarAngle = Math.PI / 2.2;
        
        // Lighting
        const ambientLight = new THREE.AmbientLight(0x404040, 0.5);
        scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
        directionalLight.position.set(50, 100, 50);
        directionalLight.castShadow = true;
        scene.add(directionalLight);
        
        // Ground plane
        const groundGeometry = new THREE.PlaneGeometry(1000, 1000, 50, 50);
        const groundMaterial = new THREE.MeshStandardMaterial({ 
            color: 0x2a2a4e,
            roughness: 0.8,
            metalness: 0.2
        });
        const ground = new THREE.Mesh(groundGeometry, groundMaterial);
        ground.rotation.x = -Math.PI / 2;
        ground.receiveShadow = true;
        scene.add(ground);
        
        // Grid
        const gridHelper = new THREE.GridHelper(500, 50, 0x444466, 0x333355);
        scene.add(gridHelper);
        
        // Vehicle management
        const vehicles = new Map();
        
        function createVehicleMesh(id) {
            const group = new THREE.Group();
            
            // Car body
            const bodyGeometry = new THREE.BoxGeometry(4.5, 1.4, 2);
            const bodyMaterial = new THREE.MeshStandardMaterial({ 
                color: 0x4488ff,
                roughness: 0.3,
                metalness: 0.7,
                emissive: 0x112244,
                emissiveIntensity: 0.2
            });
            const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
            body.position.y = 0.7;
            body.castShadow = true;
            group.add(body);
            
            // Cabin (top)
            const cabinGeometry = new THREE.BoxGeometry(2.5, 1, 1.8);
            const cabinMaterial = new THREE.MeshStandardMaterial({ 
                color: 0x222244,
                roughness: 0.1,
                metalness: 0.9
            });
            const cabin = new THREE.Mesh(cabinGeometry, cabinMaterial);
            cabin.position.set(-0.3, 1.6, 0);
            cabin.castShadow = true;
            group.add(cabin);
            
            // Wheels
            const wheelGeometry = new THREE.CylinderGeometry(0.4, 0.4, 0.3, 16);
            const wheelMaterial = new THREE.MeshStandardMaterial({ color: 0x111111 });
            const wheelPositions = [
                [1.5, 0.4, 1.1], [1.5, 0.4, -1.1],
                [-1.5, 0.4, 1.1], [-1.5, 0.4, -1.1]
            ];
            wheelPositions.forEach(pos => {
                const wheel = new THREE.Mesh(wheelGeometry, wheelMaterial);
                wheel.rotation.x = Math.PI / 2;
                wheel.position.set(...pos);
                group.add(wheel);
            });
            
            // Velocity arrow
            const arrowDir = new THREE.Vector3(1, 0, 0);
            const arrowOrigin = new THREE.Vector3(0, 2, 0);
            const arrow = new THREE.ArrowHelper(arrowDir, arrowOrigin, 5, 0xffaa00, 1, 0.5);
            arrow.name = 'velocity_arrow';
            group.add(arrow);
            
            // Label
            // (Would add text sprite here in production)
            
            return group;
        }
        
        function updateVehicle(id, data) {
            let vehicle = vehicles.get(id);
            
            if (!vehicle) {
                vehicle = createVehicleMesh(id);
                scene.add(vehicle);
                vehicles.set(id, vehicle);
            }
            
            // Update position (swap y/z for Three.js coordinate system)
            vehicle.position.set(data.x, data.z + 0.5, -data.y);
            
            // Update rotation (yaw only for simplicity)
            vehicle.rotation.y = -THREE.MathUtils.degToRad(data.yaw);
            
            // Update velocity arrow
            const arrow = vehicle.getObjectByName('velocity_arrow');
            if (arrow) {
                const speed = Math.sqrt(data.vx*data.vx + data.vy*data.vy);
                if (speed > 0.5) {
                    arrow.visible = true;
                    arrow.setLength(Math.min(speed * 2, 15), 1, 0.5);
                    const velDir = new THREE.Vector3(data.vx, 0, -data.vy).normalize();
                    arrow.setDirection(velDir);
                } else {
                    arrow.visible = false;
                }
            }
        }
        
        // WebSocket Connection
        var ws = null;
        var isConnecting = false;
        
        function connect() {
            if (isConnecting) return;
            isConnecting = true;
            
            try {
                ws = new WebSocket('ws://localhost:8766');
            } catch (e) {
                console.error('WebSocket creation failed:', e);
                isConnecting = false;
                setTimeout(connect, 2000);
                return;
            }
            
            ws.onopen = function() {
                isConnecting = false;
                document.getElementById('connection-status').textContent = 'Connected';
                document.getElementById('connection-status').className = 'connected';
                console.log('WebSocket connected!');
            };
            
            ws.onclose = function() {
                isConnecting = false;
                document.getElementById('connection-status').textContent = 'Disconnected';
                document.getElementById('connection-status').className = 'disconnected';
                console.log('WebSocket closed, reconnecting...');
                setTimeout(connect, 2000);
            };
            
            ws.onerror = function(err) {
                console.error('WebSocket error:', err);
                isConnecting = false;
                if (ws && ws.readyState !== WebSocket.CLOSED) {
                    ws.close();
                }
            };
            
            ws.onmessage = function(event) {
                var data = JSON.parse(event.data);
                
                // Update HUD
                document.getElementById('vehicle-count').textContent = data.vehicle_count;
                document.getElementById('avg-speed').textContent = data.avg_speed.toFixed(1) + ' m/s';
                document.getElementById('max-speed').textContent = data.max_speed.toFixed(1) + ' m/s';
                document.getElementById('frame-id').textContent = data.frame_id;
                document.getElementById('sim-time').textContent = data.sim_time.toFixed(1) + 's';
                
                // Update vehicles
                var activeIds = new Set();
                data.vehicles.forEach(function(v) {
                    updateVehicle(v.id, v);
                    activeIds.add(v.id);
                });
                
                // Remove stale vehicles
                vehicles.forEach(function(mesh, id) {
                    if (!activeIds.has(id)) {
                        scene.remove(mesh);
                        vehicles.delete(id);
                    }
                });
                
                // Center camera on average position
                if (data.vehicles.length > 0) {
                    var avgX = 0, avgZ = 0;
                    data.vehicles.forEach(function(v) {
                        avgX += v.x;
                        avgZ += v.y;
                    });
                    avgX /= data.vehicles.length;
                    avgZ /= data.vehicles.length;
                    controls.target.lerp(new THREE.Vector3(avgX, 0, -avgZ), 0.02);
                }
            };
        }
        
        // Start connection
        connect();
        
        // Animation loop
        function animate() {
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        }
        animate();
        
        // Handle resize
        window.addEventListener('resize', () => {
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
        });
    </script>
</body>
</html>
'''


class ThreeJSCarlaBridge:
    """
    CARLA to Three.js bridge via WebSocket.
    
    - Connects to CARLA simulation
    - Runs HTTP server for Three.js page
    - Streams vehicle data via WebSocket
    """
    
    def __init__(
        self,
        carla_host: str = 'localhost',
        carla_port: int = 2000,
        http_port: int = 8080,
        ws_port: int = 8766,
    ):
        print("╔════════════════════════════════════════════════╗")
        print("║   THREE.JS CARLA BRIDGE                        ║")
        print("╚════════════════════════════════════════════════╝\n")
        
        # Connect to CARLA
        print(f"🔌 Connecting to CARLA at {carla_host}:{carla_port}...")
        self.client = carla.Client(carla_host, carla_port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        # Configure simulation
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 Hz
        settings.no_rendering_mode = True
        self.world.apply_settings(settings)
        
        print(f"✅ Connected to: {self.world.get_map().name}\n")
        
        self.http_port = http_port
        self.ws_port = ws_port
        self.frame_id = 0
        self.vehicles: Dict[int, carla.Actor] = {}
        self.ws_clients: set = set()
        
    def spawn_vehicles(self, count: int = 5):
        """Spawn test vehicles."""
        print(f"🚗 Spawning {count} vehicles...")
        
        # Clean existing
        for actor in self.world.get_actors().filter('vehicle.*'):
            actor.destroy()
        self.world.tick()
        
        bp_library = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()
        vehicle_bps = list(bp_library.filter('vehicle.*'))
        
        spawned = 0
        for i, sp in enumerate(spawn_points[:count*2]):
            if spawned >= count:
                break
            bp = vehicle_bps[i % len(vehicle_bps)]
            try:
                v = self.world.spawn_actor(bp, sp)
                v.set_autopilot(True)
                self.vehicles[v.id] = v
                spawned += 1
            except:
                pass
        
        self.world.tick()
        print(f"✅ Spawned {spawned} vehicles\n")
    
    def start_http_server(self):
        """Start HTTP server for Three.js page."""
        class Handler(http.server.SimpleHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.send_header('Content-type', 'text/html')
                self.end_headers()
                self.wfile.write(HTML_CONTENT.encode())
                
            def log_message(self, format, *args):
                pass  # Suppress logging
        
        def run_server():
            with socketserver.TCPServer(("", self.http_port), Handler) as httpd:
                httpd.serve_forever()
        
        thread = threading.Thread(target=run_server, daemon=True)
        thread.start()
        print(f"🌐 HTTP server: http://localhost:{self.http_port}")
    
    async def websocket_handler(self, websocket):
        """Handle WebSocket connection."""
        self.ws_clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.ws_clients.discard(websocket)
    
    async def broadcast(self, message: str):
        """Broadcast to all WebSocket clients."""
        if self.ws_clients:
            await asyncio.gather(
                *[client.send(message) for client in self.ws_clients],
                return_exceptions=True
            )
    
    def build_message(self, snapshot: carla.WorldSnapshot) -> dict:
        """Build vehicle data message."""
        vehicles_data = []
        speeds = []
        
        for actor_id, actor in self.vehicles.items():
            if not actor.is_alive:
                continue
            
            actor_snap = snapshot.find(actor_id)
            if not actor_snap:
                continue
            
            t = actor_snap.get_transform()
            v = actor_snap.get_velocity()
            speed = math.sqrt(v.x**2 + v.y**2)
            speeds.append(speed)
            
            vehicles_data.append({
                'id': actor_id,
                'x': t.location.x,
                'y': t.location.y,
                'z': t.location.z,
                'yaw': t.rotation.yaw,
                'vx': v.x,
                'vy': v.y,
                'vz': v.z,
                'speed': speed
            })
        
        return {
            'frame_id': self.frame_id,
            'sim_time': snapshot.timestamp.elapsed_seconds,
            'vehicle_count': len(vehicles_data),
            'avg_speed': np.mean(speeds) if speeds else 0,
            'max_speed': max(speeds) if speeds else 0,
            'vehicles': vehicles_data
        }
    
    async def run(self, duration: float = 120.0):
        """Run the bridge."""
        print(f"📡 WebSocket server: ws://localhost:{self.ws_port}")
        print()
        print("=" * 60)
        print("🌐 Open your browser to: http://localhost:{self.http_port}")
        print("=" * 60)
        print()
        
        # Start WebSocket server
        async with websockets.serve(self.websocket_handler, "0.0.0.0", self.ws_port):
            print("✅ WebSocket server started")
            
            start_time = time.time()
            last_print = start_time
            
            try:
                while time.time() - start_time < duration:
                    # Tick simulation
                    self.world.tick()
                    snapshot = self.world.get_snapshot()
                    
                    # Build and broadcast message
                    msg = self.build_message(snapshot)
                    await self.broadcast(json.dumps(msg))
                    
                    self.frame_id += 1
                    
                    # Print progress
                    if time.time() - last_print >= 2.0:
                        clients = len(self.ws_clients)
                        print(f"⏱️  Frame {self.frame_id} | "
                              f"Vehicles: {msg['vehicle_count']} | "
                              f"Speed: {msg['avg_speed']:.1f} m/s | "
                              f"Clients: {clients}")
                        last_print = time.time()
                    
                    await asyncio.sleep(0.04)  # ~25 FPS to browser
                    
            except KeyboardInterrupt:
                print("\n⚠️  Interrupted")
            finally:
                self._cleanup()
    
    def _cleanup(self):
        """Clean up resources."""
        print("\n🧹 Cleaning up...")
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        self.world.apply_settings(settings)
        print("✅ Done")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Three.js CARLA Bridge')
    parser.add_argument('--host', default='localhost', help='CARLA host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA port')
    parser.add_argument('--http-port', type=int, default=8080, help='HTTP server port')
    parser.add_argument('--ws-port', type=int, default=8766, help='WebSocket port')
    parser.add_argument('--vehicles', type=int, default=5, help='Number of vehicles')
    parser.add_argument('--duration', type=float, default=300.0, help='Duration in seconds')
    
    args = parser.parse_args()
    
    bridge = ThreeJSCarlaBridge(
        carla_host=args.host,
        carla_port=args.port,
        http_port=args.http_port,
        ws_port=args.ws_port,
    )
    
    bridge.spawn_vehicles(args.vehicles)
    bridge.start_http_server()
    
    asyncio.run(bridge.run(duration=args.duration))


if __name__ == '__main__':
    main()
