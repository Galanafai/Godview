Visualization Approaches for the GodView
Distributed Sensor Fusion Demo
To effectively demonstrate GodView – a multi-agent sensor fusion system with 6D Gaussian uncertainty
ellipsoids, cryptographically signed packets, and distributed track ID consensus (Highlander CRDT) – we
recommend three top visualization approaches. Each balances in-browser performance (leveraging
WebGL/GPU) and visual impact for an engaging LinkedIn-ready demo. We detail the tools, examples,
complexity, and suitability for real-time interactivity or video capture.
1. WebGL Geospatial Data Visualization (Deck.gl + Map Integration)
Approach: Use Deck.gl (Uber’s open-source WebGL data viz framework) layered on an interactive map
(Mapbox GL or MapLibre) to create a “God’s-eye” view. This approach excels at rendering multiple agents
and sensor data on a geographic plane, taking advantage of Deck.gl’s GPU acceleration and built-in
geospatial support . Agents can be depicted on a 3D map with their uncertainty ellipsoids and interagent communications.
Tools/Libraries: Deck.gl for high-performance layered visualization, with a basemap (Mapbox,
Google Maps, or MapLibre) for context . Deck.gl offers a variety of layers (e.g., ScatterplotLayer
for detections, TextLayer for IDs, etc.) and can integrate custom layers for unique needs . For
example, a MeshLayer or custom layer can render each agent’s covariance ellipsoid by scaling a
sphere geometry (visualizing the 6D Gaussian uncertainty volume). Deck.gl also supports H3 geoindexing, aligning with GodView’s use of H3 + altitude for spatial indexing.
Key Features to Demonstrate: Multiple agents’ uncertainty ellipsoids drawn as translucent 3D
volumes at the agents’ locations; dynamic fusion events using Covariance Intersection (e.g. two
ellipsoids merging into a new one to show fused uncertainty). You can animate the process by
updating layer data in real-time, since Deck.gl efficiently re-renders on data updates . Track ID
consensus can be illustrated by labels or color-coding – for instance, initially two agents tag the
same object with different IDs (different colored labels), then a consensus algorithm (Highlander
CRDT – “there can be only one” ID) resolves to a single ID label across agents. Cryptographic
signatures could be indicated via icon layers or color: e.g. a small lock icon next to data points, or
green/red highlights if a packet’s Ed25519 signature is verified or not.
Example Implementation: A relevant example is a flight simulation on a 3D map using Deck.gl:
Louis Yoong’s 2025 “3D Flights” demo shows 300+ moving airplanes (3D models) with altitude, heading,
speed on a globe, all rendered and animated in-browser . This demo uses Deck.gl’s
ScenegraphLayer to render GLTF models and updates their positions each frame, achieving smooth
motion and interactivity completely on the frontend . The GodView demo can build on similar
concepts – animating multiple moving agents and sensor readings in real time. Deck.gl’s architecture
is well-suited for streaming or animating data; it maps data to visual layers and handles efficient
redrawing of large datasets with WebGL2/WebGPU under the hood . The framework natively
1 2
•
3
4
•
5
•
6 7
7
2
1
supports interactive filtering, picking, and high-volume data animation (for example, animating
millions of points smoothly ).
Complexity & Suitability: Development complexity is moderate. Deck.gl is JavaScript (or Python
via PyDeck) and offers high-level APIs; composing standard layers requires minimal WebGL
knowledge. Custom behaviors (like an ellipsoid mesh or animated packet trails) might need writing a
custom layer or shader, leveraging Deck.gl’s extensibility . The learning curve for basic use is
gentle, and many examples/docs are available. For real-time interaction, this approach is excellent
– Deck.gl is designed for interactive performance, handling real-time data updates and user
interaction at 60 FPS on modern GPUs . It can easily accommodate an online demo in-browser (all
simulation can run client-side with dummy data or live feed) . For video capture, one can record
the screen or use Deck.gl’s ability to render offscreen. The visual style (map + 3D overlays) will
impress engineers by combining technical clarity (true geospatial coordinates, clear uncertainty
visuals) with an attractive, data-rich presentation. Overall, this approach yields an engaging “live
dashboard” feel – ideal for demonstrating distributed sensor fusion in context.
Sources: Deck.gl official docs emphasize its high-performance WebGL visualization of large data and
custom layer extensibility . The flight tracker example shows Deck.gl handling hundreds of moving
3D objects with smooth animation entirely in-browser , proving its capability for GodView’s real-time
simulation needs.
2. Game Engine in WebGL (Unity WebGL Export)
Approach: Utilize the Unity game engine to create a rich 3D simulation of the multi-agent system, then
export it as a WebGL build. Unity’s editor and physics can be leveraged to craft an immersive scene (for
instance, a simple 3D environment with vehicles or drones as agents) while showcasing sensor fusion
concepts with high visual fidelity. Unity is widely used in the autonomous vehicles industry for visualization
and simulation, making it a fitting choice to wow an engineering audience .
Tools/Libraries: Unity3D with its WebGL build target. Unity’s WebGL exporter compiles the game to
HTML5/WebAssembly, allowing it to run in-browser using WebGL for rendering . No external
library is needed on the web side beyond the generated files. Inside Unity, one can use built-in
features: the 3D rendering engine (with support for lighting, shadows, materials), physics (if
simulating realistic motion), and UI system for overlays (to display text like track IDs or packet info).
Unity’s asset ecosystem can provide 3D models (e.g., car or drone models for agents) and visual
effects. The Highlander CRDT consensus on track ID can be visualized via a Unity UI text that
updates when consensus is reached, and cryptographic verification could be indicated by icons (e.g.,
a lock symbol that appears on verified data points) using Unity’s Canvas.
Key Features to Demonstrate: Each autonomous agent could be a 3D model in the scene, moving
in a simulated space. Their uncertainty ellipsoids can be shown as semi-transparent ellipsoidal
meshes attached to them (Unity can scale a sphere primitive along axes to represent covariance
bounds). Covariance Intersection fusion might be illustrated by two agents both tracking a target:
initially, each target has its own uncertainty bubble; when fusion occurs, Unity could animate the two
ellipsoids merging into a single (perhaps smaller or re-oriented) ellipsoid to indicate the fused
estimate. Unity’s timeline or simple scripting can animate this process for clarity. Distributed track
ID consensus (Highlander): if two agents initially label the same object differently, Unity can show
8
•
4
5
7
2 4
6 7
9
•
10
•
2
two labels hovering over the object, then one label fading out when consensus (single ID) wins – a
visual storytelling of conflict resolution. Signed data packets: Unity can depict packet exchange
between agents using line renderers or particle effects (e.g., a glowing line or moving arrow
between agent objects whenever they communicate). The lines or packets can change color (red vs
green) depending on signature validity, or show a small padlock icon floating in the scene when a
packet is verified (leveraging World-space Canvas or TextMeshPro icons).
Example Implementation: BMW’s autonomous driving visualization is a real-world example of
Unity’s use: BMW’s teams use Unity to set up thousands of scenario simulations and visualize them
for testing ADAS/autonomous features . Unity enabled BMW to visualize and validate AV
scenarios at scale, demonstrating its strength in rendering complex automotive scenes. While BMW’s
is a proprietary setup, it underscores that Unity can handle advanced vehicle and sensor visuals. On
a smaller scale, the popular Udacity Self-Driving Car Simulator (and projects like LGSVL/Apollo
Simulator) were built with Unity, containing vehicles, sensor display (like LIDAR point clouds), and
real-time feedback – all of which were rendered with high fidelity (though those targeted native
execution). Unity WebGL, specifically, has been used for lighter-weight demos and games in-browser.
The Unity WebGL player runs the full Unity content in-browser , meaning our GodView
simulation can be experienced interactively on a webpage (desktop browsers). For instance, a Unity
WebGL demo could allow viewers to toggle views or step through the consensus process.
Complexity & Suitability: Development complexity is medium-high. Unity provides a visual editor
and a lot of functionality out-of-the-box, which speeds up creating 3D visuals and physics, but
implementing custom logic (e.g., Highlander CRDT behavior or Covariance Intersection math)
requires C# scripting. The build size and performance need consideration: Unity’s WebGL export will
produce a fairly large bundle (tens of MB), and it doesn’t run on mobile browsers . However, for a
LinkedIn video, that’s not an issue – one can screen-capture the Unity simulation at high resolution.
If an interactive web demo is desired, it’s feasible on desktop; just ensure to keep the scene
lightweight (few models, moderate poly counts) to maintain smooth WebGL performance. Unity’s
rendering quality can be a big plus for visual impact – you can add lighting, shadows, and even postprocessing for an appealing look. For real-time interaction, Unity WebGL can handle it on a capable
machine (real-time 3D up to moderate complexity), though it won’t match the raw efficiency of a
hand-optimized WebGL library. Still, as noted, major companies have used Unity to simulate complex
scenarios for autonomous driving because it allows rapid development of visuals and supports
large-scale scenario management . For video capture, Unity is well-suited – it even has an
official Recorder tool to capture in-engine footage, or simply play the scene and record using any
screen capture. The end result will look like a polished 3D animation illustrating the GodView
concepts. This approach scores high on impressiveness (realistic 3D rendering), making it great for
showcasing to engineers (and non-engineers) on LinkedIn, albeit with more development effort and
heavier runtime than pure web libraries.
Sources: Unity’s WebGL support allows deploying rich 3D content in-browser . The approach is validated
by industry use – e.g., “Unity allows developers to visualize and set up thousands of simulated scenarios to
validate performance under diverse conditions” in BMW’s autonomous driving program . This suggests
Unity can capably present our distributed fusion scenario with high fidelity.
•
9
10
•
11
9
10
9
3
3. Custom Browser 3D Engine (Babylon.js or Three.js)
Approach: Build a custom interactive 3D visualization using a browser-based 3D engine like Babylon.js (or
underlying Three.js). This approach sits between the other two: it’s code-centric like Deck.gl but offers the
full 3D flexibility of a game engine, all in the browser without bulky exports. Babylon.js is an open-source
WebGL engine known for its ease of use and performance optimizations, making it ideal for rendering
multiple agents, geometric ellipsoids, and real-time effects in a self-contained HTML/JS app. It’s essentially
using WebGL directly via a high-level API – achieving lightweight deployment and fine control.
Tools/Libraries: Babylon.js (or Three.js). Babylon.js provides a rich feature set (scene graph, camera
controls, materials, shadows, etc.) and has WebGPU support for future-proof performance. Three.js
is a lower-level library for WebGL; Babylon is built on similar concepts but with an all-in-one engine
approach (including a GUI system, physics, and loaders). We suggest Babylon for faster development
of interactive storytelling (it has a Viewer and Playground for quick prototyping, plus an extensive
toolkit for GUI, animations, etc.). You’d write JavaScript or TypeScript code to set up the scene. This
includes creating 3D objects for agents, uncertainty ellipsoids, and any visual indicators for packets
or consensus. Babylon’s engine runs fully in-browser (just include the JS library), aligning with the
requirement of self-contained HTML/JS and making it easy to export as a single web page or to
embed on a site.
Key Features to Demonstrate: Similar to Unity, each agent can be a 3D model or a simple shape in
the scene. Uncertainty ellipsoids can be rendered by scaling a sphere mesh (Babylon supports
dynamic scaling and also has tools like Lines/Edges if you want a wireframe ellipsoid representation).
These ellipsoids can be updated in size/orientation in real-time as the covariance evolves.
Covariance Intersection fusion: you can script an animation where two ellipsoid volumes gradually
overlap and form a new ellipsoid (demonstrating the fusion result). Babylon allows updating mesh
geometry or replacing one mesh with another over time, so an illustrative transition is achievable.
Distributed track ID (CRDT) consensus: could be shown by text labels or colored tags attached to
the object being tracked. Babylon.js has a GUI layer and 3D text capabilities; for instance, you might
have floating text above a target that initially reads “ID A” from agent1’s perspective and “ID B” from
agent2. Using a simple state machine, you can switch it to a single “ID C” (consensus ID) and maybe
add a brief visual effect (like two labels merging into one, or a highlight flash) to represent the
moment of consensus. Cryptographic signatures: can be indicated with visual icons or particle
effects. For example, when an agent receives a signed packet, the packet could be a small glowing
sphere traveling between agents; if signature is verified, the sphere could turn green or spawn a
“verified” checkmark above the receiving agent. Babylon’s particle system or custom shaders could
depict valid vs. tampered data (e.g., a red X particle burst if a bad signature is detected).
Example Implementation: The Babylon.js forum/community offers insight into similar projects.
One developer’s Automotive Sensor Visualization project (2021) used Babylon inside an Electron app to
display real-time computer vision sensor outputs for a vehicle . It visualized detections in 3D,
showing Babylon can handle incoming data and update visuals accordingly. In terms of raw
capability, Babylon can render thousands of objects efficiently – a Babylon team demo rendered
1000+ animated units at 60-70 FPS using instancing . While GodView might only need, say, a
dozen agents and a handful of ellipsoids/packets, this shows Babylon has headroom for smooth
real-time performance. Babylon.js is also used in digital twin and IoT dashboards where live sensor
data changes object states/colors in 3D (for example, changing machine indicator colors based on
•
•
•
12
13
4
data, etc.), highlighting its suitability for real-time data visualization on the web. The Babylon.js
documentation notes that it “makes it simple to create Digital Twins applications on the Web” and
visualize real-time IoT data by updating 3D object properties (materials, animations, etc.) as data
streams in . This directly aligns with updating agent states, uncertainties, and network
messages in our scenario.
Complexity & Suitability: Development complexity is moderate. You will be writing JavaScript/
TypeScript to define the scene and behaviors, which requires familiarity with 3D concepts but no
need to deal with WebGL at the shader level unless desired. Compared to Unity, there’s no visual
editor – all objects and animations must be coded or configured via scripts – but the flipside is a very
lightweight deployment (just a web page). Babylon.js has a gentle learning curve for those with
basic Three.js or graphics experience; even without, its documentation and playground examples
ease the process. For real-time interaction, this approach is excellent: everything runs in the
browser with no plugins, and Babylon is optimized for speed (it can leverage instancing, hardware
particles, and other GPU techniques to maintain performance ). The demo can run live in a
browser tab, or be wrapped in an Electron app if needed. It’s also straightforward to capture as a
video by running the simulation and recording it. One advantage is that because you control the
code, you can easily script camera motions or narrative sequences (e.g., fly the camera to each agent
to show its view, or slow-motion during a fusion event) to create a cinematic presentation for
LinkedIn. The visual quality can be high (Babylon supports PBR materials, shadows, etc.), though
achieving Unity-like realism might require more manual tweaking. Nonetheless, a crisp 3D
visualization with clear ellipsoids, lines, and labels can look highly professional and techy.
Impression on engineers: Seeing a custom-built simulation in a web page highlighting
cryptographic verification and consensus will certainly impress – it shows mastery of both the
domain and the tech. This approach offers a great balance of flexibility and performance, using
only web technologies.
Sources: Babylon.js’s capability to handle many objects and animate them is demonstrated by their
instancing demo (1000+ entities at high FPS) . Community projects confirm Babylon’s use in real-time
sensor visualization for vehicles . The official Babylon pitch for IoT/Digital Twins underlines its strength
in visualizing live data in 3D on the web, simplifying cross-platform deployment (just a browser) .
Each of these approaches can fulfill the five visualization requirements – (1) drawing Gaussian uncertainty
ellipsoids, (2) showing Covariance Intersection fusion, (3) narrating distributed track ID consensus, (4)
simulating real-time data traffic, (5) indicating cryptographic trust – but they offer different strengths:
Deck.gl + Map: Best for a geo-contextual view and data-driven animations. It’s relatively lightweight
and proven for real-time browser visualizations . Ideal if you want an interactive map-based
demo with clarity and minimal coding of low-level graphics.
Unity WebGL: Best for high-fidelity visuals and a game-like presentation. Great if you want realistic
3D models and possibly leverage Unity’s physics or existing assets to enhance the story. It requires
more setup and results in larger web builds, but it can produce a striking demo (or high-quality
recorded video) backed by an engine used in industry .
14 15
•
16 17
13
12
14
•
7
•
9
5
Babylon.js / Three.js: Best for custom-tailored interactivity in a pure web stack. Offers full control to
implement bespoke visuals (custom indicators for signatures, etc.) and is highly portable. It hits a
sweet spot of performance and flexibility, though it needs more coding than a specialized
framework. The result can run as a standalone HTML file, convenient for sharing or embedding (and
easy to record as well).
Recommendation: If the goal is to impress engineers with both technical depth and visual appeal, you
might even combine approaches: e.g., use Deck.gl for a global 2D/3D map overview and Babylon or Unity
for zoomed-in 3D sequences. However, within a limited timeline, choosing one of the above should suffice.
Deck.gl is a strong choice for an in-browser interactive dashboard feel , whereas Unity/Babylon shine
for scripted explainer visuals. All three approaches can be made LinkedIn-friendly (either as an interactive
link or more commonly as a compelling video clip). Consider your team’s familiarity: web developers might
prefer Deck.gl or Babylon (JavaScript), while those with game dev experience might get results faster in
Unity. Any of these top 3 approaches, done well, will vividly convey GodView’s innovative fusion of Time (EKF
timelines), Space (H3 geospatial data), Trust (crypto signatures), and Tracking (GNN+CI logic) – engaging your
audience with a clear, high-tech story of distributed sensor fusion.
Sources:
Deck.gl documentation – “deck.gl is designed to simplify high-performance, WebGPU/WebGL2 based
visualization of large data sets… Users can get impressive visual results with minimal effort by composing
layers… core classes are easily extendable for custom needs.”
Deck.gl showcase (Flight data demo) – Demonstrated 300+ moving 3D objects (flights) on a WebGL
globe with smooth animation, entirely in-browser: “dummy flight data — simulate moving airplanes on
a 3D map… Airplanes smoothly fly across the map… All simulated on the frontend. Everything runs fully in
the browser — no backend required.”
Unity WebGL official docs – “The WebGL build option allows Unity to publish content as JavaScript
programs which use HTML5/WebGL to run Unity content in a web browser.” Unity case study – “Unity
allows developers to visualize and set up thousands of simulated scenarios to validate performance under
diverse conditions.” (BMW using Unity for AV testing)
Babylon.js performance demo – “1000+ animated entities can be rendered simultaneously… proper use
of Babylon.js optimizations (instancing, etc.) can make a big difference… 343,000 instances running at
70fps.” Community project – “real-time computer vision project for vehicles and visualization is
powered by Babylon.JS… Communication of data via TCP, work in progress” (showing Babylon for live
sensor viz)
•
1
1.
2 4
2.
6 7
3.
10
9
4.
13 17
12
6
Introduction | deck.gl
https://deck.gl/docs
Showcase | deck.gl
https://deck.gl/showcase
Simulate 3D Flights on a Map with React, Deck.gl & Mapbox | by Louis Yoong | Medium
https://medium.com/@louisyoong/simulate-3d-flights-on-a-map-with-react-deck-gl-mapbox-e95ed6fef0e3
Realistic 3D Simulators for Automotive: A Review of Main Applications and Features - PMC
https://pmc.ncbi.nlm.nih.gov/articles/PMC11435838/
Unity - Manual: Getting started with WebGL development
https://docs.unity3d.com/560/Documentation/Manual/webgl-gettingstarted.html
Automotive Sensor Visualization - Demos and projects - Babylon.js
https://forum.babylonjs.com/t/automotive-sensor-visualization/17487
Creating thousands of animated entities in Babylon.js | by Babylon.js | Medium
https://babylonjs.medium.com/creating-thousands-of-animated-entities-in-babylon-js-ce3c439bdacf
Babylon.js Digital Twins and IoT
https://www.babylonjs.com/digitalTwinIot/
1 2 3 4
5 8
6 7
9
10 11
12
13 16 17
14 15
7