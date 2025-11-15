🎮 Features
🔹 Real-Time Physics Simulation

Circle–circle collision

Box–box collision (AABB)

Circle–box collision

Elastic collisions with adjustable restitution

Basic friction & damping

Configurable gravity (1–5 keys adjust strength)

Fixed-timestep physics loop (up to 120Hz)

🔹 Interactive Object Creation

Left-click: create or drag a circle

Right-click: create a box

Drag objects: move them; velocity resets while dragging

DELETE: remove selected body

F: apply upward impulse to selected body

🔹 Camera & Controls

Scroll: Zoom in/out

Arrow keys: Move camera

SPACE: Toggle gravity

R: Reset world (floor recreated)

🔹 OpenGL Renderer

Custom shader pipeline (vertex + fragment)

On-CPU triangulation for circles

Screen-space camera transform to NDC

Dynamic VBO updates per frame

Clean immediate-mode-style drawing via shader batching

🔹 Clean C++ OOP Architecture

RigidBody class for circles & AABBs

PhysicsWorld handling integration + collision

Renderer for converting world coords → NDC and drawing shapes

Application class for full input + loop management
