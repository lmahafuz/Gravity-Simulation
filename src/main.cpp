#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <vector>
#include <memory>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>

// ------------------ Simple 2D math ------------------
class Vec2 {
public:
    float x, y;
    Vec2() : x(0), y(0) {}
    Vec2(float _x, float _y) : x(_x), y(_y) {}
    Vec2 operator+(const Vec2 &o) const { return Vec2(x + o.x, y + o.y); }
    Vec2 operator-(const Vec2 &o) const { return Vec2(x - o.x, y - o.y); }
    Vec2 operator*(float s) const { return Vec2(x * s, y * s); }
    Vec2& operator+=(const Vec2 &o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2 &o) { x -= o.x; y -= o.y; return *this; }
    float length() const { return std::sqrt(x * x + y * y); }
    float lengthSq() const { return x * x + y * y; }
};

// ------------------ Rigid body ------------------
enum class ShapeType { Circle, Box };
class RigidBody {
public:
    ShapeType type;
    Vec2 pos;
    Vec2 vel;
    float radius; // for circle
    float hw, hh; // half-width/half-height for box
    float invMass; // 0 => static
    float restitution;
    float friction;
    float r, g, b; // color
    bool remove = false;

    RigidBody(ShapeType t, Vec2 p) : type(t), pos(p), vel(0, 0), radius(12), hw(16), hh(12),
        invMass(1.0f), restitution(0.8f), friction(0.2f), r(0.7f), g(0.7f), b(0.7f) {}
    bool isStatic() const { return invMass == 0.0f; }
};

// ------------------ Physics world ------------------
class PhysicsWorld {
public:
    std::vector<std::unique_ptr<RigidBody>> bodies;
    Vec2 gravity = Vec2(0.0f, -600.0f); // pixels per second^2
    bool gravityOn = true;
    float gravityScale = 1.0f; // for adjustable gravity strength

    void step(float dt) {
        // Integrate
        for (auto &b : bodies) {
            if (!b->isStatic()) {
                if (gravityOn) b->vel += gravity * gravityScale * dt;
                b->pos += b->vel * dt;
                b->vel = b->vel * 0.999f; // Simple damping
            }
        }
        // Collisions (O(n^2))
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                collide(*bodies[i], *bodies[j]);
            }
        }
        // Remove flagged bodies
        bodies.erase(std::remove_if(bodies.begin(), bodies.end(),
            [](const std::unique_ptr<RigidBody> &p) { return p->remove; }), bodies.end());
    }

    void clear() { bodies.clear(); }

private:
    void collide(RigidBody &A, RigidBody &B) {
        if (A.type == ShapeType::Circle && B.type == ShapeType::Circle) collideCircleCircle(A, B);
        else if (A.type == ShapeType::Box && B.type == ShapeType::Box) collideBoxBox(A, B);
        else if (A.type == ShapeType::Circle && B.type == ShapeType::Box) collideCircleBox(A, B);
        else if (A.type == ShapeType::Box && B.type == ShapeType::Circle) collideCircleBox(B, A);
    }

    void collideCircleCircle(RigidBody &a, RigidBody &b) {
        Vec2 d = b.pos - a.pos;
        float dist2 = d.lengthSq();
        float rsum = a.radius + b.radius;
        if (dist2 == 0.0f || dist2 >= rsum * rsum) return;
        float dist = std::sqrt(dist2);
        Vec2 n = d * (1.0f / dist);
        float penetration = rsum - dist;
        float totalInv = a.invMass + b.invMass;
        if (totalInv == 0) return;
        // Positional correction
        if (!a.isStatic()) a.pos -= n * (penetration * (a.invMass / totalInv));
        if (!b.isStatic()) b.pos += n * (penetration * (b.invMass / totalInv));
        // Velocity
        Vec2 rv = b.vel - a.vel;
        float relVel = rv.x * n.x + rv.y * n.y;
        if (relVel > 0) return;
        float e = std::min(a.restitution, b.restitution);
        float j = -(1 + e) * relVel / totalInv;
        Vec2 impulse = n * j;
        if (!a.isStatic()) a.vel -= impulse * a.invMass;
        if (!b.isStatic()) b.vel += impulse * b.invMass;
    }

    void collideBoxBox(RigidBody &a, RigidBody &b) {
        float ax1 = a.pos.x - a.hw, ax2 = a.pos.x + a.hw;
        float ay1 = a.pos.y - a.hh, ay2 = a.pos.y + a.hh;
        float bx1 = b.pos.x - b.hw, bx2 = b.pos.x + b.hw;
        float by1 = b.pos.y - b.hh, by2 = b.pos.y + b.hh;
        if (ax2 < bx1 || ax1 > bx2 || ay2 < by1 || ay1 > by2) return;
        float overlapX = std::min(ax2, bx2) - std::max(ax1, bx1);
        float overlapY = std::min(ay2, by2) - std::max(ay1, by1);
        Vec2 n;
        float penetration;
        if (overlapX < overlapY) {
            n = (a.pos.x < b.pos.x) ? Vec2(-1, 0) : Vec2(1, 0);
            penetration = overlapX;
        } else {
            n = (a.pos.y < b.pos.y) ? Vec2(0, -1) : Vec2(0, 1);
            penetration = overlapY;
        }
        float totalInv = a.invMass + b.invMass;
        if (totalInv == 0) return;
        if (!a.isStatic()) a.pos -= n * (penetration * (a.invMass / totalInv));
        if (!b.isStatic()) b.pos += n * (penetration * (b.invMass / totalInv));
        Vec2 rv = b.vel - a.vel;
        float relVel = rv.x * n.x + rv.y * n.y;
        if (relVel > 0) return;
        float e = std::min(a.restitution, b.restitution);
        float j = -(1 + e) * relVel / totalInv;
        Vec2 impulse = n * j;
        if (!a.isStatic()) a.vel -= impulse * a.invMass;
        if (!b.isStatic()) b.vel += impulse * b.invMass;
    }

    void collideCircleBox(RigidBody &c, RigidBody &b) {
        float closestX = std::clamp(c.pos.x, b.pos.x - b.hw, b.pos.x + b.hw);
        float closestY = std::clamp(c.pos.y, b.pos.y - b.hh, b.pos.y + b.hh);
        Vec2 d = Vec2(closestX - c.pos.x, closestY - c.pos.y);
        float dist2 = d.lengthSq();
        if (dist2 > c.radius * c.radius) return;
        float dist = dist2 == 0 ? 0 : std::sqrt(dist2);
        Vec2 n = (dist == 0) ? Vec2(0, 1) : d * (1.0f / dist);
        
        // Corrected positional correction:
        float penetration = c.radius - dist;
        float totalInv = c.invMass + b.invMass;
        if (totalInv == 0) return;
        if (!c.isStatic()) c.pos -= n * (penetration * (c.invMass / totalInv));
        if (!b.isStatic()) b.pos += n * (penetration * (b.invMass / totalInv));

        Vec2 rv = b.vel - c.vel;
        float relVel = rv.x * n.x + rv.y * n.y;
        if (relVel > 0) return;
        float e = std::min(c.restitution, b.restitution);
        float j = -(1 + e) * relVel / totalInv;
        Vec2 impulse = n * j;
        if (!c.isStatic()) c.vel -= impulse * c.invMass;
        if (!b.isStatic()) b.vel += impulse * b.invMass;
    }
};

// ------------------ Renderer ------------------
class Renderer {
public:
    int screenW = 800, screenH = 600;
    float camZoom = 1.0f; // pixels per world unit
    Vec2 camPos = Vec2(400.0f, 300.0f); // center in world coords
    GLuint vao = 0, vbo = 0, program = 0;

    bool init(int w, int h) {
        screenW = w;
        screenH = h;
        const char* vs = R"glsl(#version 330 core
layout(location=0) in vec2 aPos; // NDC
layout(location=1) in vec3 aColor;
out vec3 vColor;
void main() { gl_Position = vec4(aPos, 0.0, 1.0); vColor = aColor; }
)glsl";
        const char* fs = R"glsl(#version 330 core
in vec3 vColor;
out vec4 FragColor;
void main() { FragColor = vec4(vColor, 1.0); }
)glsl";
        GLuint vsId = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vsId, 1, &vs, nullptr);
        glCompileShader(vsId);
        GLint success;
        glGetShaderiv(vsId, GL_COMPILE_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetShaderInfoLog(vsId, 512, nullptr, infoLog);
            std::cerr << "Vertex shader compilation failed: " << infoLog << std::endl;
            return false;
        }
        GLuint fsId = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fsId, 1, &fs, nullptr);
        glCompileShader(fsId);
        glGetShaderiv(fsId, GL_COMPILE_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetShaderInfoLog(fsId, 512, nullptr, infoLog);
            std::cerr << "Fragment shader compilation failed: " << infoLog << std::endl;
            return false;
        }
        program = glCreateProgram();
        glAttachShader(program, vsId);
        glAttachShader(program, fsId);
        glLinkProgram(program);
        glGetProgramiv(program, GL_LINK_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetProgramInfoLog(program, 512, nullptr, infoLog);
            std::cerr << "Program linking failed: " << infoLog << std::endl;
            return false;
        }
        glDeleteShader(vsId);
        glDeleteShader(fsId);
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(2 * sizeof(float)));
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        return true;
    }

    void worldToNDC(float wx, float wy, float &ndcX, float &ndcY) {
        float px = (wx - camPos.x) * camZoom + screenW * 0.5f;
        float py = (wy - camPos.y) * camZoom + screenH * 0.5f;
        ndcX = (px / screenW) * 2.0f - 1.0f;
        ndcY = (py / screenH) * 2.0f - 1.0f;
    }

    void drawCircle(const Vec2 &pos, float radius, float rr, float gg, float bb) {
        const int seg = 24;
        std::vector<float> data;
        data.reserve((seg + 2) * 5);
        float cx, cy;
        worldToNDC(pos.x, pos.y, cx, cy);
        data.push_back(cx); data.push_back(cy); data.push_back(rr); data.push_back(gg); data.push_back(bb);
        for (int i = 0; i <= seg; ++i) {
            float a = (float)i / seg * 2.0f * 3.1415926f;
            float wx = pos.x + std::cos(a) * radius;
            float wy = pos.y + std::sin(a) * radius;
            float nx, ny;
            worldToNDC(wx, wy, nx, ny);
            data.push_back(nx); data.push_back(ny); data.push_back(rr); data.push_back(gg); data.push_back(bb);
        }
        renderArray(data, GL_TRIANGLE_FAN);
    }

    void drawBox(const Vec2 &pos, float hw, float hh, float rr, float gg, float bb) {
        std::vector<float> data = {
            pos.x - hw, pos.y - hh, rr, gg, bb,
            pos.x + hw, pos.y - hh, rr, gg, bb,
            pos.x + hw, pos.y + hh, rr, gg, bb,
            pos.x - hw, pos.y + hh, rr, gg, bb
        };
        for (size_t i = 0; i < data.size(); i += 5) {
            float nx, ny;
            worldToNDC(data[i], data[i + 1], nx, ny);
            data[i] = nx;
            data[i + 1] = ny;
        }
        std::vector<float> tris;
        tris.insert(tris.end(), data.begin(), data.begin() + 5);
        tris.insert(tris.end(), data.begin() + 5, data.begin() + 10);
        tris.insert(tris.end(), data.begin() + 10, data.begin() + 15);
        tris.insert(tris.end(), data.begin() + 10, data.begin() + 15);
        tris.insert(tris.end(), data.begin() + 15, data.begin() + 20);
        tris.insert(tris.end(), data.begin(), data.begin() + 5);
        renderArray(tris, GL_TRIANGLES);
    }

    void renderArray(const std::vector<float> &arr, GLenum mode) {
        if (arr.empty()) return;
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, arr.size() * sizeof(float), arr.data(), GL_DYNAMIC_DRAW);
        glUseProgram(program);
        glDrawArrays(mode, 0, (GLsizei)(arr.size() / 5));
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
};

// ------------------ Application ------------------
class Application {
public:
    GLFWwindow* window = nullptr;
    int width = 1000, height = 700;
    Renderer renderer;
    PhysicsWorld world;
    RigidBody* selected = nullptr;
    Vec2 dragOffset;
    std::mt19937 rng;
    double fixedTimeStep = 1.0 / 120.0;
    int maxSubSteps = 5;

    Application() { rng.seed(12345); }

    bool init() {
        if (!glfwInit()) {
            std::cerr << "Failed to initialize GLFW" << std::endl;
            return false;
        }
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        window = glfwCreateWindow(width, height, "Physics Sandbox", nullptr, nullptr);
        if (!window) {
            std::cerr << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return false;
        }
        glfwMakeContextCurrent(window);
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            std::cerr << "Failed to initialize GLAD" << std::endl;
            glfwTerminate();
            return false;
        }
        glViewport(0, 0, width, height);
        glfwSetWindowUserPointer(window, this);
        glfwSetFramebufferSizeCallback(window, [](GLFWwindow* w, int w_, int h_) {
            auto* app = static_cast<Application*>(glfwGetWindowUserPointer(w));
            app->onResize(w_, h_);
        });
        glfwSetMouseButtonCallback(window, [](GLFWwindow* w, int button, int action, int mods) {
            auto* app = static_cast<Application*>(glfwGetWindowUserPointer(w));
            app->onMouse(button, action, mods);
        });
        glfwSetCursorPosCallback(window, [](GLFWwindow* w, double x, double y) {
            auto* app = static_cast<Application*>(glfwGetWindowUserPointer(w));
            app->onMouseMove(x, y);
        });
        glfwSetScrollCallback(window, [](GLFWwindow* w, double x, double y) {
            auto* app = static_cast<Application*>(glfwGetWindowUserPointer(w));
            app->onScroll(x, y);
        });
        glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int scancode, int action, int mods) {
            auto* app = static_cast<Application*>(glfwGetWindowUserPointer(w));
            app->onKey(key, scancode, action, mods);
        });

        if (!renderer.init(width, height)) {
            std::cerr << "Failed to initialize renderer" << std::endl;
            glfwTerminate();
            return false;
        }
        auto floor = std::make_unique<RigidBody>(ShapeType::Box, Vec2(width * 0.5f, 20.0f));
        floor->hw = width * 0.5f;
        floor->hh = 20.0f;
        floor->invMass = 0.0f;
        floor->r = 1.0f;
        floor->g = 1.0f;
        floor->b = 1.0f;
        world.bodies.push_back(std::move(floor));
        return true;
    }

    void onResize(int w, int h) {
        width = w;
        height = h;
        glViewport(0, 0, w, h);
        renderer.screenW = w;
        renderer.screenH = h;
    }

    Vec2 screenToWorld(double sx, double sy) {
        float nx = (float)sx;
        float ny = (float)(renderer.screenH - sy);
        float wx = (nx - renderer.screenW * 0.5f) / renderer.camZoom + renderer.camPos.x;
        float wy = (ny - renderer.screenH * 0.5f) / renderer.camZoom + renderer.camPos.y;
        return Vec2(wx, wy);
    }

    float randf(float a, float b) {
        std::uniform_real_distribution<float> dist(a, b);
        return dist(rng);
    }

    void onMouse(int button, int action, int mods) {
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);
        Vec2 wp = screenToWorld(mx, my);
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            // Select or create circle
            selected = pickBody(wp);
            if (!selected) {
                auto c = std::make_unique<RigidBody>(ShapeType::Circle, wp);
                c->radius = 20.0f;
                c->invMass = 1.0f;
                c->restitution = 0.95f;
                c->r = randf(0.2f, 1.0f);
                c->g = randf(0.2f, 1.0f);
                c->b = randf(0.2f, 1.0f);
                selected = c.get();
                dragOffset = Vec2(0, 0);
                world.bodies.push_back(std::move(c));
            } else {
                dragOffset = wp - selected->pos;
            }
        } else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
            // Create box
            auto b = std::make_unique<RigidBody>(ShapeType::Box, wp);
            b->hw = 20;
            b->hh = 14;
            b->invMass = 0.5f;
            b->restitution = 0.95f;
            b->r = randf(0.2f, 1.0f);
            b->g = randf(0.2f, 1.0f);
            b->b = randf(0.2f, 1.0f);
            selected = b.get();
            dragOffset = Vec2(0, 0);
            world.bodies.push_back(std::move(b));
        } else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
            selected = nullptr;
        }
    }

    void onMouseMove(double mx, double my) {
        if (selected) {
            Vec2 wp = screenToWorld(mx, my);
            selected->pos = wp - dragOffset;
            selected->vel = Vec2(0, 0);
        }
    }

    void onScroll(double xoff, double yoff) {
        float zoomFactor = (yoff > 0) ? 1.1f : 0.9f;
        renderer.camZoom *= zoomFactor;
        renderer.camZoom = std::clamp(renderer.camZoom, 0.2f, 4.0f);
    }

    void onKey(int key, int scancode, int action, int mods) {
        if (action != GLFW_PRESS) return;
        if (key == GLFW_KEY_SPACE) {
            world.gravityOn = !world.gravityOn;
        } else if (key == GLFW_KEY_R) {
            world.clear();
            auto floor = std::make_unique<RigidBody>(ShapeType::Box, Vec2(width * 0.5f, 20.0f));
            floor->hw = width * 0.5f;
            floor->hh = 20.0f;
            floor->invMass = 0.0f;
            floor->r = 1.0f;
            floor->g = 1.0f;
            floor->b = 1.0f;
            world.bodies.push_back(std::move(floor));
        } else if (key == GLFW_KEY_F && selected) {
            selected->vel += Vec2(0, 300.0f);
        } else if (key == GLFW_KEY_DELETE && selected) {
            selected->remove = true;
            selected = nullptr;
        } else if (key == GLFW_KEY_LEFT) {
            renderer.camPos.x -= 50.0f / renderer.camZoom;
        } else if (key == GLFW_KEY_RIGHT) {
            renderer.camPos.x += 50.0f / renderer.camZoom;
        } else if (key == GLFW_KEY_UP) {
            renderer.camPos.y += 50.0f / renderer.camZoom;
        } else if (key == GLFW_KEY_DOWN) {
            renderer.camPos.y -= 50.0f / renderer.camZoom;
        } else if (key >= GLFW_KEY_1 && key <= GLFW_KEY_5) {
            world.gravityScale = 0.2f * (key - GLFW_KEY_0);
        }
    }

    RigidBody* pickBody(const Vec2 &wp) {
        for (int i = (int)world.bodies.size() - 1; i >= 0; --i) {
            RigidBody* b = world.bodies[i].get();
            if (b->type == ShapeType::Circle) {
                float d2 = (b->pos - wp).lengthSq();
                if (d2 <= b->radius * b->radius) return b;
            } else {
                if (wp.x >= b->pos.x - b->hw && wp.x <= b->pos.x + b->hw &&
                    wp.y >= b->pos.y - b->hh && wp.y <= b->pos.y + b->hh) return b;
            }
        }
        return nullptr;
    }

    void run() {
        double last = glfwGetTime();
        double accumulator = 0.0;
        double fixedTimeStep = 1.0 / 120.0;
        int maxSubSteps = 5;

        while (!glfwWindowShouldClose(window)) {
            double now = glfwGetTime();
            double frameTime = now - last;
            last = now;

            if (frameTime > 0.25) frameTime = 0.25;
            accumulator += frameTime;

            int steps = 0;
            while (accumulator >= fixedTimeStep && steps < maxSubSteps) {
                world.step((float)fixedTimeStep);
                accumulator -= fixedTimeStep;
                steps++;
            }

            glClearColor(0.529f, 0.808f, 0.922f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);

            for (auto &u : world.bodies) {
                RigidBody* b = u.get();
                if (b->type == ShapeType::Circle) {
                    renderer.drawCircle(b->pos, b->radius, b->r, b->g, b->b);
                } else {
                    renderer.drawBox(b->pos, b->hw, b->hh, b->r, b->g, b->b);
                }
            }

            glfwSwapBuffers(window);
            glfwPollEvents();
        }
        glfwTerminate();
    }
};

int main() {
    Application app;
    if (!app.init()) return -1;
    app.run();
    return 0;
}