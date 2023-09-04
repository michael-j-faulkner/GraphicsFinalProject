#define COW_PATCH_FRAMERATE
//#define COW_PATCH_FRAMERATE_SLEEP
#include "include.cpp"

// Component Multiply
template <int N>
Vec<N> operator*(Vec<N> v1, Vec<N> v2) {
    Vec<N> result{};
    for (int i = 0; i < N; ++i) {
        result[i] = v1[i] * v2[i];
    }
    return result;
}

// Store the position and rotation of objects (parent for hierarchical objects)
struct Transform {
    Transform *parent = nullptr;
    mat4 R = globals.Identity;
    vec3 pos{};

    mat4 getM() {
        mat4 M = M4_Translation(pos) * R;
        if (parent == nullptr)
            return M;
        else
            return parent->getM() * M;
    }
};

// Control the collision of objects
struct Collider3D {
    enum class Type {
        Box,
        Cylinder,
        NONE
    };

    Collider3D()
        :   type{Type::NONE}, T{}
    {}

    Collider3D(Type type, Transform *T)
        : type{type}, T{T}
    {}

    Collider3D(Transform *T, vec3 origin, vec3 dimension)
        :  type{Type::Box}, T{T}, origin{origin}, dimension{dimension}
    {}

    Type type;
    Transform *T;

    // Debug draw function shows the corners of the collider
    void draw(mat4 P, mat4 V, vec3 color = monokai.white) {
        eso_begin(P * V * T->getM(), SOUP_POINTS);
        eso_color(color);
        for (int i = 0; i < 8; i++) {
            eso_vertex(origin + dimension * V3((i) % 2, ((i) / 2) % 2, (i) / 4));
        }
        eso_end();
    }

    // Checks if this collider intersects with the passed one
    bool intersects(Collider3D &collider) {
        if (type == Type::Box && collider.type == Type::Box)
            return box_intersects_box(*this, collider);
        if (type == Type::NONE || collider.type == Type::NONE)
            return false;
        
        ASSERT(false);
        return false;
    }

    static vec3 min_componentwise(vec3 v1, vec3 v2) {
        return {MIN(v1.x, v2.x), MIN(v1.y, v2.y), MIN(v1.z, v2.z)};
    }
      
    static vec3 max_componentwise(vec3 v1, vec3 v2) {
        return {MAX(v1.x, v2.x), MAX(v1.y, v2.y), MAX(v1.z, v2.z)};
    }

    vec3 origin, dimension;
    // Checks if two box colliders intersect
    static bool box_intersects_box(Collider3D box1, Collider3D box2) {
        mat4 box1M = box1.T->getM();
        vec3 box1pos1 = transformPoint(box1M, box1.origin);
        vec3 box1pos2 = transformPoint(box1M, box1.origin + box1.dimension);
        mat4 box2M = box2.T->getM();
        vec3 box2pos1 = transformPoint(box2M, box2.origin);
        vec3 box2pos2 = transformPoint(box2M, box2.origin + box2.dimension);
        vec3 box1_min = min_componentwise(box1pos1, box1pos2);
        vec3 box1_max = max_componentwise(box1pos1, box1pos2);
        vec3 box2_min = min_componentwise(box2pos1, box2pos2);
        vec3 box2_max = max_componentwise(box2pos1, box2pos2);
        return (box2_min.x < box1_max.x) && (box1_min.x < box2_max.x)
                && (box2_min.y < box1_max.y) && (box1_min.y < box2_max.y)
                && (box2_min.z < box1_max.z) && (box1_min.z < box2_max.z);
    }
};

// Generic game object
struct Object {
    Object()
        : mesh{}, T{}, collider{}, velocity{}, hasGravity{}, onGround{}
    {}

    Object(IndexedTriangleMesh3D mesh, Transform T, Collider3D collider, vec3 velocity = {}, bool isMovable = false, bool hasGravity = false) 
        : mesh{mesh}, T{T}, collider{collider}, velocity{velocity}, isMovable{isMovable}, hasGravity{hasGravity}, onGround{}
    {}

    IndexedTriangleMesh3D mesh;
    Transform T;
    Collider3D collider;
    vec3 velocity;
    bool isMovable = false;
    bool hasGravity;
    bool onGround;

    void draw(mat4 P, mat4 V) {
        mesh.draw(P, V, T.getM(), monokai.green);
    }
};

struct PlayerCamera {
    vec3 origin;
    double theta;
    double phi;
    double _angle_of_view;
};

struct Player : public Object {
    Player(PlayerCamera camera, vec3 camera_offset, IndexedTriangleMesh3D mesh, Transform T, Collider3D collider, vec3 velocity = {}, bool hasGravity = true) 
        : Object(mesh, T, collider, velocity, true, hasGravity), camera{camera}, camera_offset{camera_offset}, jump_time{}
    {}
    PlayerCamera camera;
    vec3 camera_offset;
    int jump_time;
};

struct Portal {
    Transform T;
    vec3 color;
    Portal *dest;
    Collider3D screen_collider;
    IndexedTriangleMesh3D screen_mesh;
    vec3 n;
    Object *screen;
    Object *border[4];

    void draw(mat4 P, mat4 V) {
        screen_mesh.draw(P, V, T.getM());
    }
};

Object *create_box(vec3 pos, vec3 dim, vec3 color = monokai.purple, Transform *parent = nullptr);

Portal *create_portal(Transform T, vec3 color, Portal *dest = nullptr) {
    Portal *portal = (Portal *)malloc(sizeof(Portal));
    *portal = {T, color, dest, Collider3D{&portal->T, {-10, -15, 0}, {20, 30, 0}}};
    portal->n = transformVector(transpose(inverse(portal->T.getM())), V3(0.0, 0.0, -1.0));
    
    // Use 4 boxes to make the outline around the portal
    portal->border[0] = create_box(V3(-11, -16, -.1), V3(22, 1, .2), color, &portal->T);
    portal->border[1] = create_box(V3(-11, -15, -.1), V3(1, 30, .2), color, &portal->T);
    portal->border[2] = create_box(V3(10, -15, -.1), V3(1, 30, .2), color, &portal->T);
    portal->border[3] = create_box(V3(-11, 15, -.1), V3(22, 1, .2), color, &portal->T);
    { // Screen Mesh
        portal->screen_mesh.num_vertices = 4;
        portal->screen_mesh.vertex_positions = (vec3 *) malloc(portal->screen_mesh.num_vertices * sizeof(vec3));
        { // Vertex Positions
            int k = 0;
            portal->screen_mesh.vertex_positions[k++] = {-10, -15, 0};
            portal->screen_mesh.vertex_positions[k++] = {10, -15, 0};
            portal->screen_mesh.vertex_positions[k++] = {10, 15, 0};
            portal->screen_mesh.vertex_positions[k++] = {-10, 15, 0};
        }

        portal->screen_mesh.num_triangles = 2;
        portal->screen_mesh.triangle_indices = (int3 *) malloc(portal->screen_mesh.num_triangles * sizeof(int3));
        { // Triangle Indicies
            int k = 0;
            portal->screen_mesh.triangle_indices[k++] = {0, 1, 3};
            portal->screen_mesh.triangle_indices[k++] = {1, 2, 3};
        }

        portal->screen_mesh.vertex_colors = (vec3 *) malloc(portal->screen_mesh.num_vertices * sizeof(vec3));
        { // Vertex Colors
            for (int k = 0; k < portal->screen_mesh.num_vertices; ++k) {
                portal->screen_mesh.vertex_colors[k] = color;
            }
        }
    }

    return portal;
}

void free_portal(Portal *portal) {
    free(portal->screen_mesh.vertex_positions);
    free(portal->screen_mesh.vertex_colors);
    free(portal->screen_mesh.triangle_indices);
    for (int i = 0; i < 4; ++i) {
        free(portal->border[i]);
    }
    free(portal);
}

Object *create_box(vec3 pos, vec3 dim, vec3 color, Transform *parent) {
    Object *box_ptr = (Object *)malloc(sizeof(Object));
    Object box{};
    box.T.parent = parent;

    // Extract the boxes dimensions
    const double l = dim.x;
    const double w = dim.z;
    const double h = dim.y;
    box.T.pos = pos;
    
    box.mesh.num_vertices = 24;
    box.mesh.vertex_positions = (vec3 *) malloc(box.mesh.num_vertices * sizeof(vec3));
    { // Vertex Positions
        int k = 0;
        box.mesh.vertex_positions[k++] = {0, 0, 0};
        box.mesh.vertex_positions[k++] = {l, 0, 0};
        box.mesh.vertex_positions[k++] = {l, 0, w};
        box.mesh.vertex_positions[k++] = {0, 0, w};
        box.mesh.vertex_positions[k++] = {0, h, 0};
        box.mesh.vertex_positions[k++] = {l, h, 0};
        box.mesh.vertex_positions[k++] = {l, h, w};
        box.mesh.vertex_positions[k++] = {0, h, w};
        
        box.mesh.vertex_positions[k++] = {0, 0, 0};
        box.mesh.vertex_positions[k++] = {l, 0, 0};
        box.mesh.vertex_positions[k++] = {l, 0, w};
        box.mesh.vertex_positions[k++] = {0, 0, w};
        box.mesh.vertex_positions[k++] = {0, h, 0};
        box.mesh.vertex_positions[k++] = {l, h, 0};
        box.mesh.vertex_positions[k++] = {l, h, w};
        box.mesh.vertex_positions[k++] = {0, h, w};
        
        box.mesh.vertex_positions[k++] = {0, 0, 0};
        box.mesh.vertex_positions[k++] = {l, 0, 0};
        box.mesh.vertex_positions[k++] = {l, 0, w};
        box.mesh.vertex_positions[k++] = {0, 0, w};
        box.mesh.vertex_positions[k++] = {0, h, 0};
        box.mesh.vertex_positions[k++] = {l, h, 0};
        box.mesh.vertex_positions[k++] = {l, h, w};
        box.mesh.vertex_positions[k++] = {0, h, w};
    }

    box.mesh.num_triangles = 12;
    box.mesh.triangle_indices = (int3 *) malloc(box.mesh.num_triangles * sizeof(int3));
    { // Triangle Indicies
        int k = 0;
        box.mesh.triangle_indices[k++] = {0, 1, 2};
        box.mesh.triangle_indices[k++] = {0, 2, 3};
        box.mesh.triangle_indices[k++] = {8, 12, 13};
        box.mesh.triangle_indices[k++] = {8, 13, 9};
        box.mesh.triangle_indices[k++] = {16, 23, 20};
        box.mesh.triangle_indices[k++] = {16, 19, 23};
        box.mesh.triangle_indices[k++] = {18, 17, 21};
        box.mesh.triangle_indices[k++] = {18, 21, 22};
        box.mesh.triangle_indices[k++] = {10, 14, 15};
        box.mesh.triangle_indices[k++] = {10, 15, 11};
        box.mesh.triangle_indices[k++] = {4, 6, 5};
        box.mesh.triangle_indices[k++] = {4, 7, 6};
    }

    box.mesh.vertex_normals = (vec3 *) malloc(box.mesh.num_vertices * sizeof(vec3));
    { // Vertex Normals
        int k = 0;
        box.mesh.vertex_normals[k++] = {0, -1, 0};
        box.mesh.vertex_normals[k++] = {0, -1, 0};
        box.mesh.vertex_normals[k++] = {0, -1, 0};
        box.mesh.vertex_normals[k++] = {0, -1, 0};
        box.mesh.vertex_normals[k++] = {0, 1, 0};
        box.mesh.vertex_normals[k++] = {0, 1, 0};
        box.mesh.vertex_normals[k++] = {0, 1, 0};
        box.mesh.vertex_normals[k++] = {0, 1, 0};
        box.mesh.vertex_normals[k++] = {0, 0, -1};
        box.mesh.vertex_normals[k++] = {0, 0, -1};
        box.mesh.vertex_normals[k++] = {0, 0, 1};
        box.mesh.vertex_normals[k++] = {0, 0, 1};
        box.mesh.vertex_normals[k++] = {0, 0, -1};
        box.mesh.vertex_normals[k++] = {0, 0, -1};
        box.mesh.vertex_normals[k++] = {0, 0, 1};
        box.mesh.vertex_normals[k++] = {0, 0, 1};
        box.mesh.vertex_normals[k++] = {-1, 0, 0};
        box.mesh.vertex_normals[k++] = {1, 0, 0};
        box.mesh.vertex_normals[k++] = {1, 0, 0};
        box.mesh.vertex_normals[k++] = {-1, 0, 0};
        box.mesh.vertex_normals[k++] = {-1, 0, 0};
        box.mesh.vertex_normals[k++] = {1, 0, 0};
        box.mesh.vertex_normals[k++] = {1, 0, 0};
        box.mesh.vertex_normals[k++] = {-1, 0, 0};
    }

    box.mesh.vertex_colors = (vec3 *) malloc(box.mesh.num_vertices * sizeof(vec3));
    { // Vertex Colors
        for (int k = 0; k < box.mesh.num_vertices; ++k) {
            box.mesh.vertex_colors[k] = color;
        }
    }
    
    box.collider = Collider3D{&box_ptr->T, {0, 0, 0}, {l, h, w}};
    *box_ptr = box;
    return box_ptr;
}

// Allocates all the objects for the demo world
constexpr int NUM_OBJECTS = 14;
Object **create_objects() {
    Object **objects = (Object **)malloc(NUM_OBJECTS * sizeof(Object*));

    int i = 0;
    { // Player
        Player *player = (Player *)malloc(sizeof(Player));
        *player = Player{{{}, 0.0, 0.0, RAD(60)}, {0.0, 0.0, 0.0}, {}, {nullptr, globals.Identity, {0.0, 20.0, 0.0}}, Collider3D{&player->T, {-2.0, -17.0, -2.0}, {4.0, 20.0, 4.0}}};

        objects[i++] = player;
    }
    { // Level Geometry
        objects[i++] = create_box({ -500, -10, -500 }, { 1000, 10, 1000 }, color_kelly(i));
        objects[i++] = create_box({ 20, 0, 0 }, { 10, 10, 10 }, color_kelly(i));
        objects[i++] = create_box({ 40, 0, 20 }, { 20, 20, 20 }, color_kelly(i));
        objects[i++] = create_box({ 70, 0, 10 }, { 30, 30, 30 }, color_kelly(i));
        objects[i++] = create_box({ 110, 0, -50 }, { 40, 40, 40 }, color_kelly(i));
        objects[i++] = create_box({ 50, 0, -110 }, { 50, 50, 50 }, color_kelly(i));
        objects[i++] = create_box({ -20, 0, -140 }, { 60, 60, 60 }, color_kelly(i));
        objects[i++] = create_box({ -100, 0, -110 }, { 70, 70, 70 }, color_kelly(i));
        objects[i++] = create_box({ -190, 0, -30 }, { 80, 80, 80 }, color_kelly(i));
        objects[i++] = create_box({ -130, 0, 60 }, { 90, 90, 90 }, color_kelly(i));
        objects[i++] = create_box({ -80, 90, 105 }, { 30, 30, 30 }, color_kelly(i-1));
        objects[i++] = create_box({ 20, 0, 60 }, { 90, 90, 90 }, color_kelly(i));
        objects[i++] = create_box({ 50, 90, 90 }, { 50, 90, 50 }, color_kelly(i-1));
    }
    ASSERT(NUM_OBJECTS == i);
    return objects;
}

// Cleans up all objects created by create_objects()
void free_objects(Object **objects) {
    for (int i = 0; i < NUM_OBJECTS; ++i) {
        free(objects[i]->mesh.vertex_positions);
        free(objects[i]->mesh.vertex_normals);
        free(objects[i]->mesh.triangle_indices);
        free(objects[i]->mesh.vertex_colors);
        free(objects[i]);
    }
    free(objects);
}

mat4 player_camera_get_C(PlayerCamera *player) {
    return M4_Translation(player->origin) * M4_RotationAboutYAxis(player->theta) * M4_RotationAboutXAxis(player->phi);
}

mat4 player_camera_get_V(PlayerCamera *player) {
    return inverse(player_camera_get_C(player));
}

void update_player_camera(Player *player) {
    // Offset camera to be at "eye level"
    player->camera.origin = transformPoint(player->T.getM(), player->camera_offset);

    // Capture mouse focus
    if (!window_is_pointer_locked() && globals.mouse_left_pressed)
            window_pointer_lock();
    if (globals.key_pressed[COW_KEY_ESCAPE])
        window_pointer_unlock();

    // Update direction player is looking
    if (window_is_pointer_locked()) {
        player->camera.theta = player->camera.theta + (-PI / 2) * globals.mouse_change_in_position_NDC.x;
        player->camera.phi = CLAMP(player->camera.phi + (PI / 2) * globals.mouse_change_in_position_NDC.y, -PI/2, PI/2);
    }
}

void tick_player_controls(Player *player) {
    constexpr real acceleration = 0.6;

    if (globals.key_pressed[COW_KEY_SPACE] && player->onGround) {
        player->velocity += {0.0, 4, 0.0};
    }

    vec3 delta_v = {};
    if (globals.key_held['w']) 
        delta_v += V3(0.0, 0.0, -1.0);
    if (globals.key_held['s'])
        delta_v += V3(0.0, 0.0, 1.0);
    if (globals.key_held['a'])
        delta_v += V3(-1.0, 0.0, 0.0);
    if (globals.key_held['d'])
        delta_v += V3(1.0, 0.0, 0.0);

    // Normalize player's direction and apply their acceleration
    if (!IS_ZERO(norm(delta_v)))
        delta_v = acceleration * transformVector(M4_RotationAboutYAxis(player->camera.theta), normalized(delta_v));
        
    // Make it harder to move in the air
    player->velocity += player->onGround ? delta_v : delta_v * 0.1;
}

void move_with_collision(Object **obj, int i) {
    // Increment the objects position
    vec3 original_pos = obj[i]->T.pos;
    obj[i]->T.pos += obj[i]->velocity;

    // Check for any intersections
    for (int j = 0; j < NUM_OBJECTS; ++j) {
        if (i == j)
            continue;
        if (obj[i]->isMovable && obj[i]->collider.intersects(obj[j]->collider)) {
            // Calculate face it entered through
            real min_t = DBL_MAX;
            int min_k = -1;
            real min_d = -1;
            for (int k = 0; k < 6; ++k) {
                if (IS_ZERO(obj[i]->velocity[k % 3]))
                    continue;
                // Calculate the distance between 2 faces of the box collider
                real r = obj[i]->T.pos[k % 3] + obj[i]->collider.origin[k % 3] + ((5 - k) / 3) * obj[i]->collider.dimension[k % 3];
                real d = (obj[j]->T.pos[k % 3] + obj[j]->collider.origin[k % 3] + (k / 3) * obj[j]->collider.dimension[k % 3] - r);
                if (IS_ZERO(obj[i]->velocity[k % 3] + d)) //  Fix for floating point imprecision
                    d = -obj[i]->velocity[k % 3];

                // Calculate the amount of "time" the object would have traveled to enter from each face and choose
                // the face that requires the smallest time as the one it entered through
                real t = d / (-obj[i]->velocity[k % 3]);
                if (t < min_t && t >= 0.0) {
                    min_k = k;
                    min_t = t;
                    min_d = d;
                }
            }

            // Move object out of the other through the face it entered
            if (min_k >= 0) {
                vec3 n = {};
                vec3 d = {};
                n[min_k % 3] = min_k < 3 ? -1.0 : 1.0;
                d[min_k % 3] = min_d;

                obj[i]->T.pos += d;
                obj[i]->velocity = obj[i]->velocity - dot(obj[i]->velocity, n) * n;
                break;
            } else {
                obj[i]->T.pos = original_pos;
            }
        }
    }
} 

void check_for_teleport(Object *obj, vec3 old_pos, Portal **portals, int num_portals, bool isPlayer = false) {
    for (int j = 0; j < num_portals; ++j) {
        real old_portal_side = dot(old_pos - portals[j]->T.pos, portals[j]->n);
        real new_portal_side = dot(obj->T.pos - portals[j]->T.pos, portals[j]->n);

        // If object changed sides of and is touching the portal then teleport them
        if (((old_portal_side >= 0 && new_portal_side < 0) || (old_portal_side < 0 && new_portal_side >= 0)) && obj->collider.intersects(portals[j]->screen_collider)) { 
            obj->T.pos = transformPoint(portals[j]->dest->T.getM() * inverse(portals[j]->T.getM()), obj->T.pos);
            obj->velocity = transformVector(portals[j]->dest->T.getM() * inverse(portals[j]->T.getM()), obj->velocity);
            vec3 n1 = portals[j]->n;
            vec3 n2 = portals[j]->dest->n;
            n1.y = 0; n2.y = 0;
            if (IS_ZERO(norm(n1)) || IS_ZERO(norm(n2)))
                break;
            if (isPlayer) // Fix since player isn't cylinder
                ((Player*)obj)->camera.theta -= atan2(n2.z, n2.x) - atan2(n1.z, n1.x);
            else
                obj[j].T.R = M4_RotationAboutYAxis(atan2(n2.z, n2.x) - atan2(n1.z, n1.x)) * obj[j].T.R;
            break;
        }
    }
}

void tick_objects(Object **obj, Portal **portals, int num_portals) {
    constexpr vec3 drag_ground = {0.3, 0.05, 0.3};
    constexpr vec3 drag_air = {0.03, 0.05, 0.03};
    constexpr vec3 gravity = {0.0, -0.3, 0.0};

    for (int i = 0; i < NUM_OBJECTS; ++i) {
        // Move each object according to their velocity
        vec3 old_pos = obj[i]->T.pos;
        obj[i]->velocity += obj[i]->onGround ? -obj[i]->velocity * drag_ground : -obj[i]->velocity * drag_air;
        move_with_collision(obj, i);
        check_for_teleport(obj[i], old_pos, portals, num_portals, i == 0);
        
        
        // Apply Gravity
        if (obj[i]->hasGravity) {
            vec3 original_pos = obj[i]->T.pos;
            vec3 original_vel = obj[i]->velocity;
            obj[i]->velocity += gravity;
            obj[i]->T.pos += gravity;
            obj[i]->onGround = false;
            for (int j = 0; j < NUM_OBJECTS; ++j) {
                if (i == j)
                    continue;
                if (obj[i]->collider.intersects(obj[j]->collider)) {
                    obj[i]->onGround = true;
                    obj[i]->T.pos = original_pos;
                    obj[i]->velocity = original_vel;
                    break;
                }
            }
            check_for_teleport(obj[i], original_pos, portals, num_portals);
        }
    }
}

// Discussion of the math for oblique clip plane found in this article:
// http://www.terathon.com/lengyel/Lengyel-Oblique.pdf
mat4 clip_projection_to_plane(const mat4 &P, vec4 plane) {
    mat4 modified_P = P;
    mat4 inverse_P = inverse(P);
    mat4 inverse_transpose = transpose(inverse_P);
    vec4 plane_in_clip_space = inverse_transpose * plane;
    vec4 Q_in_clip_space = {plane_in_clip_space.x < 0 ? -1.0 : 1.0 , plane_in_clip_space.y < 0 ? -1.0 : 1.0, 1.0, 1.0};
    vec4 Q_in_camera_space = inverse_P * Q_in_clip_space;
    vec4 new_row_3 = -2 * Q_in_camera_space.z / dot(plane, Q_in_camera_space) * plane + V4(0.0, 0.0, 1.0, 0.0);
    modified_P(2, 0) = new_row_3[0];
    modified_P(2, 1) = new_row_3[1];
    modified_P(2, 2) = new_row_3[2];
    modified_P(2, 3) = new_row_3[3];
    return modified_P;
}

// Recursion limits
constexpr int NESTED_OTHER_PORTAL_DEPTH = 1;
constexpr int NESTED_PORTAL_DEPTH = 10;
void draw_portal(Portal &portal, Object **obj, Portal **portals, int num_portals, const mat4 &P, mat4 V, int recursion_depth = 0) {
    if (recursion_depth >= NESTED_PORTAL_DEPTH)
        return;
    // Get the view matrix as if you were teleported relative to the portal
    mat4 portal_V = inverse(portal.dest->T.getM() * inverse(portal.T.getM()) * inverse(V));
    
    // Modify the projection matrix to start at the plane of the portal
    vec3 normal = transformVector(transpose(inverse(portal_V)), portal.dest->n);
    vec3 dest_portal_in_camera_coords = transformPoint(portal_V, portal.dest->T.pos);
    if (dot(-normal, dest_portal_in_camera_coords) >= 0)
        normal = -normal;
    mat4 portal_P = clip_projection_to_plane(P, V4(normal.x, normal.y, normal.z, dot(-normal, dest_portal_in_camera_coords)));
    
    // Fill the stencil of the portal with 1s and restrict drawing to that region
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_STENCIL_TEST);
    glStencilMask(0xFF);
    glStencilFunc(GL_EQUAL, recursion_depth, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
    portal.draw(P, V);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glStencilMask(0x00);
    glStencilFunc(GL_EQUAL, recursion_depth + 1, 0xFF);

    // Reset the depth buffer in the stenciled region
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_ALWAYS);
    glDepthRange(1, 1);
    portal.draw(P, V);
    glDepthRange(0, 1);
    glDepthFunc(GL_LEQUAL);
    
    // Make recursive call then reset drawing parameters that might be changed
    for (int i = 0; i < num_portals; ++i) {
        if (portal.dest == portals[i])
            continue;
        if (((&portal != portals[i]) && recursion_depth >= NESTED_OTHER_PORTAL_DEPTH))
            continue;

        // Check if portal is visible from new perspective before rendering it
        mat4 portal_PVM = portal_P * portal_V * portals[i]->T.getM();
        vec2 min_max_in_clip_plane[3]{ {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0} };
        for (int j = 0; j < 4; ++j) {
            vec3 corner = transformPoint(portal_PVM, portals[i]->screen_mesh.vertex_positions[j]);
            for (int k = 0; k < 3; ++k) {
                if (corner[k] < min_max_in_clip_plane[k][0])
                    min_max_in_clip_plane[k][0] = corner[k];
                if (corner[k] > min_max_in_clip_plane[k][1])
                    min_max_in_clip_plane[k][1] = corner[k];
            }
        }

        // Only if a portal is visible do we render it
        if ((min_max_in_clip_plane[0][0] <= 1.0) && (-1.0 <= min_max_in_clip_plane[0][1])
                && (min_max_in_clip_plane[1][0] <= 1.0) && (-1.0 <= min_max_in_clip_plane[1][1])
                && (min_max_in_clip_plane[2][0] <= 1.0) && (-1.0 <= min_max_in_clip_plane[2][1])) {
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glStencilMask(0xFF);
            glDisable(GL_STENCIL_TEST);
            draw_portal(*portals[i], obj, portals, num_portals, portal_P, portal_V, recursion_depth + 1);
        }
    }
    glEnable(GL_STENCIL_TEST);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glStencilMask(0x00);
    glStencilFunc(GL_EQUAL, recursion_depth + 1, 0xFF);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // Draw the world from the fake camera's perspective only in the stencil region
    for(int j = 0; j < NUM_OBJECTS; ++j) {
        obj[j]->draw(portal_P, portal_V);
    }
    //portal.border_mesh.draw(portal_P, portal_V, portal.T.getM());
    for (int k = 0; k < num_portals; ++k) {
        for (int j = 0; j < 4; ++j) {
            portals[k]->border[j]->draw(portal_P, portal_V);
        }
    }

    // Overlay the depth of the portal screen as the depth for that region so other portals don't draw on it
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_ALWAYS);
    glStencilMask(0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
    portal.draw(P, V);
    glDepthFunc(GL_LEQUAL);

    // Go back to normal drawing
    glStencilMask(0xFF);
    glDisable(GL_STENCIL_TEST);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
}

void draw_world(Object **obj, Portal **portals, int num_portals) {
    Player &player = (Player&)(*obj[0]);
    mat4 P = _window_get_P_perspective(player.camera._angle_of_view);
    mat4 V = player_camera_get_V(&player.camera);

    // Render the portals (with recursion)
    for (int i = 0; i < num_portals; ++i) {
        draw_portal(*portals[i], obj, portals, num_portals, P, V);        
    }

    // Draw the world
    for(int i = 0; i < NUM_OBJECTS; ++i) {
        obj[i]->draw(P, V);
    }
    for (int i = 0; i < num_portals; ++i) {
        for (int j = 0; j < 4; ++j) {
            portals[i]->border[j]->draw(P, V);
        }
    }
}

void clear_screen() {
    // Manually Clear Buffers (cow makes assumptions that aren't true)
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);
    glStencilMask(0xFF);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
}

void final_proj() {
    Object **objects = create_objects();
    Player &player = (Player&)(*objects[0]);

    // Create the portals
    constexpr int num_portals = 4;
    Portal* portals[num_portals]{};
    portals[0] = create_portal({{}, M4_RotationAboutYAxis(RAD(90)), {-50, 136, 120}}, {0.0, 0.0, 1.0});
    portals[1] = create_portal({{}, M4_RotationAboutYAxis(RAD(90)) * M4_RotationAboutXAxis(RAD(90)), {-10, 40, 80}}, {0.0, 0.0, 1.0}, portals[0]);
    portals[0]->dest = portals[1];
    portals[2] = create_portal({{}, M4_RotationAboutYAxis(RAD(-90)), {-50, 16, 0}}, {0.0, 1.0, 1.0});
    portals[3] = create_portal({{}, M4_RotationAboutYAxis(RAD(-90)), {50, 16, 0}}, {0.0, 1.0, 1.0}, portals[2]);
    portals[2]->dest = portals[3];

    glClearStencil(0);
    while (cow_begin_frame()) {
        clear_screen();
        
        update_player_camera(&player);
        tick_player_controls(&player);
        tick_objects(objects, portals, num_portals);

        draw_world(objects, portals, num_portals);
        
        // Statistic readout
        //gui_readout("vel: ", &player.velocity);
        //gui_readout("pos: ", &player.T.pos);
        //gui_slider("NESTED_OTHER_PORTAL_DEPTH", &NESTED_OTHER_PORTAL_DEPTH, 0, 5, 'u', 'o');
        //gui_slider("NESTED_PORTAL_DEPTH", &NESTED_PORTAL_DEPTH, 0, 10, 'j', 'k');
    }

    free_objects(objects);
    for (int i = 0; i < num_portals; ++i) {
        free_portal(portals[i]);
    }
}


int main() {
    APPS{
        APP(final_proj);
    }
    
    return 0;
}