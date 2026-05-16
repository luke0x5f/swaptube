#define _POSIX_C_SOURCE 200809L
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { float x, y, z; } Vec3;
typedef struct { Vec3 kd, ks; float ns, reflectivity; char name[64]; } Material;
typedef struct { Vec3 v0, v1, v2, n0, n1, n2, face_n, centroid, bmin, bmax; int mat; } Triangle;
typedef struct { int v, n; } FaceVert;
typedef struct { Vec3 orig, dir; } Ray;
typedef struct { Vec3 bmin, bmax; int left, right, start, count; } BVHNode;
typedef struct { float t; Vec3 pos, normal; int tri; } Hit;

typedef struct { Vec3 *data; int len, cap; } VecArray;
typedef struct { Triangle *data; int len, cap; } TriArray;
typedef struct { Material *data; int len, cap; } MatArray;
typedef struct { BVHNode *data; int len, cap; } NodeArray;

typedef struct {
    VecArray verts, norms;
    TriArray tris;
    MatArray mats;
    int current_mat;
    char base_dir[1024];
} Scene;

static Vec3 v3(float x, float y, float z) { Vec3 v = {x, y, z}; return v; }
static Vec3 add(Vec3 a, Vec3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static Vec3 sub(Vec3 a, Vec3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static Vec3 mul(Vec3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static Vec3 had(Vec3 a, Vec3 b) { return v3(a.x * b.x, a.y * b.y, a.z * b.z); }
static float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static Vec3 cross(Vec3 a, Vec3 b) { return v3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x); }
static float clampf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
static Vec3 minv(Vec3 a, Vec3 b) { return v3(fminf(a.x,b.x), fminf(a.y,b.y), fminf(a.z,b.z)); }
static Vec3 maxv(Vec3 a, Vec3 b) { return v3(fmaxf(a.x,b.x), fmaxf(a.y,b.y), fmaxf(a.z,b.z)); }
static Vec3 norm(Vec3 a) { float l = sqrtf(dot(a, a)); return l > 0.0f ? mul(a, 1.0f / l) : v3(0, 0, 0); }
static Vec3 reflectv(Vec3 i, Vec3 n) { return sub(i, mul(n, 2.0f * dot(i, n))); }
static Vec3 mixv(Vec3 a, Vec3 b, float t) { return add(mul(a, 1.0f - t), mul(b, t)); }

static void *xrealloc(void *p, size_t n) { void *r = realloc(p, n); if (!r) { perror("realloc"); exit(1); } return r; }
static void push_vec(VecArray *a, Vec3 v) { if (a->len == a->cap) { a->cap = a->cap ? a->cap * 2 : 256; a->data = xrealloc(a->data, (size_t)a->cap * sizeof(*a->data)); } a->data[a->len++] = v; }
static void push_tri(TriArray *a, Triangle t) { if (a->len == a->cap) { a->cap = a->cap ? a->cap * 2 : 256; a->data = xrealloc(a->data, (size_t)a->cap * sizeof(*a->data)); } a->data[a->len++] = t; }
static int push_mat(MatArray *a, Material m) { if (a->len == a->cap) { a->cap = a->cap ? a->cap * 2 : 32; a->data = xrealloc(a->data, (size_t)a->cap * sizeof(*a->data)); } a->data[a->len] = m; return a->len++; }
static int push_node(NodeArray *a, BVHNode n) { if (a->len == a->cap) { a->cap = a->cap ? a->cap * 2 : 256; a->data = xrealloc(a->data, (size_t)a->cap * sizeof(*a->data)); } a->data[a->len] = n; return a->len++; }

static char *trim(char *s) {
    while (isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = '\0';
    return s;
}

static void path_dir(const char *path, char *out, size_t out_sz) {
    const char *slash = strrchr(path, '/');
    if (!slash) { snprintf(out, out_sz, "."); return; }
    size_t n = (size_t)(slash - path);
    if (n >= out_sz) n = out_sz - 1;
    memcpy(out, path, n); out[n] = '\0';
}

static int find_mat(Scene *s, const char *name) {
    for (int i = 0; i < s->mats.len; i++) if (strcmp(s->mats.data[i].name, name) == 0) return i;
    return 0;
}

static Material default_mat(const char *name) {
    Material m;
    memset(&m, 0, sizeof(m));
    snprintf(m.name, sizeof(m.name), "%s", name ? name : "default");
    m.kd = v3(0.75f, 0.75f, 0.75f);
    m.ks = v3(0.08f, 0.08f, 0.08f);
    m.ns = 32.0f;
    m.reflectivity = -1.0f;
    return m;
}

static void load_mtl(Scene *s, const char *mtl_name) {
    char path[1400];
    snprintf(path, sizeof(path), "%s/%s", s->base_dir, mtl_name);
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "warning: cannot open MTL %s: %s\n", path, strerror(errno)); return; }
    char line[1024];
    int cur = -1;
    while (fgets(line, sizeof(line), f)) {
        char *p = trim(line);
        if (!*p || *p == '#') continue;
        char key[64];
        if (sscanf(p, "%63s", key) != 1) continue;
        p += strlen(key); p = trim(p);
        if (strcmp(key, "newmtl") == 0) {
            Material m = default_mat(p);
            cur = push_mat(&s->mats, m);
        } else if (cur >= 0 && strcmp(key, "Kd") == 0) {
            sscanf(p, "%f %f %f", &s->mats.data[cur].kd.x, &s->mats.data[cur].kd.y, &s->mats.data[cur].kd.z);
        } else if (cur >= 0 && strcmp(key, "Ks") == 0) {
            sscanf(p, "%f %f %f", &s->mats.data[cur].ks.x, &s->mats.data[cur].ks.y, &s->mats.data[cur].ks.z);
        } else if (cur >= 0 && strcmp(key, "Ns") == 0) {
            sscanf(p, "%f", &s->mats.data[cur].ns);
        } else if (cur >= 0 && (strcmp(key, "refl") == 0 || strcmp(key, "reflectivity") == 0)) {
            sscanf(p, "%f", &s->mats.data[cur].reflectivity);
        }
    }
    fclose(f);
}

static FaceVert parse_face_vert(const char *tok, int vcount, int ncount) {
    FaceVert fv = {0, 0};
    char tmp[128];
    snprintf(tmp, sizeof(tmp), "%s", tok);
    char *a = strtok(tmp, "/");
    char *b = strtok(NULL, "/");
    char *c = strtok(NULL, "/");
    if (a) fv.v = atoi(a);
    if (c) fv.n = atoi(c);
    else if (b && strstr(tok, "//")) fv.n = atoi(b);
    if (fv.v < 0) fv.v = vcount + fv.v + 1;
    if (fv.n < 0) fv.n = ncount + fv.n + 1;
    return fv;
}

static void add_triangle(Scene *s, FaceVert a, FaceVert b, FaceVert c) {
    if (a.v <= 0 || b.v <= 0 || c.v <= 0 || a.v > s->verts.len || b.v > s->verts.len || c.v > s->verts.len) return;
    Triangle t;
    memset(&t, 0, sizeof(t));
    t.v0 = s->verts.data[a.v - 1]; t.v1 = s->verts.data[b.v - 1]; t.v2 = s->verts.data[c.v - 1];
    t.face_n = norm(cross(sub(t.v1, t.v0), sub(t.v2, t.v0)));
    t.n0 = (a.n > 0 && a.n <= s->norms.len) ? s->norms.data[a.n - 1] : t.face_n;
    t.n1 = (b.n > 0 && b.n <= s->norms.len) ? s->norms.data[b.n - 1] : t.face_n;
    t.n2 = (c.n > 0 && c.n <= s->norms.len) ? s->norms.data[c.n - 1] : t.face_n;
    t.centroid = mul(add(add(t.v0, t.v1), t.v2), 1.0f / 3.0f);
    t.bmin = minv(t.v0, minv(t.v1, t.v2));
    t.bmax = maxv(t.v0, maxv(t.v1, t.v2));
    t.mat = s->current_mat;
    push_tri(&s->tris, t);
}

static void load_obj(Scene *s, const char *obj_path) {
    path_dir(obj_path, s->base_dir, sizeof(s->base_dir));
    push_mat(&s->mats, default_mat("default"));
    FILE *f = fopen(obj_path, "r");
    if (!f) { fprintf(stderr, "cannot open OBJ %s: %s\n", obj_path, strerror(errno)); exit(1); }
    char line[2048];
    while (fgets(line, sizeof(line), f)) {
        char *p = trim(line);
        if (!*p || *p == '#') continue;
        if (strncmp(p, "v ", 2) == 0) { Vec3 v; if (sscanf(p + 2, "%f %f %f", &v.x, &v.y, &v.z) == 3) push_vec(&s->verts, v); }
        else if (strncmp(p, "vn ", 3) == 0) { Vec3 n; if (sscanf(p + 3, "%f %f %f", &n.x, &n.y, &n.z) == 3) push_vec(&s->norms, norm(n)); }
        else if (strncmp(p, "mtllib ", 7) == 0) load_mtl(s, trim(p + 7));
        else if (strncmp(p, "usemtl ", 7) == 0) s->current_mat = find_mat(s, trim(p + 7));
        else if (strncmp(p, "f ", 2) == 0) {
            FaceVert fv[64]; int n = 0;
            char *ctx = NULL;
            for (char *tok = strtok_r(p + 2, " \t\r\n", &ctx); tok && n < 64; tok = strtok_r(NULL, " \t\r\n", &ctx))
                fv[n++] = parse_face_vert(tok, s->verts.len, s->norms.len);
            for (int i = 1; i + 1 < n; i++) add_triangle(s, fv[0], fv[i], fv[i + 1]);
        }
    }
    fclose(f);
}

static Triangle *g_sort_tris;
static int g_axis;
static int cmp_centroid(const void *a, const void *b) {
    const int ia = *(const int *)a, ib = *(const int *)b;
    float ca = g_axis == 0 ? g_sort_tris[ia].centroid.x : (g_axis == 1 ? g_sort_tris[ia].centroid.y : g_sort_tris[ia].centroid.z);
    float cb = g_axis == 0 ? g_sort_tris[ib].centroid.x : (g_axis == 1 ? g_sort_tris[ib].centroid.y : g_sort_tris[ib].centroid.z);
    return (ca > cb) - (ca < cb);
}

static int build_bvh(NodeArray *nodes, Triangle *tris, int *idx, int start, int count) {
    BVHNode node;
    node.bmin = v3(FLT_MAX, FLT_MAX, FLT_MAX); node.bmax = v3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    node.left = node.right = -1; node.start = start; node.count = count;
    Vec3 cmin = node.bmin, cmax = node.bmax;
    for (int i = start; i < start + count; i++) {
        Triangle *t = &tris[idx[i]];
        node.bmin = minv(node.bmin, t->bmin); node.bmax = maxv(node.bmax, t->bmax);
        cmin = minv(cmin, t->centroid); cmax = maxv(cmax, t->centroid);
    }
    int ni = push_node(nodes, node);
    if (count <= 4) return ni;
    Vec3 ext = sub(cmax, cmin);
    g_axis = ext.x > ext.y && ext.x > ext.z ? 0 : (ext.y > ext.z ? 1 : 2);
    g_sort_tris = tris;
    qsort(idx + start, (size_t)count, sizeof(*idx), cmp_centroid);
    int mid = start + count / 2;
    nodes->data[ni].left = build_bvh(nodes, tris, idx, start, mid - start);
    nodes->data[ni].right = build_bvh(nodes, tris, idx, mid, start + count - mid);
    nodes->data[ni].count = 0;
    return ni;
}

static int hit_aabb(Ray r, Vec3 bmin, Vec3 bmax, float tmax) {
    float tmin = 0.001f;
    for (int a = 0; a < 3; a++) {
        float o = a == 0 ? r.orig.x : (a == 1 ? r.orig.y : r.orig.z);
        float d = a == 0 ? r.dir.x : (a == 1 ? r.dir.y : r.dir.z);
        float mn = a == 0 ? bmin.x : (a == 1 ? bmin.y : bmin.z);
        float mx = a == 0 ? bmax.x : (a == 1 ? bmax.y : bmax.z);
        float inv = 1.0f / d;
        float t0 = (mn - o) * inv, t1 = (mx - o) * inv;
        if (inv < 0.0f) { float tmp = t0; t0 = t1; t1 = tmp; }
        tmin = fmaxf(tmin, t0); tmax = fminf(tmax, t1);
        if (tmax <= tmin) return 0;
    }
    return 1;
}

static int hit_tri(Ray r, Triangle *t, float *out_t, float *u, float *v) {
    Vec3 e1 = sub(t->v1, t->v0), e2 = sub(t->v2, t->v0);
    Vec3 p = cross(r.dir, e2);
    float det = dot(e1, p);
    if (fabsf(det) < 1e-7f) return 0;
    float inv = 1.0f / det;
    Vec3 tv = sub(r.orig, t->v0);
    *u = dot(tv, p) * inv;
    if (*u < 0.0f || *u > 1.0f) return 0;
    Vec3 q = cross(tv, e1);
    *v = dot(r.dir, q) * inv;
    if (*v < 0.0f || *u + *v > 1.0f) return 0;
    float tt = dot(e2, q) * inv;
    if (tt <= 0.001f) return 0;
    *out_t = tt;
    return 1;
}

static int trace_hit(Ray r, Scene *s, NodeArray *nodes, int *idx, Hit *hit) {
    int stack[128], sp = 0, ok = 0;
    float best = FLT_MAX;
    stack[sp++] = 0;
    while (sp) {
        BVHNode *n = &nodes->data[stack[--sp]];
        if (!hit_aabb(r, n->bmin, n->bmax, best)) continue;
        if (n->left < 0) {
            for (int i = n->start; i < n->start + n->count; i++) {
                float tt, u, v;
                int ti = idx[i];
                if (hit_tri(r, &s->tris.data[ti], &tt, &u, &v) && tt < best) {
                    Triangle *tr = &s->tris.data[ti];
                    best = tt; ok = 1; hit->tri = ti; hit->t = tt;
                    hit->pos = add(r.orig, mul(r.dir, tt));
                    hit->normal = norm(add(add(mul(tr->n1, u), mul(tr->n2, v)), mul(tr->n0, 1.0f - u - v)));
                    if (dot(hit->normal, r.dir) > 0.0f) hit->normal = mul(hit->normal, -1.0f);
                }
            }
        } else {
            if (sp + 2 < 128) { stack[sp++] = n->left; stack[sp++] = n->right; }
        }
    }
    return ok;
}

static Vec3 sky(Vec3 d) {
    float t = 0.5f * (d.y + 1.0f);
    return mixv(v3(0.78f, 0.82f, 0.90f), v3(0.22f, 0.42f, 0.72f), t);
}

static Vec3 radiance(Ray r, Scene *s, NodeArray *nodes, int *idx, int depth) {
    Hit h;
    if (!trace_hit(r, s, nodes, idx, &h)) return sky(r.dir);
    Triangle *tri = &s->tris.data[h.tri];
    Material *m = &s->mats.data[tri->mat];
    Vec3 light_pos = v3(-1.2f, 3.6f, 2.4f), light_color = v3(5.0f, 4.7f, 4.2f);
    Vec3 to_l = sub(light_pos, h.pos);
    float dist2 = dot(to_l, to_l);
    Vec3 ldir = mul(to_l, 1.0f / sqrtf(dist2));
    Ray shadow = { add(h.pos, mul(h.normal, 0.003f)), ldir };
    Hit sh;
    int blocked = trace_hit(shadow, s, nodes, idx, &sh) && sh.t * sh.t < dist2;
    float ndl = blocked ? 0.0f : fmaxf(0.0f, dot(h.normal, ldir));
    Vec3 view = mul(r.dir, -1.0f);
    Vec3 halfv = norm(add(ldir, view));
    float spec = blocked ? 0.0f : powf(fmaxf(0.0f, dot(h.normal, halfv)), fmaxf(1.0f, m->ns));
    Vec3 ambient = mul(m->kd, 0.08f);
    Vec3 diffuse = mul(had(m->kd, light_color), ndl / fmaxf(0.2f, dist2));
    Vec3 glossy = mul(had(m->ks, light_color), spec);
    Vec3 color = add(ambient, add(diffuse, glossy));
    float refl = m->reflectivity >= 0.0f ? m->reflectivity : 0.35f * fmaxf(m->ks.x, fmaxf(m->ks.y, m->ks.z));
    refl = clampf(refl, 0.0f, 0.95f);
    if (depth > 0 && refl > 0.001f) {
        Ray rr = { add(h.pos, mul(h.normal, 0.004f)), norm(reflectv(r.dir, h.normal)) };
        color = mixv(color, radiance(rr, s, nodes, idx, depth - 1), refl);
    }
    return color;
}

static void usage(const char *argv0) {
    fprintf(stderr, "usage: %s scene.obj output.ppm [width height] [--samples N] [--bounces N]\n", argv0);
}

int main(int argc, char **argv) {
    if (argc < 3) { usage(argv[0]); return 2; }
    int width = argc > 4 ? atoi(argv[3]) : 800;
    int height = argc > 4 ? atoi(argv[4]) : 450;
    int samples = 1, bounces = 2;
    for (int i = 5; i < argc; i++) {
        if (strcmp(argv[i], "--samples") == 0 && i + 1 < argc) samples = atoi(argv[++i]);
        else if (strcmp(argv[i], "--bounces") == 0 && i + 1 < argc) bounces = atoi(argv[++i]);
    }
    if (width <= 0 || height <= 0 || samples <= 0 || bounces < 0) { usage(argv[0]); return 2; }

    Scene s;
    memset(&s, 0, sizeof(s));
    load_obj(&s, argv[1]);
    if (s.tris.len == 0) { fprintf(stderr, "no triangles loaded\n"); return 1; }
    int *idx = xrealloc(NULL, (size_t)s.tris.len * sizeof(*idx));
    for (int i = 0; i < s.tris.len; i++) idx[i] = i;
    NodeArray nodes = {0};
    build_bvh(&nodes, s.tris.data, idx, 0, s.tris.len);
    fprintf(stderr, "loaded %d vertices, %d triangles, %d materials, %d BVH nodes\n", s.verts.len, s.tris.len, s.mats.len, nodes.len);

    FILE *out = fopen(argv[2], "wb");
    if (!out) { fprintf(stderr, "cannot write %s: %s\n", argv[2], strerror(errno)); return 1; }
    fprintf(out, "P6\n%d %d\n255\n", width, height);

    Vec3 cam = v3(0.0f, 1.2f, 4.2f), target = v3(0.0f, 0.7f, 0.0f);
    Vec3 forward = norm(sub(target, cam));
    Vec3 right = norm(cross(forward, v3(0, 1, 0)));
    Vec3 up = cross(right, forward);
    float aspect = (float)width / (float)height;
    float scale = tanf(55.0f * 0.5f * 3.1415926535f / 180.0f);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Vec3 col = v3(0, 0, 0);
            for (int sy = 0; sy < samples; sy++) for (int sx = 0; sx < samples; sx++) {
                float u = (2.0f * ((float)x + ((float)sx + 0.5f) / samples) / (float)width - 1.0f) * aspect * scale;
                float v = (1.0f - 2.0f * ((float)y + ((float)sy + 0.5f) / samples) / (float)height) * scale;
                Ray r = { cam, norm(add(forward, add(mul(right, u), mul(up, v)))) };
                col = add(col, radiance(r, &s, &nodes, idx, bounces));
            }
            col = mul(col, 1.0f / (float)(samples * samples));
            unsigned char rgb[3] = {
                (unsigned char)(255.0f * powf(clampf(col.x, 0.0f, 1.0f), 1.0f / 2.2f)),
                (unsigned char)(255.0f * powf(clampf(col.y, 0.0f, 1.0f), 1.0f / 2.2f)),
                (unsigned char)(255.0f * powf(clampf(col.z, 0.0f, 1.0f), 1.0f / 2.2f))
            };
            fwrite(rgb, 1, 3, out);
        }
    }
    fclose(out);
    free(idx); free(nodes.data); free(s.verts.data); free(s.norms.data); free(s.tris.data); free(s.mats.data);
    return 0;
}
