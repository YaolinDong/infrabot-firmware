/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "app_lcd_3d.h"

#include "app_config.h"
#include "app_lcd.h"
#include "app_lcd_mesh.h"
#include "app_sdram.h"
#include "app_state.h"
#include "infrabot_ik_mcu.h"
#include "main.h"

#include <math.h>
#include <stdlib.h>

#define DEG2RAD(d)  ((d) * 0.017453293f)

#define C3D_BG          APP_LCD_RGB565(8U, 10U, 16U)
#define C3D_GRID        APP_LCD_RGB565(40U, 46U, 58U)
#define C3D_BASE        APP_LCD_RGB565(90U, 90U, 100U)
#define C3D_AXIS_X      APP_LCD_RGB565(255U, 60U, 60U)
#define C3D_AXIS_Y      APP_LCD_RGB565(60U, 220U, 60U)
#define C3D_AXIS_Z      APP_LCD_RGB565(70U, 120U, 255U)

#define C3D_TOOL_AXIS_M 0.08f
#define C3D_GRID_HALF_M 0.4f
#define C3D_GRID_STEP_M 0.1f
/* Points of the tool axes: base, the seven joint frame origins, the flange. */
#define C3D_POINTS      (ARM_IK_AXES + 2U)

/* A redraw is skipped while no joint moved more than this (rad) and the
 * view did not change: a resting arm costs nothing. */
#define C3D_STILL_RAD   0.001f



typedef struct
{
  int16_t x0, y0, x1, y1; /* right and bottom are exclusive */
} Rect;

typedef struct
{
  uint16_t *pixels;
  int16_t w;
  int16_t h;
  Rect clip;
  int16_t origin_x, origin_y; /* screen-space position of pixels[0] */
} Canvas;

/* Own complete cache lines.  DMA2D only reads this buffer; the CPU clears
 * and rebuilds it before every use, then publishes it with one cache clean.
 * Keep it in DMA-accessible AXI SRAM, not the default DTCM .bss. */
#define C3D_FAST_PIXELS (64U * 1024U)
#if APP_LCD_FAST_BUFFER
static uint16_t fast_pixels[C3D_FAST_PIXELS]
    __attribute__((section(".lcd_ram"), aligned(32)));
#endif

typedef struct
{
  float sx, sy, depth;
} Projected;

/* Orbit camera, same convention and home view as the PC GUI's URDF viewer
 * (InfraBot SDK qt_control_gui.cpp setHomeView(-45, 25, 1.05, (0.05, 0, 0.22))):
 * the camera sits at azimuth `yaw` from +X and elevation `pitch` above the
 * target, Z up.  The LCD projects orthographically; 495 px/m matches the
 * GUI's 45-degree perspective at 1.05 m on this viewport height; there is
 * no zoom. */
#define CAM_HOME_YAW    DEG2RAD(-45.0f)
#define CAM_HOME_PITCH  DEG2RAD(25.0f)
#define CAM_HOME_SCALE  495.0f
static float cam_yaw = CAM_HOME_YAW;
static float cam_pitch = CAM_HOME_PITCH;
static const float cam_scale = CAM_HOME_SCALE;
static const float cam_target[3] = {0.05f, 0.0f, 0.22f};

/* Camera basis, rebuilt each frame: right, up, toward the camera. */
static float basis_r[3], basis_u[3], basis_d[3];
static float view_cx, view_cy;
static float view_scale;

/* What the screen shows now, for skipping unchanged frames. */
static float drawn_q[ARM_IK_AXES];
static float drawn_yaw, drawn_pitch;
static uint8_t drawn_valid;
static uint16_t drawn_x, drawn_y, drawn_w, drawn_h;
static Rect drawn_bounds;
static Rect drawn_scene_bounds;
/* Cycle counts of the last rendered frame: geometry (FK, projection, sort,
 * binning) and pixels (fill, raster, D-cache clean, DMA2D). */
static AppLcd3dTiming frame_timing;

void AppLcd3d_SetView(AppLcd3dView view)
{
  switch (view)
  {
    case APP_LCD3D_VIEW_FRONT: cam_yaw = 0.0f;              cam_pitch = 0.0f;             break;
    case APP_LCD3D_VIEW_SIDE:  cam_yaw = DEG2RAD(-90.0f);   cam_pitch = 0.0f;             break;
    case APP_LCD3D_VIEW_TOP:   cam_yaw = 0.0f;              cam_pitch = DEG2RAD(89.0f);   break;
    case APP_LCD3D_VIEW_HOME:
    default:
      cam_yaw = CAM_HOME_YAW;
      cam_pitch = CAM_HOME_PITCH;
      break;
  }
}

void AppLcd3d_Orbit(int16_t dx, int16_t dy)
{
  /* 0.5 degree per pixel, as in the GUI viewer's mouse drag. */
  cam_yaw -= DEG2RAD((float)dx * 0.5f);
  cam_pitch += DEG2RAD((float)dy * 0.5f);
  if (cam_pitch > DEG2RAD(89.0f)) cam_pitch = DEG2RAD(89.0f);
  if (cam_pitch < DEG2RAD(-89.0f)) cam_pitch = DEG2RAD(-89.0f);
}

void AppLcd3d_Timing(AppLcd3dTiming *timing)
{
  *timing = frame_timing;
}

void AppLcd3d_Invalidate(void)
{
  drawn_valid = 0U;
}

static void AppLcd3d_BuildCamera(int16_t w, int16_t h)
{
  const float cp = cosf(cam_pitch), sp = sinf(cam_pitch);
  const float cy = cosf(cam_yaw), sy = sinf(cam_yaw);

  basis_d[0] = cp * cy; basis_d[1] = cp * sy; basis_d[2] = sp;
  /* right = forward x Z with forward = -d, i.e. Z x d, normalised. */
  basis_r[0] = -basis_d[1]; basis_r[1] = basis_d[0]; basis_r[2] = 0.0f;
  const float n = sqrtf(basis_r[0] * basis_r[0] + basis_r[1] * basis_r[1]);
  basis_r[0] /= n; basis_r[1] /= n;
  /* up = d x right. */
  basis_u[0] = basis_d[1] * basis_r[2] - basis_d[2] * basis_r[1];
  basis_u[1] = basis_d[2] * basis_r[0] - basis_d[0] * basis_r[2];
  basis_u[2] = basis_d[0] * basis_r[1] - basis_d[1] * basis_r[0];
  view_scale=cam_scale*fminf((float)w/584.0f,(float)h/431.0f);
  view_cx = (float)w * 0.5f;
  /* The target sits a little below the viewport centre. */
  view_cy = (float)h * 0.58f;
}

static Projected AppLcd3d_Project(const float p[3])
{
  const float v[3] = {p[0] - cam_target[0], p[1] - cam_target[1], p[2] - cam_target[2]};
  Projected out;
  out.sx = view_cx + view_scale * (v[0] * basis_r[0] + v[1] * basis_r[1] + v[2] * basis_r[2]);
  out.sy = view_cy - view_scale * (v[0] * basis_u[0] + v[1] * basis_u[1] + v[2] * basis_u[2]);
  out.depth = v[0] * basis_d[0] + v[1] * basis_d[1] + v[2] * basis_d[2];
  return out;
}

/* ---- Raster primitives (no anti-aliasing, clipped per pixel) ----------- */

static inline void Put(const Canvas *c, int x, int y, uint16_t color)
{
  if (x >= c->clip.x0 && y >= c->clip.y0 && x < c->clip.x1 && y < c->clip.y1)
    c->pixels[(y - c->origin_y) * c->w + x - c->origin_x] = color;
}

static int ClampCoord(float v)
{
  if (v < -2000.0f) return -2000;
  if (v > 4000.0f) return 4000;
  return (int)lroundf(v);
}

/* Bresenham with `thick` parallel copies along the minor axis: no gaps and
 * about thick * length writes. */
static void Line(const Canvas *c, Projected a, Projected b, int thick, uint16_t color)
{
  int x0 = ClampCoord(a.sx), y0 = ClampCoord(a.sy);
  const int x1 = ClampCoord(b.sx), y1 = ClampCoord(b.sy);
  const int half = thick / 2;
  /* Background tiles often contain no part of this grid line.  Reject
   * those before walking Bresenham, preserving its rounding where visible. */
  if ((x0 < x1 ? x1 : x0) + half < c->clip.x0 ||
      (x0 < x1 ? x0 : x1) - half >= c->clip.x1 ||
      (y0 < y1 ? y1 : y0) + half < c->clip.y0 ||
      (y0 < y1 ? y0 : y1) - half >= c->clip.y1) return;
  const int dx = abs(x1 - x0), dy = abs(y1 - y0);
  const int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
  const int steep = dy > dx;
  int err = dx - dy;

  for (;;)
  {
    for (int k = -half; k < thick - half; ++k)
    {
      if (steep) Put(c, x0 + k, y0, color);
      else Put(c, x0, y0 + k, color);
    }
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 < dx) { err += dx; y0 += sy; }
  }
}

static void Disc(const Canvas *c, Projected p, int radius, uint16_t color)
{
  const int cx = ClampCoord(p.sx), cy = ClampCoord(p.sy);
  if (cx + radius < c->clip.x0 || cx - radius >= c->clip.x1 ||
      cy + radius < c->clip.y0 || cy - radius >= c->clip.y1) return;
  for (int dy = -radius; dy <= radius; ++dy)
  {
    int span = 0;
    while ((span + 1) * (span + 1) + dy * dy <= radius * radius) ++span;
    for (int dx = -span; dx <= span; ++dx) Put(c, cx + dx, cy + dy, color);
  }
}

/* factor 0 .. 1 scales the colour's brightness. */
static uint16_t Shade(uint16_t color, float factor)
{
  const uint32_t r = (uint32_t)((float)((color >> 11) & 0x1FU) * factor);
  const uint32_t g = (uint32_t)((float)((color >> 5) & 0x3FU) * factor);
  const uint32_t b = (uint32_t)((float)(color & 0x1FU) * factor);
  return (uint16_t)((r << 11) | (g << 5) | b);
}

/* ---- Grid and tool axes ------------------------------------------------ */

static void AppLcd3d_DrawGrid(const Canvas *c)
{
  for (float v = -C3D_GRID_HALF_M; v <= C3D_GRID_HALF_M + 1e-3f; v += C3D_GRID_STEP_M)
  {
    const float a0[3] = {v, -C3D_GRID_HALF_M, 0.0f}, a1[3] = {v, C3D_GRID_HALF_M, 0.0f};
    const float b0[3] = {-C3D_GRID_HALF_M, v, 0.0f}, b1[3] = {C3D_GRID_HALF_M, v, 0.0f};
    Line(c, AppLcd3d_Project(a0), AppLcd3d_Project(a1), 1, C3D_GRID);
    Line(c, AppLcd3d_Project(b0), AppLcd3d_Project(b1), 1, C3D_GRID);
  }
  /* Base frame axes on the floor: X red, Y green. */
  const float o[3] = {0.0f, 0.0f, 0.0f}, x[3] = {0.15f, 0.0f, 0.0f}, y[3] = {0.0f, 0.15f, 0.0f};
  Line(c, AppLcd3d_Project(o), AppLcd3d_Project(x), 2, C3D_AXIS_X);
  Line(c, AppLcd3d_Project(o), AppLcd3d_Project(y), 2, C3D_AXIS_Y);
  Disc(c, AppLcd3d_Project(o), 10, C3D_BASE);
}

/* Projected flange point and axis tips, computed once per frame. */
static Projected tool_points[4];

static void AppLcd3d_PrepareToolAxes(const float q[ARM_IK_AXES])
{
  float joints[ARM_IK_AXES + 1U][3], tool[3][3];

  ArmIk_JointPoints(q, joints, tool);
  const float *f = joints[ARM_IK_AXES];
  tool_points[0] = AppLcd3d_Project(f);
  for (uint8_t a = 0U; a < 3U; ++a)
  {
    const float tip[3] = {f[0] + tool[0][a] * C3D_TOOL_AXIS_M, f[1] + tool[1][a] * C3D_TOOL_AXIS_M,
                          f[2] + tool[2][a] * C3D_TOOL_AXIS_M};
    tool_points[a + 1U] = AppLcd3d_Project(tip);
  }
}

static void AppLcd3d_DrawToolAxes(const Canvas *c)
{
  const uint16_t colors[3] = {C3D_AXIS_X, C3D_AXIS_Y, C3D_AXIS_Z};
  for (uint8_t a = 0U; a < 3U; ++a) Line(c, tool_points[0], tool_points[a + 1U], 3, colors[a]);
}

/* ---- Solid model: painter's sort, flat shading, both sides ------------- */

typedef struct
{
  uint16_t a, b, c;   /* indices into solid_vertices */
  uint16_t color;
} Face;

/* Depth is sorted over a fixed range around the camera target, which every
 * point of the arm lies in; 4096 buckets = 0.6 mm. */
#define C3D_BUCKETS      4096U
#define C3D_DEPTH_RANGE  1.2f
#define C3D_SHADES       32U

/* CPU-only geometry in cacheable AXI SRAM, sized by the generated mesh;
 * faces are stored already in back-to-front order. */
static Projected solid_vertices[APP_LCD_MESH_TOTAL_VERTICES] __attribute__((section(".lcd_ram")));
static Face solid_faces[APP_LCD_MESH_TOTAL_TRIANGLES] __attribute__((section(".lcd_ram")));
static uint16_t solid_bucket[C3D_BUCKETS + 1U] __attribute__((section(".lcd_ram")));
static uint32_t solid_face_count;

/* Rounds to nearest for the coordinates the fill uses (callers reject
 * values below -0.5 first, where truncation would differ). */
static inline int RoundPos(float v)
{
  return (int)(v + 0.5f);
}

/* GCC's aliasing type permits aligned pairs of RGB565 pixels without
 * disabling strict aliasing for the renderer. */
typedef uint32_t PixelPair __attribute__((__may_alias__));

static inline void FillSpan(const Canvas *c, int y, float xa, float xb,
                            float min_x, float max_x, uint16_t color)
{
  const float lo = fmaxf(fminf(xa, xb), min_x), hi = fminf(fmaxf(xa, xb), max_x);
  if (hi < -0.5f) return;
  int x0 = (lo < 0.0f) ? 0 : RoundPos(lo);
  int x1 = RoundPos(hi);
  if (x0 < c->clip.x0) x0 = c->clip.x0;
  if (x1 >= c->clip.x1) x1 = c->clip.x1 - 1;
  if (x0 > x1) return;

  uint16_t *dst = &c->pixels[(y - c->origin_y) * c->w + x0 - c->origin_x];
  int count = x1 - x0 + 1;
  /* Most tiny faces have short spans; avoid the pair setup in that case. */
  if (count < 4)
  {
    while (count-- > 0) *dst++ = color;
    return;
  }
  if (((uintptr_t)dst & 2U) != 0U)
  {
    *dst++ = color;
    --count;
  }
  const uint32_t pair = (uint32_t)color | ((uint32_t)color << 16);
  PixelPair *out = (PixelPair *)dst;
  while (count >= 2)
  {
    *out++ = pair;
    count -= 2;
  }
  if (count != 0) *(uint16_t *)out = color;
}

/* Walk a half triangle with additions.  Rebase both edges at each half
 * to limit accumulation error; the rounded boundary rows are handled
 * separately so a flat edge is never extrapolated. */
static void FillTriangleRows(const Canvas *c, Projected top, float long_slope,
                              Projected edge, float short_slope, int y, int end,
                              float min_x, float max_x, uint16_t color)
{
  /* Start each half at the same row regardless of its tile's clip.  Walking
   * skipped rows preserves exactly the additions/rounding of a full canvas;
   * rebasing at a tile boundary can otherwise move an edge by one pixel. */
  int row = edge.sy <= 0.0f ? 0 : (int)edge.sy;
  if ((float)row < edge.sy) ++row;
  float xa = top.sx + long_slope * ((float)row - top.sy);
  float xb = edge.sx + short_slope * ((float)row - edge.sy);
  for (; row < y; ++row) { xa += long_slope; xb += short_slope; }
  for (; y <= end; ++y)
  {
    FillSpan(c, y, xa, xb, min_x, max_x, color);
    xa += long_slope;
    xb += short_slope;
  }
}

/* Scanline fill; span ends keep the original nearest-pixel rounding so
 * neighbouring triangles of the decimated mesh leave no pinholes. */
static void FillTriangle(const Canvas *c, Projected p0, Projected p1, Projected p2, uint16_t color)
{
  Projected t;
  if (p1.sy < p0.sy) { t = p0; p0 = p1; p1 = t; }
  if (p2.sy < p0.sy) { t = p0; p0 = p2; p2 = t; }
  if (p2.sy < p1.sy) { t = p1; p1 = p2; p2 = t; }
  if (p2.sy < (float)c->clip.y0 - 0.5f || p0.sy > (float)c->clip.y1) return;

  const float min_x = fminf(p0.sx, fminf(p1.sx, p2.sx));
  const float max_x = fmaxf(p0.sx, fmaxf(p1.sx, p2.sx));
  if (max_x < (float)c->clip.x0 - 0.5f || min_x > (float)c->clip.x1) return;
  const float d02 = p2.sy - p0.sy, d01 = p1.sy - p0.sy, d12 = p2.sy - p1.sy;
  const float s02 = (d02 > 1e-6f) ? (p2.sx - p0.sx) / d02 : 0.0f;
  const float s01 = (d01 > 1e-6f) ? (p1.sx - p0.sx) / d01 : 0.0f;
  const float s12 = (d12 > 1e-6f) ? (p2.sx - p1.sx) / d12 : 0.0f;

  int y = (p0.sy < 0.0f) ? 0 : RoundPos(p0.sy);
  int y_end = RoundPos(p2.sy);
  if (y < c->clip.y0) y = c->clip.y0;
  if (y_end >= c->clip.y1) y_end = c->clip.y1 - 1;
  if (y > y_end) return;

  if ((float)y < p0.sy)
  {
    const float xb = (p0.sy < p1.sy) ? p0.sx : p1.sx + s12 * (p0.sy - p1.sy);
    FillSpan(c, y++, p0.sx, xb, min_x, max_x, color);
  }
  if (y > y_end) return;

  const int bottom_row = ((float)y_end > p2.sy);
  const int interior_end = y_end - bottom_row;
  /* First integer row on the lower edge (ceil for positive screen y). */
  int split = (p1.sy <= 0.0f) ? 0 : (int)p1.sy;
  if ((float)split < p1.sy) ++split;
  const int upper_end = (split - 1 < interior_end) ? split - 1 : interior_end;
  if (y <= upper_end)
  {
    FillTriangleRows(c, p0, s02, p0, s01, y, upper_end, min_x, max_x, color);
    y = upper_end + 1;
  }
  if (y <= interior_end)
    FillTriangleRows(c, p0, s02, p1, s12, y, interior_end, min_x, max_x, color);
  if (bottom_row)
    FillSpan(c, y_end, p0.sx + s02 * d02, p1.sx + s12 * d12, min_x, max_x, color);
}

static inline uint32_t FaceBucket(const uint16_t *tri, uint32_t base)
{
  const float depth = (solid_vertices[base + tri[0]].depth + solid_vertices[base + tri[1]].depth +
                       solid_vertices[base + tri[2]].depth) * (1.0f / 3.0f);
  int32_t b = (int32_t)((depth + C3D_DEPTH_RANGE) * ((float)C3D_BUCKETS / (2.0f * C3D_DEPTH_RANGE)));
  if (b < 0) b = 0;
  if (b >= (int32_t)C3D_BUCKETS) b = (int32_t)C3D_BUCKETS - 1;
  return (uint32_t)b;
}

/* Projects every vertex, then stores the faces sorted back to front by a
 * counting sort on depth.  Normals come precomputed per face in the link
 * frame, so lighting is a dot product with the light and view directions
 * turned into that frame once per link. */
static void AppLcd3d_PrepareSolid(const float q[ARM_IK_AXES])
{
  float rotation[ARM_IK_FRAMES][3][3], position[ARM_IK_FRAMES][3];
  float light[3];
  uint16_t shade[C3D_SHADES];
  uint32_t base;
  _Static_assert(APP_LCD_MESH_COUNT == ARM_IK_FRAMES - 1U || APP_LCD_MESH_COUNT == ARM_IK_FRAMES,
                 "base and seven moving links required; terminal TCP mesh is optional");
  const uint8_t meshes = APP_LCD_MESH_COUNT;

  ArmIk_LinkFrames(q, rotation, position);
  /* Light from the camera, a little above and to the left. */
  for (uint8_t k = 0U; k < 3U; ++k) light[k] = basis_d[k] + 0.6f * basis_u[k] - 0.3f * basis_r[k];
  const float ln = sqrtf(light[0] * light[0] + light[1] * light[1] + light[2] * light[2]);
  for (uint8_t k = 0U; k < 3U; ++k) light[k] /= ln;

  /* Pass 1: project the vertices and count faces per depth bucket. */
  for (uint32_t b = 0U; b <= C3D_BUCKETS; ++b) solid_bucket[b] = 0U;
  base = 0U;
  for (uint8_t m = 0U; m < meshes; ++m)
  {
    const AppLcdMesh *mesh = &app_lcd_meshes[m];
    const float (*r)[3] = rotation[m];
    const float *t = position[m];
    for (uint16_t i = 0U; i < mesh->vertex_count; ++i)
    {
      const float v[3] = {mesh->vertices[i][0] * APP_LCD_MESH_UNIT_M,
                          mesh->vertices[i][1] * APP_LCD_MESH_UNIT_M,
                          mesh->vertices[i][2] * APP_LCD_MESH_UNIT_M};
      const float world[3] = {r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2] + t[0],
                              r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2] + t[1],
                              r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2] + t[2]};
      solid_vertices[base + i] = AppLcd3d_Project(world);
    }
    for (uint16_t i = 0U; i < mesh->triangle_count; ++i)
      solid_bucket[FaceBucket(mesh->triangles[i], base) + 1U]++;
    base += mesh->vertex_count;
  }
  for (uint32_t b = 1U; b <= C3D_BUCKETS; ++b) solid_bucket[b] = (uint16_t)(solid_bucket[b] + solid_bucket[b - 1U]);
  solid_face_count = solid_bucket[C3D_BUCKETS];

  /* Pass 2: shade each face and place it at its sorted position. */
  base = 0U;
  for (uint8_t m = 0U; m < meshes; ++m)
  {
    const AppLcdMesh *mesh = &app_lcd_meshes[m];
    const float (*r)[3] = rotation[m];
    /* Light and view directions in the link frame (R^T v), scaled for the
     * normals' 1/127 units. */
    float ll[3], dl[3];
    for (uint8_t k = 0U; k < 3U; ++k)
    {
      ll[k] = (r[0][k] * light[0] + r[1][k] * light[1] + r[2][k] * light[2]) * (1.0f / 127.0f);
      dl[k] = r[0][k] * basis_d[0] + r[1][k] * basis_d[1] + r[2][k] * basis_d[2];
    }
    for (uint32_t k = 0U; k < C3D_SHADES; ++k)
      shade[k] = Shade(mesh->color, 0.30f + 0.70f * (float)k / (float)(C3D_SHADES - 1U));

    for (uint16_t i = 0U; i < mesh->triangle_count; ++i)
    {
      const uint16_t *tri = mesh->triangles[i];
      const int8_t *n = mesh->normals[i];
      /* The decimated meshes do not keep a reliable winding, so both sides
       * are drawn and lit as if facing the camera; the depth sort hides
       * what is behind. */
      const float facing = n[0] * dl[0] + n[1] * dl[1] + n[2] * dl[2];
      float lit = n[0] * ll[0] + n[1] * ll[1] + n[2] * ll[2];
      if (facing < 0.0f) lit = -lit;
      int32_t level = (int32_t)(lit * (float)(C3D_SHADES - 1U) + 0.5f);
      if (level < 0) level = 0;
      if (level > (int32_t)C3D_SHADES - 1) level = (int32_t)C3D_SHADES - 1;

      Face *f = &solid_faces[solid_bucket[FaceBucket(tri, base)]++];
      f->a = (uint16_t)(base + tri[0]);
      f->b = (uint16_t)(base + tri[1]);
      f->c = (uint16_t)(base + tri[2]);
      f->color = shade[level];
    }
    base += mesh->vertex_count;
  }
}

/* ---- Frame ------------------------------------------------------------- */

static uint8_t RectEmpty(Rect r)
{
  return r.x0 >= r.x1 || r.y0 >= r.y1;
}

static Rect RectUnion(Rect a, Rect b)
{
  if (RectEmpty(a)) return b;
  if (RectEmpty(b)) return a;
  const Rect r = {a.x0 < b.x0 ? a.x0 : b.x0, a.y0 < b.y0 ? a.y0 : b.y0,
                  a.x1 > b.x1 ? a.x1 : b.x1, a.y1 > b.y1 ? a.y1 : b.y1};
  return r;
}

static uint32_t RectPixels(Rect r)
{
  return RectEmpty(r) ? 0U : (uint32_t)(r.x1 - r.x0) * (uint32_t)(r.y1 - r.y0);
}

/* Rotation only changes the grid/base and the model.  Pixels outside both
 * the old and new scene bounds remain background, even while orbiting. */
static Rect AppLcd3d_GridBounds(uint16_t w, uint16_t h)
{
  const float origin[3] = {0.0f, 0.0f, 0.0f};
  const Projected o = AppLcd3d_Project(origin);
  float x0 = o.sx - 12.0f, y0 = o.sy - 12.0f;
  float x1 = o.sx + 13.0f, y1 = o.sy + 13.0f;
  for (unsigned n = 0U; n < 4U; ++n)
  {
    const float corner[3] = {(n & 1U) ? C3D_GRID_HALF_M : -C3D_GRID_HALF_M,
                             (n & 2U) ? C3D_GRID_HALF_M : -C3D_GRID_HALF_M, 0.0f};
    const Projected p = AppLcd3d_Project(corner);
    x0 = fminf(x0, p.sx - 2.0f); y0 = fminf(y0, p.sy - 2.0f);
    x1 = fmaxf(x1, p.sx + 3.0f); y1 = fmaxf(y1, p.sy + 3.0f);
  }
  const Rect r = {(int16_t)fminf((float)w, fmaxf(0.0f, x0)),
                  (int16_t)fminf((float)h, fmaxf(0.0f, y0)),
                  (int16_t)fminf((float)w, fmaxf(0.0f, x1)),
                  (int16_t)fminf((float)h, fmaxf(0.0f, y1))};
  return r;
}

/* The grid is static for a fixed camera.  Cover every model vertex and
 * tool-axis tip, with a margin for nearest-pixel rounding and thick lines.
 * Include the previous bounds too, so moving away restores the old grid. */
static Rect AppLcd3d_ModelBounds(uint16_t w, uint16_t h)
{
  float x0 = tool_points[0].sx, x1 = x0;
  float y0 = tool_points[0].sy, y1 = y0;
  for (uint32_t n = 0U; n < APP_LCD_MESH_TOTAL_VERTICES + 4U; ++n)
  {
    const Projected *p = n < APP_LCD_MESH_TOTAL_VERTICES ? &solid_vertices[n] :
                         &tool_points[n - APP_LCD_MESH_TOTAL_VERTICES];
    x0 = fminf(x0, p->sx); x1 = fmaxf(x1, p->sx);
    y0 = fminf(y0, p->sy); y1 = fmaxf(y1, p->sy);
  }
  const Rect r = {(int16_t)fminf((float)w, fmaxf(0.0f, x0 - 2.0f)),
                  (int16_t)fminf((float)h, fmaxf(0.0f, y0 - 2.0f)),
                  (int16_t)fminf((float)w, fmaxf(0.0f, x1 + 3.0f)),
                  (int16_t)fminf((float)h, fmaxf(0.0f, y1 + 3.0f))};
  return r;
}

/* Complete and submit one independent rectangle; geometry is prepared
 * once for the entire frame, and each display pixel is submitted once. */
static void AppLcd3d_DrawRegion(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                               Rect clip, uint8_t draw_model)
{
  Canvas canvas = {AppLcd_Scratch(), (int16_t)w, (int16_t)h, clip, 0, 0};
  const uint16_t dirty_w = (uint16_t)(canvas.clip.x1 - canvas.clip.x0);
  const uint16_t dirty_h = (uint16_t)(canvas.clip.y1 - canvas.clip.y0);
  const uint32_t dirty_pixels = (uint32_t)dirty_w * dirty_h;
  uint8_t use_sram = 0U;
#if APP_LCD_FAST_BUFFER
  if (dirty_pixels <= C3D_FAST_PIXELS)
  {
    canvas.pixels = fast_pixels;
    canvas.w = (int16_t)dirty_w;
    canvas.h = (int16_t)dirty_h;
    canvas.origin_x = canvas.clip.x0;
    canvas.origin_y = canvas.clip.y0;
    use_sram = 1U;
  }
#endif
  uint16_t *const source = &canvas.pixels[(uint32_t)(canvas.clip.y0 - canvas.origin_y) * canvas.w
                                          + canvas.clip.x0 - canvas.origin_x];
  const uint32_t t1 = DWT->CYCCNT;

#if APP_LCD_SCRATCH_CACHE || APP_LCD_FAST_BUFFER
  const uintptr_t cache_start = (uintptr_t)source & ~(uintptr_t)31U;
  const uintptr_t cache_end = ((uintptr_t)(source + (uint32_t)(dirty_h - 1U) * canvas.w + dirty_w)
                                + 31U) & ~(uintptr_t)31U;
  const int32_t cache_bytes = (int32_t)(cache_end - cache_start);
#endif
  if (use_sram)
  {
    /* A compact, aligned CPU fill keeps SRAM cache lines available to the
     * rasterizer and needs no invalidate/DMA ownership round trip. */
    PixelPair *dst = (PixelPair *)source;
    const uint32_t bg = (uint32_t)C3D_BG | ((uint32_t)C3D_BG << 16);
    for (uint32_t n = 0U; n < dirty_pixels / 2U; ++n) dst[n] = bg;
    if ((dirty_pixels & 1U) != 0U) source[dirty_pixels - 1U] = C3D_BG;
  }
  else
  {
#if APP_LCD_SCRATCH_CACHE
    /* Enclose the strided SDRAM rectangle in whole cache lines.  All CPU
     * writes were cleaned at the last use; untouched neighbours are valid
     * in RAM, even if intervening frames used the independent SRAM buffer. */
    SCB_InvalidateDCache_by_Addr((void *)cache_start, cache_bytes);
#endif
    AppLcd_FillRectIn(canvas.pixels, w, (uint16_t)canvas.clip.x0, (uint16_t)canvas.clip.y0,
                      dirty_w, dirty_h, C3D_BG);
#if APP_LCD_SCRATCH_CACHE
    SCB_InvalidateDCache_by_Addr((void *)cache_start, cache_bytes);
#endif
  }
  const uint32_t t2 = DWT->CYCCNT;
  AppLcd3d_DrawGrid(&canvas);
  if (draw_model)
  {
    for (uint32_t n = 0U; n < solid_face_count; ++n)
    {
      const Face *f = &solid_faces[n];
      FillTriangle(&canvas, solid_vertices[f->a], solid_vertices[f->b], solid_vertices[f->c], f->color);
    }
    AppLcd3d_DrawToolAxes(&canvas);
  }
  const uint32_t t3 = DWT->CYCCNT;
#if APP_LCD_SCRATCH_CACHE || APP_LCD_FAST_BUFFER
  /* AXI SRAM remains cacheable even when the SDRAM cache A/B switch is off. */
  if (use_sram || APP_LCD_SCRATCH_CACHE)
    SCB_CleanDCache_by_Addr((void *)cache_start, cache_bytes);
#endif
  const uint32_t t4 = DWT->CYCCNT;
  AppLcd_Blit(source, (uint16_t)canvas.w, (uint16_t)(x + canvas.clip.x0), (uint16_t)(y + canvas.clip.y0),
              dirty_w, dirty_h);
  const uint32_t t5 = DWT->CYCCNT;

  frame_timing.clear_cycles += t2 - t1;
  frame_timing.raster_cycles += t3 - t2;
  frame_timing.clean_cycles += t4 - t3;
  frame_timing.blit_cycles += t5 - t4;
  frame_timing.updated_pixels += dirty_pixels;
  frame_timing.sram_buffer &= use_sram;
  ++frame_timing.tiles;
}

static void AppLcd3d_DrawTiles(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                              Rect region, uint8_t draw_model)
{
  if (RectEmpty(region)) return;
#if APP_LCD_FAST_BUFFER
  const uint32_t rows = C3D_FAST_PIXELS / (uint32_t)(region.x1 - region.x0);
  for (int32_t top = region.y0; top < region.y1; top += (int32_t)rows)
  {
    const int32_t bottom = top + (int32_t)rows;
    const Rect tile = {region.x0, (int16_t)top, region.x1,
                       (int16_t)(bottom < region.y1 ? bottom : region.y1)};
    AppLcd3d_DrawRegion(x, y, w, h, tile, draw_model);
  }
#else
  AppLcd3d_DrawRegion(x, y, w, h, region, draw_model);
#endif
}

uint8_t AppLcd3d_Render(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  float measured[ARM_IK_AXES];
  uint8_t moved = 0U;
  if (w == 0U || h == 0U || w > INT16_MAX || h > INT16_MAX ||
      (uint32_t)w * h > APP_LCD_SCRATCH_PIXELS) return 0U;
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i)
  {
    measured[i] = (dm_motors[i].position - dm_motor_runtime.zero_offset[i]) * DM_MOTOR_DIRECTION(i);
    if (fabsf(measured[i] - drawn_q[i]) > C3D_STILL_RAD) moved = 1U;
  }
  const uint8_t invalid = !drawn_valid || drawn_x != x || drawn_y != y || drawn_w != w || drawn_h != h;
  const uint8_t camera_changed = drawn_yaw != cam_yaw || drawn_pitch != cam_pitch;
  if (!moved && !invalid && !camera_changed) return 0U;

  const uint32_t t0 = DWT->CYCCNT;
  AppLcd3d_BuildCamera((int16_t)w, (int16_t)h);
  AppLcd3d_PrepareSolid(measured);
  AppLcd3d_PrepareToolAxes(measured);
  const Rect bounds = AppLcd3d_ModelBounds(w, h);
  const Rect scene_bounds = RectUnion(bounds, AppLcd3d_GridBounds(w, h));
  Rect dirty = {0, 0, (int16_t)w, (int16_t)h};
  if (APP_LCD_PARTIAL_REFRESH && !invalid)
    dirty = camera_changed ? RectUnion(scene_bounds, drawn_scene_bounds) : RectUnion(bounds, drawn_bounds);
  for (uint8_t i = 0U; i < ARM_IK_AXES; ++i) drawn_q[i] = measured[i];
  drawn_yaw = cam_yaw; drawn_pitch = cam_pitch;
  drawn_x = x; drawn_y = y; drawn_w = w; drawn_h = h;
  drawn_bounds = bounds; drawn_scene_bounds = scene_bounds;
  drawn_valid = 1U;
  if (RectEmpty(dirty)) return 0U;

  frame_timing = (AppLcd3dTiming){0};
  frame_timing.sram_buffer = APP_LCD_FAST_BUFFER;
  const uint32_t t1 = DWT->CYCCNT;
  frame_timing.prepare_cycles = t1 - t0;
  if (!APP_LCD_FAST_BUFFER || RectPixels(dirty) <= C3D_FAST_PIXELS || RectEmpty(bounds))
    AppLcd3d_DrawTiles(x, y, w, h, dirty, !RectEmpty(bounds));
  else
  {
    /* Keep the model together instead of traversing every face for every
     * full-width strip.  Four disjoint surrounding rectangles contain only
     * background/grid, including the old model position that must be erased.
     * Submit the expensive model first, then finish the background quickly. */
    AppLcd3d_DrawTiles(x, y, w, h, bounds, 1U);
    AppLcd3d_DrawTiles(x, y, w, h, (Rect){dirty.x0, dirty.y0, dirty.x1, bounds.y0}, 0U);
    AppLcd3d_DrawTiles(x, y, w, h, (Rect){dirty.x0, bounds.y1, dirty.x1, dirty.y1}, 0U);
    AppLcd3d_DrawTiles(x, y, w, h, (Rect){dirty.x0, bounds.y0, bounds.x0, bounds.y1}, 0U);
    AppLcd3d_DrawTiles(x, y, w, h, (Rect){bounds.x1, bounds.y0, dirty.x1, bounds.y1}, 0U);
  }
  frame_timing.draw_cycles = DWT->CYCCNT - t1;
  return 1U;
}
