// flight_sim_multiplayer_logging.c
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265f
#endif
#define WIDTH 1024
#define HEIGHT 768
#define DT 0.016f
#define MAX_ENEMIES 2
#define MAX_PLAYERS 2
#define TEX_SIZE 64
#define MSG_PLAYER_DATA  0
#define MSG_CONNECT      1
#define MSG_TERRAIN_SEED 2
#define MSG_TORNADO      3
#define MSG_EXPLOSION    4
// Turbulence is now dynamic — see turbulenceLevel global
#define TURBULENCE_MAX 0.004f   // peak per-frame angular kick (was 0.01)
#define GRAVITY 9.8f
#define TERRAIN_SIZE 256       // grid cells per side (must be power-of-2)
#define TERRAIN_SCALE 2.0f     // world units per cell => 512x512 world
#define TERRAIN_HEIGHT 22.0f   // max terrain height
#define MAX_AIRPORTS 5

typedef enum { PLANE_FIGHTER=0, PLANE_PROP=1, PLANE_AIRLINER=2 } PlaneType;
typedef struct {
    float maxSpeed, takeoffSpeed, stallSpeed, brakeDecel, accel;
    float scale;    // visual scale multiplier
    const char *name;
} PlaneStats;
static const PlaneStats PLANE_DEFS[3] = {
    /* FIGHTER  */ { 18.0f, 6.0f,  3.5f, 4.0f, 6.0f, 1.0f, "F/A-18 Hornet"   },
    /* PROP     */ {  9.0f, 3.5f,  2.0f, 3.0f, 4.0f, 0.7f, "Cessna 172"      },
    /* AIRLINER */ { 14.0f, 8.0f,  5.0f, 3.5f, 3.0f, 2.2f, "Boeing 737"      },
};

typedef struct { float x,y,z; float vx,vy,vz; float pitch,yaw,roll; float throttle; PlaneType type; } Plane;
typedef struct { float x,y,z; bool alive; } EnemyPlane;
typedef struct {
    float x,y,z,pitch,yaw,roll,throttle;
    float walkerX,walkerY,walkerZ,walkerYaw;
    bool  alive;
    bool  inPlane;
    PlaneType type;
    bool  isPassenger;   // riding as passenger in pilot's plane
} RemotePlane;
typedef struct {
    float wx, wz;     // world centre
    float heading;    // runway heading in radians
    float runwayLen;  // half-length of runway
    float runwayW;    // half-width
    float groundY;    // flattened elevation
    int   size;       // 0=small, 1=medium, 2=large
    int   numGates;   // how many gate spots were placed
} Airport;
Airport airports[MAX_AIRPORTS];
int numAirports = 0;

// Parked planes at gates
#define MAX_PARKED 20
typedef struct { float wx,wy,wz, heading; bool active; PlaneType type; } ParkedPlane;
ParkedPlane parkedPlanes[MAX_PARKED];
int numParked = 0;

Plane player;
EnemyPlane enemies[MAX_ENEMIES];
RemotePlane remotePlayers[MAX_PLAYERS-1];

Display *dpy;
Window win;
GLXContext glc;
bool running=true;
bool keys[65536];
GLuint groundTexture;

// Landing gear
float gearExtension = 1.0f;   // 0.0 = fully retracted, 1.0 = fully deployed
bool  gearDeployed  = true;    // target state
bool  gearKeyHeld   = false;   // debounce

// Takeoff / landing state
bool  onGround      = true;
float airspeed      = 0.0f;   // horizontal speed magnitude
// Per-type stats accessed via PLANE_DEFS[player.type]
#define TAKEOFF_SPEED  (PLANE_DEFS[player.type].takeoffSpeed)
#define STALL_SPEED    (PLANE_DEFS[player.type].stallSpeed)
#define BRAKE_DECEL    (PLANE_DEFS[player.type].brakeDecel)
#define MAX_SPEED      (PLANE_DEFS[player.type].maxSpeed)

// Passenger system
bool isPassenger   = false;  // local player is riding as passenger
bool hasPassenger  = false;  // local player (airliner pilot) has a passenger aboard
// On-foot walker
bool  inPlane      = true;
float walkerX      = 0.0f;
float walkerY      = 0.0f;
float walkerZ      = 0.0f;
float walkerYaw    = 0.0f;
bool  enterKeyHeld = false;
#define WALKER_SPEED   5.5f
#define ENTER_DIST     6.0f   // max distance to enter a plane
#define WALKER_EYE_H   1.7f   // eye height above ground

// Map
bool mapOpen     = false;
bool mapKeyHeld  = false;
char airportNames[MAX_AIRPORTS][32];

// Turbulence — slowly varying intensity [0,1]
float turbulenceLevel = 0.3f;   // current intensity
float turbulenceTarget = 0.3f;  // drifts toward this
float turbulenceTimer  = 0.0f;  // time until next target change

// ---- Menu ----
typedef enum { MENU_MAIN, MENU_SETTINGS, MENU_JOIN, MENU_PLAYING } MenuState;
MenuState menuState = MENU_MAIN;
bool gameStarted = false;

// Settings
int   fpsCap      = 60;
bool  fullscreen  = false;
int   resWidth    = 1024;
int   resHeight   = 768;
static const int resOptions[][2] = {{800,600},{1024,768},{1280,720},{1920,1080}};
#define NUM_RES_OPTIONS 4
int   resIndex    = 1;  // default 1024x768

// Join-server IP input
char  joinIP[64]  = "127.0.0.1";
int   joinIPLen   = 9;
bool  joinIPFocus = false;
bool  joinFailed  = false;   // true when connect attempt timed out
float joinTimer   = 0.0f;    // seconds since connect was clicked
#define JOIN_TIMEOUT 5.0f    // seconds before showing "no server found"

// ---- Weather ----
typedef enum { WX_CLEAR=0, WX_CLOUDY, WX_OVERCAST, WX_STORM } WeatherType;
typedef struct {
    WeatherType type;
    float skyR, skyG, skyB;   // clear-sky colour
    float fogDensity;          // GL_EXP2 density
    float windX, windZ;        // world-space wind (units/s)
    float rainIntensity;       // 0=none, 1=heavy
    float transitionTimer;     // seconds until next weather change
    float transitionSpeed;     // lerp speed toward targets
} Weather;
Weather weather;

// Rain particles
#define MAX_RAIN 1200
typedef struct { float x,y,z; float speed; } RainDrop;
RainDrop rain[MAX_RAIN];
bool rainInited = false;

// Explosions
#define MAX_EXPLOSIONS 4
#define EXPLOSION_LIFETIME 2.2f
#define EXPLOSION_PARTICLES 48
typedef struct {
    float x, y, z;
    float timer;        // counts up from 0 to EXPLOSION_LIFETIME
    bool  active;
    // Per-particle direction vectors (unit sphere) and speed
    float px[EXPLOSION_PARTICLES];
    float py[EXPLOSION_PARTICLES];
    float pz[EXPLOSION_PARTICLES];
    float ps[EXPLOSION_PARTICLES]; // speed scalar
} Explosion;
static Explosion explosions[MAX_EXPLOSIONS];

// Outbound explosion queue (written by physics, flushed by network thread)
#define MAX_EXPLODE_QUEUE 16
typedef struct { float x,y,z,scale; } ExplodeQueueEntry;
static ExplodeQueueEntry explodeQueue[MAX_EXPLODE_QUEUE];
static int explodeQueueHead = 0, explodeQueueTail = 0;
static pthread_mutex_t explodeMutex = PTHREAD_MUTEX_INITIALIZER;

static void queueExplosionBroadcast(float x, float y, float z, float scale){
    pthread_mutex_lock(&explodeMutex);
    int next = (explodeQueueTail+1) % MAX_EXPLODE_QUEUE;
    if(next != explodeQueueHead){  // drop if full
        explodeQueue[explodeQueueTail] = (ExplodeQueueEntry){x,y,z,scale};
        explodeQueueTail = next;
    }
    pthread_mutex_unlock(&explodeMutex);
}

static void triggerExplosionEx(float x, float y, float z, float speedScale){
    for(int i=0;i<MAX_EXPLOSIONS;i++){
        if(!explosions[i].active){
            explosions[i].x = x; explosions[i].y = y; explosions[i].z = z;
            explosions[i].timer = 0.0f;
            explosions[i].active = true;
            for(int p=0;p<EXPLOSION_PARTICLES;p++){
                float th = ((float)rand()/RAND_MAX)*2.0f*(float)M_PI;
                float ph = ((float)rand()/RAND_MAX)*(float)M_PI;
                explosions[i].px[p] = sinf(ph)*cosf(th);
                explosions[i].py[p] = fabsf(cosf(ph));
                explosions[i].pz[p] = sinf(ph)*sinf(th);
                explosions[i].ps[p] = (3.0f + ((float)rand()/RAND_MAX)*5.0f) * speedScale;
            }
            queueExplosionBroadcast(x, y, z, speedScale);
            return;
        }
    }
}
static void triggerExplosion(float x, float y, float z){ triggerExplosionEx(x,y,z,1.0f); }

// Called by network thread to spawn a remotely-triggered explosion (no re-broadcast)
static void receiveExplosion(float x, float y, float z, float speedScale){
    for(int i=0;i<MAX_EXPLOSIONS;i++){
        if(!explosions[i].active){
            explosions[i].x = x; explosions[i].y = y; explosions[i].z = z;
            explosions[i].timer = 0.0f;
            explosions[i].active = true;
            for(int p=0;p<EXPLOSION_PARTICLES;p++){
                float th = ((float)rand()/RAND_MAX)*2.0f*(float)M_PI;
                float ph = ((float)rand()/RAND_MAX)*(float)M_PI;
                explosions[i].px[p] = sinf(ph)*cosf(th);
                explosions[i].py[p] = fabsf(cosf(ph));
                explosions[i].pz[p] = sinf(ph)*sinf(th);
                explosions[i].ps[p] = (3.0f + ((float)rand()/RAND_MAX)*5.0f) * speedScale;
            }
            return;
        }
    }
}

// Tornadoes
#define MAX_TORNADOS 3
typedef struct {
    float x, z;          // world position
    float vx, vz;        // movement velocity
    float angle;         // rotation phase
    float height;        // current visual height (animates up)
    bool  active;
} Tornado;
Tornado tornados[MAX_TORNADOS];
bool tornadoEnabled = false;   // toggled by 't' in terminal
pthread_mutex_t tornadoMutex  = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t remoteMutex = PTHREAD_MUTEX_INITIALIZER;

// Networking
int sockfd;
struct sockaddr_in otherAddr;
bool isServer = true;
bool clientConnected = false;  // server-side: tracks if a client addr is known
int packetsSent = 0, packetsRecv = 0;
unsigned int terrainSeed = 0;  // server picks this; client receives it
bool seedReceived = false;     // client: true once terrain seed has arrived from server
pthread_mutex_t playerMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t terrainMutex = PTHREAD_MUTEX_INITIALIZER;

static float clampf(float v,float lo,float hi){ return v<lo?lo:v>hi?hi:v; }

// FPS
int frames=0;
float fps=0.0f;
double lastTime=0.0;

// -------- Plane Model --------

// Octagonal cross-section ring at z with given rx (half-width) and ry (half-height).
// Points go: right, upper-right, top, upper-left, left, lower-left, bottom, lower-right
static void octRing(float rx, float ry, float z, float v[8][3]){
    float dx = rx*0.707f, dy = ry*0.707f;
    float pts[8][2] = {
        { rx,   0  }, { dx,  dy }, {  0,  ry }, {-dx,  dy },
        {-rx,   0  }, {-dx, -dy }, {  0, -ry }, { dx, -dy }
    };
    for(int i=0;i<8;i++){ v[i][0]=pts[i][0]; v[i][1]=pts[i][1]; v[i][2]=z; }
}

// Extrude two octagonal rings into 8 quads with outward normals.
static void drawOctSegment(float rx0,float ry0,float z0, float rx1,float ry1,float z1){
    float a[8][3], b[8][3];
    octRing(rx0,ry0,z0,a);
    octRing(rx1,ry1,z1,b);
    for(int i=0;i<8;i++){
        int j=(i+1)%8;
        // outward normal: average of corner directions
        float nx = (a[i][0]+a[j][0])*0.5f;
        float ny = (a[i][1]+a[j][1])*0.5f;
        float len = sqrtf(nx*nx+ny*ny); if(len>0){nx/=len;ny/=len;}
        glNormal3f(nx,ny,0);
        glBegin(GL_QUADS);
            glVertex3f(a[i][0],a[i][1],a[i][2]);
            glVertex3f(a[j][0],a[j][1],a[j][2]);
            glVertex3f(b[j][0],b[j][1],b[j][2]);
            glVertex3f(b[i][0],b[i][1],b[i][2]);
        glEnd();
    }
}

// Filled octagon cap (fan from centre).
static void drawOctCap(float rx,float ry,float z, float nx,float ny,float nz){
    float v[8][3]; octRing(rx,ry,z,v);
    glNormal3f(nx,ny,nz);
    glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0,0,z);
        if(nz>0) for(int i=0;i<8;i++) glVertex3f(v[i][0],v[i][1],v[i][2]);
        else     for(int i=7;i>=0;i--) glVertex3f(v[i][0],v[i][1],v[i][2]);
    glEnd();
}

// Wing panel: swept trapezoid with aerofoil-like top camber (top face raised 30% at chord mid).
static void drawWingPanel(float rootX, float tipX,
                          float rootLE, float rootTE,  // leading/trailing edge z at root
                          float tipLE,  float tipTE,
                          float thick){
    float ht = thick*0.5f;
    float rMid = (rootLE+rootTE)*0.5f, tMid = (tipLE+tipTE)*0.5f;
    float rCamber = ht*0.4f,  tCamber = ht*0.3f; // top surface raised

    // top surface (two panels for slight camber)
    glNormal3f(0,1,0);
    glBegin(GL_QUADS);
        glVertex3f(rootX, ht+rCamber, rMid);  glVertex3f(rootX, ht, rootLE);
        glVertex3f(tipX,  ht, tipLE);          glVertex3f(tipX,  ht+tCamber, tMid);
    glEnd();
    glBegin(GL_QUADS);
        glVertex3f(rootX, ht+rCamber, rMid);  glVertex3f(tipX,  ht+tCamber, tMid);
        glVertex3f(tipX,  ht, tipTE);          glVertex3f(rootX, ht, rootTE);
    glEnd();
    // bottom surface (flat)
    glNormal3f(0,-1,0);
    glBegin(GL_QUADS);
        glVertex3f(rootX,-ht, rootLE); glVertex3f(rootX,-ht, rootTE);
        glVertex3f(tipX, -ht, tipTE);  glVertex3f(tipX, -ht, tipLE);
    glEnd();
    // leading edge
    glNormal3f(0,0,1);
    glBegin(GL_QUADS);
        glVertex3f(rootX,-ht, rootLE); glVertex3f(tipX, -ht, tipLE);
        glVertex3f(tipX,  ht, tipLE);  glVertex3f(rootX, ht, rootLE);
    glEnd();
    // trailing edge
    glNormal3f(0,0,-1);
    glBegin(GL_QUADS);
        glVertex3f(rootX,-ht, rootTE); glVertex3f(rootX, ht, rootTE);
        glVertex3f(tipX,  ht, tipTE);  glVertex3f(tipX, -ht, tipTE);
    glEnd();
    // tip
    float sn = (rootX > 0) ? 1.0f : -1.0f;
    glNormal3f(sn,0,0);
    glBegin(GL_QUADS);
        glVertex3f(tipX,-ht, tipLE); glVertex3f(tipX, ht, tipLE);
        glVertex3f(tipX, ht, tipTE); glVertex3f(tipX,-ht, tipTE);
    glEnd();
}

// Aileron/flap strip along trailing edge of wing.
static void drawControlSurface(float rootX, float tipX,
                                float rootZ, float tipZ,
                                float chord, float thick, float r, float g, float b){
    float ht = thick*0.5f;
    glColor3f(r,g,b);
    glNormal3f(0,1,0);
    glBegin(GL_QUADS);
        glVertex3f(rootX, ht, rootZ);       glVertex3f(rootX, ht, rootZ-chord);
        glVertex3f(tipX,  ht, tipZ-chord);  glVertex3f(tipX,  ht, tipZ);
    glEnd();
    glNormal3f(0,-1,0);
    glBegin(GL_QUADS);
        glVertex3f(rootX,-ht, rootZ);      glVertex3f(tipX, -ht, tipZ);
        glVertex3f(tipX, -ht, tipZ-chord); glVertex3f(rootX,-ht, rootZ-chord);
    glEnd();
    glNormal3f(0,0,-1);
    glBegin(GL_QUADS);
        glVertex3f(rootX,-ht, rootZ-chord); glVertex3f(rootX, ht, rootZ-chord);
        glVertex3f(tipX,  ht, tipZ-chord);  glVertex3f(tipX, -ht, tipZ-chord);
    glEnd();
}

// Simple stick figure for a remote player on foot
static void drawWalkerModel(float x, float y, float z, float yaw){
    glPushMatrix();
    glTranslatef(x, y, z);
    glRotatef(yaw*57.2958f, 0,1,0);
    // Body
    glColor3f(0.2f,0.5f,0.9f);
    glBegin(GL_QUADS);
        // front
        glVertex3f(-0.2f,0.0f, 0.2f); glVertex3f( 0.2f,0.0f, 0.2f);
        glVertex3f( 0.2f,1.4f, 0.2f); glVertex3f(-0.2f,1.4f, 0.2f);
        // back
        glVertex3f( 0.2f,0.0f,-0.2f); glVertex3f(-0.2f,0.0f,-0.2f);
        glVertex3f(-0.2f,1.4f,-0.2f); glVertex3f( 0.2f,1.4f,-0.2f);
        // left
        glVertex3f(-0.2f,0.0f,-0.2f); glVertex3f(-0.2f,0.0f, 0.2f);
        glVertex3f(-0.2f,1.4f, 0.2f); glVertex3f(-0.2f,1.4f,-0.2f);
        // right
        glVertex3f( 0.2f,0.0f, 0.2f); glVertex3f( 0.2f,0.0f,-0.2f);
        glVertex3f( 0.2f,1.4f,-0.2f); glVertex3f( 0.2f,1.4f, 0.2f);
    glEnd();
    // Head
    glColor3f(0.9f,0.75f,0.6f);
    glBegin(GL_QUADS);
        glVertex3f(-0.15f,1.4f, 0.15f); glVertex3f( 0.15f,1.4f, 0.15f);
        glVertex3f( 0.15f,1.75f,0.15f); glVertex3f(-0.15f,1.75f,0.15f);
        glVertex3f( 0.15f,1.4f,-0.15f); glVertex3f(-0.15f,1.4f,-0.15f);
        glVertex3f(-0.15f,1.75f,-0.15f);glVertex3f( 0.15f,1.75f,-0.15f);
        glVertex3f(-0.15f,1.4f,-0.15f); glVertex3f(-0.15f,1.4f, 0.15f);
        glVertex3f(-0.15f,1.75f,0.15f); glVertex3f(-0.15f,1.75f,-0.15f);
        glVertex3f( 0.15f,1.4f, 0.15f); glVertex3f( 0.15f,1.4f,-0.15f);
        glVertex3f( 0.15f,1.75f,-0.15f);glVertex3f( 0.15f,1.75f, 0.15f);
    glEnd();
    glPopMatrix();
}

// ---- Cessna-style prop plane (high wing, single engine) ----
static void drawPropPlane(float gear){
    // Fuselage — boxy, tapers at tail
    glColor3f(0.92f,0.92f,0.88f);
    drawOctSegment(0.00f,0.00f, 0.90f,  0.06f,0.06f, 0.70f);
    drawOctSegment(0.06f,0.06f, 0.70f,  0.18f,0.14f, 0.35f);
    drawOctSegment(0.18f,0.14f, 0.35f,  0.18f,0.13f,-0.40f);
    drawOctSegment(0.18f,0.13f,-0.40f,  0.10f,0.10f,-0.85f);
    drawOctCap(0.00f,0.00f, 0.90f, 0,0, 1);
    drawOctCap(0.10f,0.10f,-0.85f, 0,0,-1);
    // Engine cowl (nose)
    glColor3f(0.70f,0.70f,0.68f);
    drawOctSegment(0.00f,0.00f,1.10f, 0.14f,0.12f,0.90f);
    drawOctCap(0.00f,0.00f,1.10f, 0,0,1);
    // Propeller
    glColor3f(0.25f,0.22f,0.18f);
    for(int b=0;b<2;b++){
        glPushMatrix();
        glRotatef(b*90.0f, 0,0,1);
        glBegin(GL_QUADS);
            glNormal3f(0,0,1);
            glVertex3f(-0.04f, 0.0f, 1.12f); glVertex3f( 0.04f, 0.0f,1.12f);
            glVertex3f( 0.06f, 0.55f,1.12f); glVertex3f(-0.06f,0.55f,1.12f);
            glVertex3f(-0.04f, 0.0f, 1.12f); glVertex3f( 0.04f, 0.0f,1.12f);
            glVertex3f( 0.06f,-0.55f,1.12f); glVertex3f(-0.06f,-0.55f,1.12f);
        glEnd();
        glPopMatrix();
    }
    // High wing (straight, above fuselage)
    glColor3f(0.88f,0.88f,0.84f);
    drawWingPanel( 0.18f, 0.65f,  0.15f, 0.00f, -0.05f,-0.10f, 0.06f);
    drawWingPanel( 0.65f, 1.20f,  0.00f,-0.10f, -0.15f,-0.15f, 0.04f);
    drawWingPanel(-0.18f,-0.65f,  0.15f, 0.00f, -0.05f,-0.10f, 0.06f);
    drawWingPanel(-0.65f,-1.20f,  0.00f,-0.10f, -0.15f,-0.15f, 0.04f);
    // Struts (connect wing root to fuselage sides — solid box beams)
    glColor3f(0.70f,0.70f,0.68f);
    for(int s=-1;s<=1;s+=2){
        float x0=s*0.19f, x1=s*0.54f;
        float y0=-0.10f,  y1= 0.13f;
        float z0=-0.05f,  z1= 0.05f;
        // top
        glBegin(GL_QUADS); glNormal3f(0,1,0);
            glVertex3f(x0,y1,z1); glVertex3f(x1,y1,z1);
            glVertex3f(x1,y1,z0); glVertex3f(x0,y1,z0);
        glEnd();
        // bottom
        glBegin(GL_QUADS); glNormal3f(0,-1,0);
            glVertex3f(x0,y0,z0); glVertex3f(x1,y0,z0);
            glVertex3f(x1,y0,z1); glVertex3f(x0,y0,z1);
        glEnd();
        // front
        glBegin(GL_QUADS); glNormal3f(0,0,1);
            glVertex3f(x0,y0,z1); glVertex3f(x1,y0,z1);
            glVertex3f(x1,y1,z1); glVertex3f(x0,y1,z1);
        glEnd();
        // back
        glBegin(GL_QUADS); glNormal3f(0,0,-1);
            glVertex3f(x0,y1,z0); glVertex3f(x1,y1,z0);
            glVertex3f(x1,y0,z0); glVertex3f(x0,y0,z0);
        glEnd();
        // outer face
        glBegin(GL_QUADS); glNormal3f(s,0,0);
            glVertex3f(x1,y0,z0); glVertex3f(x1,y1,z0);
            glVertex3f(x1,y1,z1); glVertex3f(x1,y0,z1);
        glEnd();
    }
    // Vertical tail — solid fin with leading edge and top cap
    glColor3f(0.88f,0.88f,0.84f);
    for(int side=-1;side<=1;side+=2){
        glNormal3f(side,0,0);
        glBegin(GL_QUADS);
            glVertex3f(side*0.02f,0.10f,-0.55f); glVertex3f(side*0.02f,0.10f,-0.82f);
            glVertex3f(side*0.02f,0.42f,-0.75f); glVertex3f(side*0.02f,0.42f,-0.60f);
        glEnd();
    }
    // leading edge
    glBegin(GL_QUADS); glNormal3f(0,0,1);
        glVertex3f(-0.02f,0.10f,-0.55f); glVertex3f( 0.02f,0.10f,-0.55f);
        glVertex3f( 0.02f,0.42f,-0.60f); glVertex3f(-0.02f,0.42f,-0.60f);
    glEnd();
    // trailing edge
    glBegin(GL_QUADS); glNormal3f(0,0,-1);
        glVertex3f(-0.02f,0.10f,-0.82f); glVertex3f(-0.02f,0.42f,-0.75f);
        glVertex3f( 0.02f,0.42f,-0.75f); glVertex3f( 0.02f,0.10f,-0.82f);
    glEnd();
    // top cap
    glBegin(GL_QUADS); glNormal3f(0,1,0);
        glVertex3f(-0.02f,0.42f,-0.60f); glVertex3f( 0.02f,0.42f,-0.60f);
        glVertex3f( 0.02f,0.42f,-0.75f); glVertex3f(-0.02f,0.42f,-0.75f);
    glEnd();
    // Horizontal stab
    drawWingPanel( 0.02f, 0.38f, -0.60f,-0.82f, -0.65f,-0.84f, 0.03f);
    drawWingPanel(-0.02f,-0.38f, -0.60f,-0.82f, -0.65f,-0.84f, 0.03f);
    // Windshield
    glColor3f(0.18f,0.28f,0.50f);
    glBegin(GL_QUADS);
        glNormal3f(0,0.5f,1);
        glVertex3f(-0.10f,0.14f,0.65f); glVertex3f( 0.10f,0.14f,0.65f);
        glVertex3f( 0.10f,0.28f,0.50f); glVertex3f(-0.10f,0.28f,0.50f);
    glEnd();
    // Simple fixed gear (no retraction on prop)
    glColor3f(0.35f,0.35f,0.38f);
    for(int s=-1;s<=1;s+=2){
        glBegin(GL_QUADS);
            glNormal3f(s,0,0);
            glVertex3f(s*0.05f,0.0f, 0.05f); glVertex3f(s*0.05f,-0.28f, 0.05f);
            glVertex3f(s*0.05f,-0.28f,-0.05f); glVertex3f(s*0.05f,0.0f,-0.05f);
        glEnd();
        // axle
        glBegin(GL_QUADS);
            glNormal3f(0,-1,0);
            glVertex3f(s*0.05f,-0.28f, 0.04f); glVertex3f(s*0.22f,-0.28f, 0.04f);
            glVertex3f(s*0.22f,-0.28f,-0.04f); glVertex3f(s*0.05f,-0.28f,-0.04f);
        glEnd();
    }
    (void)gear;
}

// ---- Boeing 737-style airliner (wide body, 4 engines, passenger windows) ----
static void drawAirliner(float gear){
    // Wide fuselage
    glColor3f(0.93f,0.93f,0.95f);
    drawOctSegment(0.00f,0.00f, 2.20f,  0.10f,0.10f, 1.90f);
    drawOctSegment(0.10f,0.10f, 1.90f,  0.38f,0.30f, 1.40f);
    drawOctSegment(0.38f,0.30f, 1.40f,  0.40f,0.32f,-1.00f);
    drawOctSegment(0.40f,0.32f,-1.00f,  0.28f,0.24f,-1.80f);
    drawOctSegment(0.28f,0.24f,-1.80f,  0.10f,0.12f,-2.30f);
    drawOctCap(0.00f,0.00f, 2.20f, 0,0,1);
    drawOctCap(0.10f,0.12f,-2.30f, 0,0,-1);
    // Passenger windows — offset slightly proud of fuselage to avoid z-fighting
    glColor3f(0.15f,0.25f,0.55f);
    for(int w=0;w<10;w++){
        float wz = 1.30f - w * 0.28f;
        if(wz < -0.90f) break;
        for(int side=-1;side<=1;side+=2){
            float wx2 = side*0.435f;  // 0.435 > 0.40 fuselage radius — sits proud
            glBegin(GL_QUADS);
                glNormal3f(side,0,0);
                glVertex3f(wx2, 0.08f, wz+0.06f); glVertex3f(wx2, 0.08f, wz-0.06f);
                glVertex3f(wx2, 0.20f, wz-0.06f); glVertex3f(wx2, 0.20f, wz+0.06f);
            glEnd();
        }
    }
    // Cockpit windows
    glColor3f(0.15f,0.25f,0.55f);
    glBegin(GL_QUADS);
        glNormal3f(0,0.4f,1);
        glVertex3f(-0.18f,0.18f,1.92f); glVertex3f( 0.18f,0.18f,1.92f);
        glVertex3f( 0.22f,0.32f,1.62f); glVertex3f(-0.22f,0.32f,1.62f);
    glEnd();
    // Airline stripe — proper side panels on both sides of fuselage
    glColor3f(0.10f,0.30f,0.75f);
    for(int side=-1;side<=1;side+=2){
        float sx = side*0.41f;
        glBegin(GL_QUADS);
            glNormal3f(side,0,0);
            glVertex3f(sx, 0.22f, 1.40f); glVertex3f(sx, 0.22f,-1.00f);
            glVertex3f(sx, 0.32f,-1.00f); glVertex3f(sx, 0.32f, 1.40f);
        glEnd();
    }
    // Swept wings (low-wing)
    glColor3f(0.88f,0.88f,0.90f);
    drawWingPanel( 0.40f, 1.20f,  0.40f,-0.40f, -0.20f,-0.90f, 0.12f);
    drawWingPanel( 1.20f, 2.20f, -0.20f,-0.90f, -0.60f,-1.20f, 0.07f);
    drawWingPanel(-0.40f,-1.20f,  0.40f,-0.40f, -0.20f,-0.90f, 0.12f);
    drawWingPanel(-1.20f,-2.20f, -0.20f,-0.90f, -0.60f,-1.20f, 0.07f);
    // 4 engine pods
    for(int e=0;e<4;e++){
        float ex = (e<2 ? 0.90f : 1.70f) * (e%2==0 ? 1 : -1);
        glPushMatrix(); glTranslatef(ex, -0.14f, 0.10f);
        glColor3f(0.40f,0.40f,0.45f);
        drawOctSegment(0.12f,0.10f, 0.55f, 0.12f,0.10f,-0.55f);
        glColor3f(0.55f,0.55f,0.60f);
        drawOctSegment(0.12f,0.10f, 0.55f, 0.15f,0.13f, 0.65f);
        drawOctCap(0.15f,0.13f, 0.65f, 0,0,1);
        glColor3f(0.28f,0.28f,0.30f);
        drawOctSegment(0.12f,0.10f,-0.55f, 0.08f,0.07f,-0.70f);
        drawOctCap(0.08f,0.07f,-0.70f, 0,0,-1);
        glPopMatrix();
    }
    // Vertical stabilizer (large) — two side faces + leading/trailing edges + top cap
    glColor3f(0.10f,0.30f,0.75f);
    for(int s=-1;s<=1;s+=2){
        glNormal3f(s,0,0);
        glBegin(GL_QUADS);
            glVertex3f(s*0.04f,0.28f,-1.50f); glVertex3f(s*0.04f,0.28f,-2.20f);
            glVertex3f(s*0.04f,0.90f,-1.90f); glVertex3f(s*0.04f,0.90f,-1.60f);
        glEnd();
        glBegin(GL_TRIANGLES);
            glVertex3f(s*0.04f,0.90f,-1.60f);
            glVertex3f(s*0.04f,0.90f,-1.90f);
            glVertex3f(s*0.04f,1.20f,-1.75f);
        glEnd();
    }
    // leading edge (forward face)
    glBegin(GL_QUADS); glNormal3f(0,0,1);
        glVertex3f(-0.04f,0.28f,-1.50f); glVertex3f( 0.04f,0.28f,-1.50f);
        glVertex3f( 0.04f,0.90f,-1.60f); glVertex3f(-0.04f,0.90f,-1.60f);
    glEnd();
    // trailing edge (aft face)
    glBegin(GL_QUADS); glNormal3f(0,0,-1);
        glVertex3f(-0.04f,0.28f,-2.20f); glVertex3f(-0.04f,0.90f,-1.90f);
        glVertex3f( 0.04f,0.90f,-1.90f); glVertex3f( 0.04f,0.28f,-2.20f);
    glEnd();
    // top cap (triangle)
    glBegin(GL_TRIANGLES); glNormal3f(0,1,0);
        glVertex3f(-0.04f,0.90f,-1.60f); glVertex3f( 0.04f,0.90f,-1.60f);
        glVertex3f( 0.00f,1.20f,-1.75f);
        glVertex3f( 0.04f,0.90f,-1.60f); glVertex3f( 0.04f,0.90f,-1.90f);
        glVertex3f( 0.00f,1.20f,-1.75f);
        glVertex3f( 0.04f,0.90f,-1.90f); glVertex3f(-0.04f,0.90f,-1.90f);
        glVertex3f( 0.00f,1.20f,-1.75f);
        glVertex3f(-0.04f,0.90f,-1.90f); glVertex3f(-0.04f,0.90f,-1.60f);
        glVertex3f( 0.00f,1.20f,-1.75f);
    glEnd();
    // Horizontal stabilizers
    glColor3f(0.88f,0.88f,0.90f);
    drawWingPanel( 0.04f, 0.70f, -1.60f,-2.00f, -1.70f,-2.05f, 0.06f);
    drawWingPanel(-0.04f,-0.70f, -1.60f,-2.00f, -1.70f,-2.05f, 0.06f);
    // Landing gear
    if(gear > 0.001f){
        float deployAngle = gear * 90.0f;
        // Nose gear
        glPushMatrix();
        glTranslatef(0.0f,-0.26f,1.60f);
        glRotatef(-(90.0f-deployAngle),1,0,0);
        glColor3f(0.35f,0.35f,0.38f);
        glBegin(GL_QUADS);
            glNormal3f(1,0,0);
            glVertex3f( 0.05f,0,-0.02f); glVertex3f( 0.05f,-0.50f*gear,-0.02f);
            glVertex3f( 0.05f,-0.50f*gear,0.02f); glVertex3f( 0.05f,0,0.02f);
            glNormal3f(-1,0,0);
            glVertex3f(-0.05f,0,-0.02f); glVertex3f(-0.05f,0,0.02f);
            glVertex3f(-0.05f,-0.50f*gear,0.02f); glVertex3f(-0.05f,-0.50f*gear,-0.02f);
        glEnd();
        glPopMatrix();
        // Main gear (4 wheels each side)
        for(int s=-1;s<=1;s+=2){
            glPushMatrix();
            glTranslatef(s*0.55f,-0.26f,-0.10f);
            glRotatef(s*(90.0f-deployAngle),0,0,1);
            glColor3f(0.35f,0.35f,0.38f);
            float sh = 0.55f*gear;
            glBegin(GL_QUADS);
                glNormal3f(1,0,0);
                glVertex3f(0.05f,0,-0.05f); glVertex3f(0.05f,-sh,-0.05f);
                glVertex3f(0.05f,-sh,0.05f); glVertex3f(0.05f,0,0.05f);
                glNormal3f(-1,0,0);
                glVertex3f(-0.05f,0,-0.05f); glVertex3f(-0.05f,0,0.05f);
                glVertex3f(-0.05f,-sh,0.05f); glVertex3f(-0.05f,-sh,-0.05f);
            glEnd();
            glTranslatef(0,-sh,0);
            glColor3f(0.12f,0.12f,0.12f);
            float wr=0.13f, wt=0.06f;
            for(int w=-1;w<=1;w+=2){
                glPushMatrix(); glTranslatef(0,0,w*0.10f);
                for(int side=-1;side<=1;side+=2){
                    glNormal3f(0,0,side);
                    glBegin(GL_TRIANGLE_FAN);
                        glVertex3f(0,0,side*wt);
                        for(int i=0;i<=10;i++){ float a=i*6.28318f/10;
                            glVertex3f(cosf(a)*wr,sinf(a)*wr,side*wt); }
                    glEnd();
                }
                glPopMatrix();
            }
            glPopMatrix();
        }
    }
}

void drawPlaneModel(float x,float y,float z,float pitch,float yaw,float roll,float gear,PlaneType type){
    glPushMatrix();
    glTranslatef(x,y,z);
    glRotatef(yaw*57.2958f,0,1,0);
    glRotatef(-pitch*57.2958f,1,0,0);
    glRotatef(roll*57.2958f,0,0,1);
    float sc = PLANE_DEFS[type].scale;
    glScalef(sc,sc,sc);
    if(type==PLANE_PROP){ drawPropPlane(gear); glPopMatrix(); return; }
    if(type==PLANE_AIRLINER){ drawAirliner(gear); glPopMatrix(); return; }

    // ---- FUSELAGE (octagonal cross-section, 6 segments) ----
    glColor3f(0.62f,0.63f,0.68f);
    // sharply pointed nose
    drawOctSegment(0.00f,0.00f, 1.40f,  0.08f,0.08f, 1.15f);
    // nose cone
    drawOctSegment(0.08f,0.08f, 1.15f,  0.20f,0.18f, 0.80f);
    // forward fuselage (slightly wider)
    drawOctSegment(0.20f,0.18f, 0.80f,  0.23f,0.22f, 0.20f);
    // mid fuselage
    drawOctSegment(0.23f,0.22f, 0.20f,  0.23f,0.21f,-0.50f);
    // aft fuselage taper
    drawOctSegment(0.23f,0.21f,-0.50f,  0.16f,0.15f,-0.90f);
    // tail boom
    drawOctSegment(0.16f,0.15f,-0.90f,  0.08f,0.09f,-1.30f);
    // caps
    drawOctCap(0.00f,0.00f, 1.40f,  0,0, 1);
    drawOctCap(0.08f,0.09f,-1.30f,  0,0,-1);

    // ---- SPINE RIDGE (dorsal) ----
    glColor3f(0.55f,0.56f,0.62f);
    glBegin(GL_QUADS);
        glNormal3f(0,1,0);
        glVertex3f(-0.04f,0.22f, 0.70f); glVertex3f( 0.04f,0.22f, 0.70f);
        glVertex3f( 0.04f,0.22f,-0.50f); glVertex3f(-0.04f,0.22f,-0.50f);
        glNormal3f(1,0,0);
        glVertex3f( 0.04f,0.22f, 0.70f); glVertex3f( 0.04f,0.12f, 0.70f);
        glVertex3f( 0.04f,0.12f,-0.50f); glVertex3f( 0.04f,0.22f,-0.50f);
        glNormal3f(-1,0,0);
        glVertex3f(-0.04f,0.22f, 0.70f); glVertex3f(-0.04f,0.22f,-0.50f);
        glVertex3f(-0.04f,0.12f,-0.50f); glVertex3f(-0.04f,0.12f, 0.70f);
    glEnd();

    // ---- COCKPIT CANOPY ----
    // Base frame (dark grey)
    glColor3f(0.25f,0.25f,0.28f);
    glBegin(GL_QUADS);
        glNormal3f(0,1,0);
        glVertex3f(-0.12f,0.22f, 0.75f); glVertex3f( 0.12f,0.22f, 0.75f);
        glVertex3f( 0.10f,0.22f, 0.20f); glVertex3f(-0.10f,0.22f, 0.20f);
    glEnd();
    // Glazing panels (blue-tinted)
    glColor3f(0.18f,0.28f,0.50f);
    // front windscreen
    glNormal3f(0,0.5f,1);
    glBegin(GL_QUADS);
        glVertex3f(-0.10f,0.22f, 0.75f); glVertex3f( 0.10f,0.22f, 0.75f);
        glVertex3f( 0.11f,0.38f, 0.60f); glVertex3f(-0.11f,0.38f, 0.60f);
    glEnd();
    // main bubble
    glNormal3f(0,1,0);
    glBegin(GL_QUADS);
        glVertex3f(-0.11f,0.38f, 0.60f); glVertex3f( 0.11f,0.38f, 0.60f);
        glVertex3f( 0.10f,0.36f, 0.25f); glVertex3f(-0.10f,0.36f, 0.25f);
    glEnd();
    // sides
    glNormal3f(-1,0.3f,0);
    glBegin(GL_QUADS);
        glVertex3f(-0.10f,0.22f, 0.75f); glVertex3f(-0.11f,0.38f, 0.60f);
        glVertex3f(-0.10f,0.36f, 0.25f); glVertex3f(-0.10f,0.22f, 0.20f);
    glEnd();
    glNormal3f(1,0.3f,0);
    glBegin(GL_QUADS);
        glVertex3f( 0.10f,0.22f, 0.75f); glVertex3f( 0.10f,0.22f, 0.20f);
        glVertex3f( 0.10f,0.36f, 0.25f); glVertex3f( 0.11f,0.38f, 0.60f);
    glEnd();
    // rear slope
    glNormal3f(0,0.5f,-1);
    glBegin(GL_QUADS);
        glVertex3f(-0.10f,0.36f, 0.25f); glVertex3f( 0.10f,0.36f, 0.25f);
        glVertex3f( 0.10f,0.22f, 0.20f); glVertex3f(-0.10f,0.22f, 0.20f);
    glEnd();
    // canopy frame bars
    glColor3f(0.20f,0.20f,0.22f);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
        glVertex3f(0,0.38f,0.60f); glVertex3f(0,0.36f,0.25f); // centre bar
        glVertex3f(-0.11f,0.38f,0.60f); glVertex3f(0.11f,0.38f,0.60f); // front hoop
        glVertex3f(-0.10f,0.36f,0.25f); glVertex3f(0.10f,0.36f,0.25f); // rear hoop
    glEnd();
    glLineWidth(1.0f);

    // ---- MAIN WINGS ----
    glColor3f(0.58f,0.59f,0.65f);
    // right wing inner panel (root to mid)
    drawWingPanel( 0.23f, 0.85f,  0.12f,-0.32f,  -0.08f,-0.48f, 0.07f);
    // right wing outer panel (mid to tip) — more taper
    drawWingPanel( 0.85f, 1.55f, -0.08f,-0.48f,  -0.28f,-0.58f, 0.05f);
    // left mirror
    drawWingPanel(-0.23f,-0.85f,  0.12f,-0.32f,  -0.08f,-0.48f, 0.07f);
    drawWingPanel(-0.85f,-1.55f, -0.08f,-0.48f,  -0.28f,-0.58f, 0.05f);

    // Wing root fillet (blends wing into fuselage)
    glColor3f(0.60f,0.60f,0.66f);
    glBegin(GL_QUADS);
        glNormal3f(0,1,0);
        glVertex3f(-0.23f,0.035f, 0.15f); glVertex3f( 0.23f,0.035f, 0.15f);
        glVertex3f( 0.23f,0.035f,-0.35f); glVertex3f(-0.23f,0.035f,-0.35f);
    glEnd();

    // Ailerons (outer 35% of wing, slightly darker)
    drawControlSurface( 0.85f, 1.55f, -0.28f, -0.28f, 0.10f, 0.04f, 0.50f,0.51f,0.58f);
    drawControlSurface(-0.85f,-1.55f, -0.28f, -0.28f, 0.10f, 0.04f, 0.50f,0.51f,0.58f);
    // Flaps (inner trailing edge)
    drawControlSurface( 0.23f, 0.85f, -0.32f, -0.48f, 0.12f, 0.06f, 0.52f,0.53f,0.60f);
    drawControlSurface(-0.23f,-0.85f, -0.32f, -0.48f, 0.12f, 0.06f, 0.52f,0.53f,0.60f);

    // ---- ENGINE PODS (with intake ring and nozzle) ----
    for(int side=-1;side<=1;side+=2){
        float ex = side * 0.72f;
        glPushMatrix(); glTranslatef(ex, -0.10f, -0.05f);

        // main nacelle body
        glColor3f(0.42f,0.42f,0.48f);
        drawOctSegment(0.09f,0.07f, 0.40f,  0.09f,0.07f,-0.40f);
        // intake lip (slightly wider, lighter)
        glColor3f(0.55f,0.55f,0.60f);
        drawOctSegment(0.09f,0.07f, 0.40f,  0.11f,0.09f, 0.46f);
        drawOctCap(0.11f,0.09f, 0.46f, 0,0,1);
        // nozzle (darker, narrows)
        glColor3f(0.30f,0.30f,0.33f);
        drawOctSegment(0.09f,0.07f,-0.40f,  0.06f,0.05f,-0.52f);
        drawOctCap(0.06f,0.05f,-0.52f, 0,0,-1);
        // pylon connecting pod to wing
        glColor3f(0.48f,0.48f,0.54f);
        glBegin(GL_QUADS);
            glNormal3f(0,1,0);
            glVertex3f(-0.03f, 0.07f, 0.20f); glVertex3f( 0.03f, 0.07f, 0.20f);
            glVertex3f( 0.03f, 0.07f,-0.25f); glVertex3f(-0.03f, 0.07f,-0.25f);
            glNormal3f(1,0,0);
            glVertex3f( 0.03f, 0.07f, 0.20f); glVertex3f( 0.03f, 0.00f, 0.20f);
            glVertex3f( 0.03f, 0.00f,-0.25f); glVertex3f( 0.03f, 0.07f,-0.25f);
            glNormal3f(-1,0,0);
            glVertex3f(-0.03f, 0.07f, 0.20f); glVertex3f(-0.03f, 0.07f,-0.25f);
            glVertex3f(-0.03f, 0.00f,-0.25f); glVertex3f(-0.03f, 0.00f, 0.20f);
        glEnd();
        glPopMatrix();
    }

    // ---- WINGTIP FUEL TANKS ----
    for(int side=-1;side<=1;side+=2){
        float tx = side * 1.55f;
        glPushMatrix(); glTranslatef(tx, 0.0f, -0.43f);
        glColor3f(0.58f,0.58f,0.64f);
        drawOctSegment(0.00f,0.00f, 0.22f,  0.05f,0.04f, 0.12f);
        drawOctSegment(0.05f,0.04f, 0.12f,  0.05f,0.04f,-0.10f);
        drawOctSegment(0.05f,0.04f,-0.10f,  0.00f,0.00f,-0.20f);
        drawOctCap(0.00f,0.00f, 0.22f,  0,0,1);
        drawOctCap(0.00f,0.00f,-0.20f,  0,0,-1);
        glPopMatrix();
    }

    // ---- VERTICAL STABILIZER ----
    glColor3f(0.60f,0.61f,0.67f);
    // main fin panels (two sides)
    for(int side=-1;side<=1;side+=2){
        float sx2 = side * 0.025f;
        glNormal3f(side,0,0);
        glBegin(GL_QUADS);
            // base rectangle
            glVertex3f(sx2, 0.15f,-0.85f); glVertex3f(sx2, 0.15f,-1.28f);
            glVertex3f(sx2, 0.60f,-1.10f); glVertex3f(sx2, 0.60f,-0.95f);
        glEnd();
        glBegin(GL_TRIANGLES);
            // upper triangle
            glVertex3f(sx2, 0.60f,-0.95f);
            glVertex3f(sx2, 0.60f,-1.10f);
            glVertex3f(sx2, 0.78f,-1.02f);
        glEnd();
    }
    // fin edges
    glColor3f(0.52f,0.53f,0.58f);
    glBegin(GL_QUADS);
        glNormal3f(0,0,1);
        glVertex3f(-0.025f,0.15f,-0.85f); glVertex3f( 0.025f,0.15f,-0.85f);
        glVertex3f( 0.025f,0.60f,-0.95f); glVertex3f(-0.025f,0.60f,-0.95f);
        glNormal3f(0,1,0);
        glVertex3f(-0.025f,0.60f,-0.95f); glVertex3f( 0.025f,0.60f,-0.95f);
        glVertex3f( 0.025f,0.78f,-1.02f); glVertex3f(-0.025f,0.78f,-1.02f);
        glNormal3f(0,0,-1);
        glVertex3f(-0.025f,0.15f,-1.28f); glVertex3f(-0.025f,0.60f,-1.10f);
        glVertex3f( 0.025f,0.60f,-1.10f); glVertex3f( 0.025f,0.15f,-1.28f);
    glEnd();
    // rudder (slightly darker)
    glColor3f(0.53f,0.54f,0.60f);
    for(int side=-1;side<=1;side+=2){
        glNormal3f(side,0,0);
        glBegin(GL_QUADS);
            glVertex3f(side*0.025f, 0.20f,-1.05f);
            glVertex3f(side*0.025f, 0.20f,-1.28f);
            glVertex3f(side*0.025f, 0.58f,-1.10f);
            glVertex3f(side*0.025f, 0.58f,-1.00f);
        glEnd();
    }

    // ---- HORIZONTAL STABILIZERS ----
    glColor3f(0.58f,0.59f,0.65f);
    drawWingPanel( 0.09f, 0.58f,  -0.88f,-1.18f,  -0.96f,-1.20f, 0.04f);
    drawWingPanel(-0.09f,-0.58f,  -0.88f,-1.18f,  -0.96f,-1.20f, 0.04f);
    // elevators
    drawControlSurface( 0.09f, 0.58f, -1.18f, -1.20f, 0.10f, 0.035f, 0.52f,0.53f,0.59f);
    drawControlSurface(-0.09f,-0.58f, -1.18f, -1.20f, 0.10f, 0.035f, 0.52f,0.53f,0.59f);

    // ---- LANDING GEAR ----
    // gear=0 retracted, gear=1 deployed. Leg rotates from folded (90deg up) to down.
    if(gear > 0.001f){
        float deployAngle = gear * 90.0f;  // 0=folded into bay, 90=vertical/down

        // -- Nose gear (under nose, folds forward) --
        glPushMatrix();
        glTranslatef(0.0f, -0.18f, 0.85f);       // bay position under nose
        glRotatef(-(90.0f - deployAngle), 1,0,0); // fold axis: pitch forward
        glColor3f(0.35f,0.35f,0.38f);
        // strut
        glBegin(GL_QUADS);
            glNormal3f(1,0,0);
            glVertex3f( 0.03f, 0.0f, 0.0f); glVertex3f( 0.03f,-0.32f*gear, 0.0f);
            glVertex3f( 0.03f,-0.32f*gear, 0.04f); glVertex3f( 0.03f, 0.0f, 0.04f);
            glNormal3f(-1,0,0);
            glVertex3f(-0.03f, 0.0f, 0.0f); glVertex3f(-0.03f, 0.0f, 0.04f);
            glVertex3f(-0.03f,-0.32f*gear, 0.04f); glVertex3f(-0.03f,-0.32f*gear, 0.0f);
            glNormal3f(0,0,1);
            glVertex3f(-0.03f, 0.0f, 0.04f); glVertex3f( 0.03f, 0.0f, 0.04f);
            glVertex3f( 0.03f,-0.32f*gear, 0.04f); glVertex3f(-0.03f,-0.32f*gear, 0.04f);
            glNormal3f(0,0,-1);
            glVertex3f(-0.03f, 0.0f, 0.0f); glVertex3f(-0.03f,-0.32f*gear, 0.0f);
            glVertex3f( 0.03f,-0.32f*gear, 0.0f); glVertex3f( 0.03f, 0.0f, 0.0f);
        glEnd();
        // wheel axle
        glTranslatef(0.0f, -0.32f*gear, 0.02f);
        glColor3f(0.15f,0.15f,0.15f);
        // wheel (octagon viewed from side)
        float wr=0.08f, wt=0.05f;
        for(int side=-1;side<=1;side+=2){
            glNormal3f(0,0,side);
            glBegin(GL_TRIANGLE_FAN);
                glVertex3f(0,0, side*wt);
                for(int i=0;i<=8;i++){
                    float a=i*3.14159f*2/8;
                    glVertex3f(cosf(a)*wr, sinf(a)*wr, side*wt);
                }
            glEnd();
        }
        glBegin(GL_TRIANGLE_STRIP);
        for(int i=0;i<=8;i++){
            float a=i*3.14159f*2/8;
            float cx2=cosf(a)*wr, cy=sinf(a)*wr;
            float nx2=cosf(a), ny=sinf(a);
            glNormal3f(nx2,ny,0);
            glVertex3f(cx2,cy, wt);
            glVertex3f(cx2,cy,-wt);
        }
        glEnd();
        glPopMatrix();

        // -- Main gear (two legs, fold outward/inward under wings) --
        for(int side=-1;side<=1;side+=2){
            glPushMatrix();
            glTranslatef(side*0.55f, -0.18f, -0.10f); // bay under wing root
            glRotatef(side*(90.0f - deployAngle), 0,0,1); // fold axis: roll outward
            glColor3f(0.35f,0.35f,0.38f);
            float strutH = 0.38f * gear;
            // main strut
            glBegin(GL_QUADS);
                glNormal3f(1,0,0);
                glVertex3f( 0.04f, 0.0f, -0.04f); glVertex3f( 0.04f,-strutH,-0.04f);
                glVertex3f( 0.04f,-strutH, 0.04f); glVertex3f( 0.04f, 0.0f,  0.04f);
                glNormal3f(-1,0,0);
                glVertex3f(-0.04f, 0.0f, -0.04f); glVertex3f(-0.04f, 0.0f,  0.04f);
                glVertex3f(-0.04f,-strutH, 0.04f); glVertex3f(-0.04f,-strutH,-0.04f);
                glNormal3f(0,0, 1);
                glVertex3f(-0.04f, 0.0f, 0.04f); glVertex3f( 0.04f, 0.0f, 0.04f);
                glVertex3f( 0.04f,-strutH,0.04f); glVertex3f(-0.04f,-strutH,0.04f);
                glNormal3f(0,0,-1);
                glVertex3f(-0.04f, 0.0f,-0.04f); glVertex3f(-0.04f,-strutH,-0.04f);
                glVertex3f( 0.04f,-strutH,-0.04f); glVertex3f( 0.04f, 0.0f,-0.04f);
            glEnd();
            // torque link (small diagonal brace)
            glColor3f(0.40f,0.40f,0.43f);
            glBegin(GL_QUADS);
                glNormal3f(side,0,0);
                glVertex3f(side*0.04f, -strutH*0.3f, -0.02f);
                glVertex3f(side*0.04f, -strutH*0.7f,  0.02f);
                glVertex3f(side*0.04f, -strutH*0.7f, -0.02f);
                glVertex3f(side*0.04f, -strutH*0.3f,  0.02f);
            glEnd();
            // two wheels side by side
            glTranslatef(0.0f, -strutH, 0.0f);
            for(int w=-1;w<=1;w+=2){
                glPushMatrix();
                glTranslatef(0, 0, w*0.07f);
                glColor3f(0.12f,0.12f,0.12f);
                float wr2=0.10f, wt2=0.045f;
                for(int s2=-1;s2<=1;s2+=2){
                    glNormal3f(0,0,s2);
                    glBegin(GL_TRIANGLE_FAN);
                        glVertex3f(0,0,s2*wt2);
                        for(int i=0;i<=10;i++){
                            float a=i*3.14159f*2/10;
                            glVertex3f(cosf(a)*wr2,sinf(a)*wr2,s2*wt2);
                        }
                    glEnd();
                }
                glBegin(GL_TRIANGLE_STRIP);
                for(int i=0;i<=10;i++){
                    float a=i*3.14159f*2/10;
                    glNormal3f(cosf(a),sinf(a),0);
                    glVertex3f(cosf(a)*wr2,sinf(a)*wr2, wt2);
                    glVertex3f(cosf(a)*wr2,sinf(a)*wr2,-wt2);
                }
                glEnd();
                // hub cap
                glColor3f(0.50f,0.50f,0.55f);
                glNormal3f(0,0,1);
                glBegin(GL_TRIANGLE_FAN);
                    glVertex3f(0,0,wt2+0.005f);
                    for(int i=0;i<=6;i++){
                        float a=i*3.14159f*2/6;
                        glVertex3f(cosf(a)*0.04f,sinf(a)*0.04f,wt2+0.005f);
                    }
                glEnd();
                glPopMatrix();
            }
            glPopMatrix();
        }

        // -- Gear bay doors (open when gear deployed, close when retracted) --
        // doors hinge outward as gear goes down
        float doorAngle = gear * 75.0f;
        glColor3f(0.58f,0.59f,0.65f);
        // nose gear doors (two halves hinge left/right)
        for(int side=-1;side<=1;side+=2){
            glPushMatrix();
            glTranslatef(side*0.04f, -0.18f, 0.85f);
            glRotatef(side*doorAngle, 0,0,1);
            glNormal3f(0,-1,0);
            glBegin(GL_QUADS);
                glVertex3f(0,0,  0.18f); glVertex3f(side*0.10f,0,  0.18f);
                glVertex3f(side*0.10f,0,-0.05f); glVertex3f(0,0,-0.05f);
            glEnd();
            glPopMatrix();
        }
        // main gear doors
        for(int side=-1;side<=1;side+=2){
            glPushMatrix();
            glTranslatef(side*0.55f, -0.18f, -0.10f);
            glRotatef(-side*doorAngle, 1,0,0);
            glNormal3f(0,-1,0);
            glBegin(GL_QUADS);
                glVertex3f(-side*0.15f,0, 0.20f); glVertex3f(side*0.05f,0, 0.20f);
                glVertex3f(side*0.05f,0,-0.22f);  glVertex3f(-side*0.15f,0,-0.22f);
            glEnd();
            glPopMatrix();
        }
    }

    // ---- NOSE PROBE / PITOT ----
    glColor3f(0.70f,0.70f,0.72f);
    glBegin(GL_QUADS);
        glNormal3f(0,1,0);
        glVertex3f(-0.008f, 0.005f, 1.40f); glVertex3f( 0.008f, 0.005f, 1.40f);
        glVertex3f( 0.008f, 0.005f, 1.65f); glVertex3f(-0.008f, 0.005f, 1.65f);
        glNormal3f(0,-1,0);
        glVertex3f(-0.008f,-0.005f, 1.40f); glVertex3f(-0.008f,-0.005f, 1.65f);
        glVertex3f( 0.008f,-0.005f, 1.65f); glVertex3f( 0.008f,-0.005f, 1.40f);
        glNormal3f(1,0,0);
        glVertex3f( 0.008f,-0.005f, 1.40f); glVertex3f( 0.008f,-0.005f, 1.65f);
        glVertex3f( 0.008f, 0.005f, 1.65f); glVertex3f( 0.008f, 0.005f, 1.40f);
        glNormal3f(-1,0,0);
        glVertex3f(-0.008f,-0.005f, 1.40f); glVertex3f(-0.008f, 0.005f, 1.40f);
        glVertex3f(-0.008f, 0.005f, 1.65f); glVertex3f(-0.008f,-0.005f, 1.65f);
    glEnd();

    glPopMatrix();
}

// -------- Terrain --------
#define TN (TERRAIN_SIZE+1)
static float terrainH[TN][TN];
static float terrainNX[TN][TN];
static float terrainNY[TN][TN];
static float terrainNZ[TN][TN];

// Simple value noise: hash grid coords to a pseudo-random float in [-1,1]
static float vnoise(int x, int z){
    unsigned int h = (unsigned int)(x*1619 + z*31337 + x*z*6971);
    h ^= h>>16; h *= 0x45d9f3b; h ^= h>>16;
    return (float)(h & 0xFFFF) / 32767.5f - 1.0f;
}

// Bilinear smooth noise at fractional grid coords
static float smoothNoise(float x, float z){
    int ix=(int)floorf(x), iz=(int)floorf(z);
    float fx=x-ix, fz=z-iz;
    // smoothstep
    fx = fx*fx*(3-2*fx); fz = fz*fz*(3-2*fz);
    float v00=vnoise(ix,iz),   v10=vnoise(ix+1,iz);
    float v01=vnoise(ix,iz+1), v11=vnoise(ix+1,iz+1);
    return v00*(1-fx)*(1-fz)+v10*fx*(1-fz)+v01*(1-fx)*fz+v11*fx*fz;
}

// fBm: sum several octaves of smooth noise
static float fbm(float x, float z, int octaves){
    float val=0, amp=1.0f, freq=1.0f, maxAmp=0;
    for(int i=0;i<octaves;i++){
        val += smoothNoise(x*freq, z*freq)*amp;
        maxAmp += amp;
        amp  *= 0.5f;
        freq *= 2.0f;
    }
    return val/maxAmp; // normalise to [-1,1]
}

// Phase 1: Diamond-Square base only. Leaves terrainH with low-frequency
// height values that airports can safely sample and flatten before detail is added.
static void generateTerrainBase(){
    terrainH[0][0]       = TERRAIN_HEIGHT*0.25f;
    terrainH[0][TN-1]    = TERRAIN_HEIGHT*0.35f;
    terrainH[TN-1][0]    = TERRAIN_HEIGHT*0.30f;
    terrainH[TN-1][TN-1] = TERRAIN_HEIGHT*0.40f;

    // Lower roughness = gentler hills, fewer extreme peaks
    float roughness = 0.50f, scale = TERRAIN_HEIGHT;
    for(int step=TERRAIN_SIZE; step>1; step/=2){
        int half=step/2; scale*=roughness;
        for(int y=0;y<TERRAIN_SIZE;y+=step)
            for(int x=0;x<TERRAIN_SIZE;x+=step){
                float avg=(terrainH[y][x]+terrainH[y][x+step]+
                           terrainH[y+step][x]+terrainH[y+step][x+step])*0.25f;
                terrainH[y+half][x+half]=avg+((float)rand()/RAND_MAX*2.f-1.f)*scale;
            }
        for(int y=0;y<=TERRAIN_SIZE;y+=half)
            for(int x=(y+half)%step;x<=TERRAIN_SIZE;x+=step){
                float sum=0; int cnt=0;
                if(y-half>=0)          {sum+=terrainH[y-half][x];cnt++;}
                if(y+half<=TERRAIN_SIZE){sum+=terrainH[y+half][x];cnt++;}
                if(x-half>=0)          {sum+=terrainH[y][x-half];cnt++;}
                if(x+half<=TERRAIN_SIZE){sum+=terrainH[y][x+half];cnt++;}
                terrainH[y][x]=sum/cnt+((float)rand()/RAND_MAX*2.f-1.f)*scale;
            }
    }
    // Clamp base to valid range
    for(int z=0;z<TN;z++)
        for(int x=0;x<TN;x++)
            terrainH[z][x]=clampf(terrainH[z][x],0.0f,TERRAIN_HEIGHT);
}

float terrainHeightAt(float wx, float wz); // forward declaration
void  generateAllTerrain(unsigned int seed); // forward declaration
void  generateAirportName(char *out, int idx); // forward declaration

// Sample average base height over a world-space circle of radius r
static float sampleAvgHeight(float wx, float wz, float radius){
    int steps = 8;
    float sum = terrainHeightAt(wx, wz);
    int cnt = 1;
    for(int i=0;i<steps;i++){
        float a = i * 3.14159f*2/steps;
        sum += terrainHeightAt(wx+cosf(a)*radius, wz+sinf(a)*radius);
        cnt++;
    }
    return sum / cnt;
}

// Phase 2: Add fBm detail, apply airport carves, compute normals.
// Must be called after all flattenRect calls so fBm doesn't grow peaks over flat zones.
static void generateTerrainDetail(){
    for(int z=0;z<TN;z++)
        for(int x=0;x<TN;x++){
            float fx=(float)x/TERRAIN_SIZE, fz=(float)z/TERRAIN_SIZE;
            // Gentle rolling hills (softer ridge, reduced contribution vs before)
            float ridge = fbm(fx*3.0f, fz*3.0f, 4);
            ridge = 1.0f - fabsf(ridge);  // ridge-ify
            ridge = ridge * ridge * 0.5f; // weaker than before (was *1.0, now *0.5)
            // Fine surface bumps
            float detail = fbm(fx*14.0f, fz*14.0f, 3) * 0.4f;
            terrainH[z][x] += ridge * TERRAIN_HEIGHT * 0.25f + detail * 1.5f;
            terrainH[z][x]  = clampf(terrainH[z][x], 0.0f, TERRAIN_HEIGHT);
        }

    // Recompute all normals after detail pass
    for(int z=0;z<TN;z++)
        for(int x=0;x<TN;x++){
            float h[3][3];
            for(int dz=-1;dz<=1;dz++)
                for(int dx=-1;dx<=1;dx++){
                    int sx=(int)clampf(x+dx,0,TERRAIN_SIZE);
                    int sz=(int)clampf(z+dz,0,TERRAIN_SIZE);
                    h[dz+1][dx+1]=terrainH[sz][sx];
                }
            float gx=(h[0][2]-h[0][0]+2*(h[1][2]-h[1][0])+h[2][2]-h[2][0])/(8.0f*TERRAIN_SCALE);
            float gz=(h[2][0]-h[0][0]+2*(h[2][1]-h[0][1])+h[2][2]-h[0][2])/(8.0f*TERRAIN_SCALE);
            float nx=-gx, ny=1.0f, nz2=-gz;
            float len=sqrtf(nx*nx+ny*ny+nz2*nz2);
            terrainNX[z][x]=nx/len; terrainNY[z][x]=ny/len; terrainNZ[z][x]=nz2/len;
        }
}

float terrainHeightAt(float wx, float wz){
    float ox=wx/TERRAIN_SCALE+TERRAIN_SIZE*0.5f;
    float oz=wz/TERRAIN_SCALE+TERRAIN_SIZE*0.5f;
    int ix=(int)ox, iz=(int)oz;
    ix=(int)clampf(ix,0,TERRAIN_SIZE-2);
    iz=(int)clampf(iz,0,TERRAIN_SIZE-2);
    float fx=ox-ix, fz=oz-iz;
    return terrainH[iz][ix]*(1-fx)*(1-fz)+terrainH[iz][ix+1]*fx*(1-fz)
          +terrainH[iz+1][ix]*(1-fx)*fz  +terrainH[iz+1][ix+1]*fx*fz;
}

// Height+slope colour: steep faces go rock-grey regardless of altitude
static void terrainColor(float h, float nx, float ny, float nz){
    (void)nx;(void)nz;
    float slope = 1.0f - ny;           // 0=flat, ~1=vertical cliff
    float t = h / TERRAIN_HEIGHT;

    float r,g,b;
    if(t < 0.03f){      r=0.15f; g=0.28f; b=0.62f; } // water
    else if(t < 0.07f){ r=0.76f; g=0.70f; b=0.50f; } // sand
    else if(t < 0.35f){ r=0.22f; g=0.50f; b=0.16f; } // grass
    else if(t < 0.60f){ r=0.28f; g=0.43f; b=0.18f; } // highland grass
    else if(t < 0.78f){ r=0.50f; g=0.46f; b=0.38f; } // rock
    else {              r=0.88f; g=0.90f; b=0.93f; } // snow

    // blend toward rock on steep slopes (slope > 0.3)
    if(slope > 0.3f){
        float blend = clampf((slope-0.3f)/0.35f, 0.0f, 1.0f);
        r=r*(1-blend)+0.48f*blend;
        g=g*(1-blend)+0.44f*blend;
        b=b*(1-blend)+0.37f*blend;
    }
    glColor3f(r,g,b);
}

// -------- Airports --------

// Flatten a rotated rectangular region of the heightmap to a given elevation.
static void flattenRect(float cx, float cz, float heading,
                        float halfLen, float halfWid, float elev, float blendExtra){
    float cosH=cosf(heading), sinH=sinf(heading);
    // iterate over bounding box in terrain grid space
    float total = halfLen + halfWid + blendExtra;
    int x0=(int)clampf((cx-total)/TERRAIN_SCALE+TERRAIN_SIZE*0.5f-2, 0, TERRAIN_SIZE);
    int x1=(int)clampf((cx+total)/TERRAIN_SCALE+TERRAIN_SIZE*0.5f+2, 0, TERRAIN_SIZE);
    int z0=(int)clampf((cz-total)/TERRAIN_SCALE+TERRAIN_SIZE*0.5f-2, 0, TERRAIN_SIZE);
    int z1=(int)clampf((cz+total)/TERRAIN_SCALE+TERRAIN_SIZE*0.5f+2, 0, TERRAIN_SIZE);
    for(int gz=z0; gz<=z1; gz++)
        for(int gx=x0; gx<=x1; gx++){
            float wx=((float)gx-TERRAIN_SIZE*0.5f)*TERRAIN_SCALE;
            float wz=((float)gz-TERRAIN_SIZE*0.5f)*TERRAIN_SCALE;
            float dx=wx-cx, dz2=wz-cz;
            // rotate into runway local space
            float local_fwd =  cosH*dx + sinH*dz2;
            float local_side= -sinH*dx + cosH*dz2;
            float distFwd = fabsf(local_fwd) - halfLen;
            float distSide= fabsf(local_side)- halfWid;
            float dist = fmaxf(distFwd, distSide); // signed dist from rect edge
            if(dist <= 0.0f){
                terrainH[gz][gx] = elev;
            } else if(dist < blendExtra){
                float t = dist / blendExtra;
                terrainH[gz][gx] = elev*(1-t) + terrainH[gz][gx]*t;
            }
        }
}

// Recompute normals for cells affected by airport flattening.
static void recomputeNormalsRect(float cx, float cz, float halfLen, float halfWid, float extra){
    float total = halfLen + halfWid + extra + TERRAIN_SCALE*2;
    int x0=(int)clampf((cx-total)/TERRAIN_SCALE+TERRAIN_SIZE*0.5f-1,1,TERRAIN_SIZE-1);
    int x1=(int)clampf((cx+total)/TERRAIN_SCALE+TERRAIN_SIZE*0.5f+1,1,TERRAIN_SIZE-1);
    int z0=(int)clampf((cz-total)/TERRAIN_SCALE+TERRAIN_SIZE*0.5f-1,1,TERRAIN_SIZE-1);
    int z1=(int)clampf((cz+total)/TERRAIN_SCALE+TERRAIN_SIZE*0.5f+1,1,TERRAIN_SIZE-1);
    for(int z=z0;z<=z1;z++)
        for(int x=x0;x<=x1;x++){
            float h[3][3];
            for(int dz=-1;dz<=1;dz++)
                for(int dx=-1;dx<=1;dx++)
                    h[dz+1][dx+1]=terrainH[(int)clampf(z+dz,0,TERRAIN_SIZE)]
                                          [(int)clampf(x+dx,0,TERRAIN_SIZE)];
            float gx=(h[0][2]-h[0][0]+2*(h[1][2]-h[1][0])+h[2][2]-h[2][0])/(8.0f*TERRAIN_SCALE);
            float gz=(h[2][0]-h[0][0]+2*(h[2][1]-h[0][1])+h[2][2]-h[0][2])/(8.0f*TERRAIN_SCALE);
            float nx=-gx, ny=1.0f, nz2=-gz;
            float len=sqrtf(nx*nx+ny*ny+nz2*nz2);
            terrainNX[z][x]=nx/len; terrainNY[z][x]=ny/len; terrainNZ[z][x]=nz2/len;
        }
}

// Convert airport-local (fwd, side) to world X,Z
static void apLocalToWorld(const Airport *ap, float fwd, float side,
                           float *wx, float *wz){
    float c = cosf(ap->heading), s = sinf(ap->heading);
    *wx = ap->wx + c*fwd + s*side;
    *wz = ap->wz - s*fwd + c*side;
}

static void spawnParkedPlanesForAirport(const Airport *ap){
    int ngates = 2 + ap->size * 2;
    float twW    = 3.5f;
    float twCenZ = ap->runwayW + 2.0f + twW;
    float apronZ0 = twCenZ + twW;
    float apronZ1 = apronZ0 + 20.0f;
    float apronHalfX = (ngates - 1) * 8.0f * 0.5f + 5.0f;
    float gzFront = apronZ0 + 2.0f;   // plane nose stops here
    float planeZ  = gzFront + 5.0f;   // plane centre (nose toward terminal = +Z)

    for(int g = 0; g < ngates && numParked < MAX_PARKED; g++){
        if(rand() % 3 == 0) continue; // ~33% chance gate is empty
        float lx = -apronHalfX + 5.0f + g * 8.0f;
        float wx, wz;
        apLocalToWorld(ap, lx, planeZ, &wx, &wz);
        ParkedPlane *pp = &parkedPlanes[numParked++];
        pp->wx      = wx;
        pp->wy      = ap->groundY + 1.0f;
        pp->wz      = wz;
        // Plane faces away from terminal (toward taxiway), which is -Z in local = heading+PI
        pp->heading = ap->heading + 3.14159f;
        pp->active  = true;
        // Assign type: large airports get a mix including airliners; small = prop or fighter
        int r = rand() % (ap->size == 2 ? 3 : (ap->size == 1 ? 2 : 2));
        pp->type = (PlaneType)r;
    }
}

// generateAirports() must be called AFTER generateTerrainBase() and BEFORE
// generateTerrainDetail(). It samples base heights, flattens footprints, then
// detail is added on top — so ridges never grow back inside cleared airport zones.
void generateAirports(){
    numAirports = 0;
    numParked   = 0;

    // Home base at terrain centre — sample base height there
    float homeH = sampleAvgHeight(0, 0, 20.0f);
    airports[0].wx       = 0.0f; airports[0].wz = 0.0f;
    airports[0].heading  = 0.0f;
    airports[0].runwayLen= 55.0f;
    airports[0].runwayW  = 5.0f;
    airports[0].groundY  = homeH;
    airports[0].size     = 2;
    airports[0].numGates = 6;
    numAirports = 1;
    generateAirportName(airportNames[0], 0);
    flattenRect(0,0,0, 55.0f, 5.0f,   homeH, 18.0f);
    flattenRect(0,0,0, 55.0f, 50.0f,  homeH, 12.0f);

    // Random airports — all placed on base heightmap before detail pass
    int attempts = 0;
    while(numAirports < MAX_AIRPORTS && attempts < 2000){
        attempts++;
        float margin = TERRAIN_SIZE*TERRAIN_SCALE*0.5f - 50.0f;
        float wx = ((float)rand()/RAND_MAX*2.0f-1.0f)*margin;
        float wz = ((float)rand()/RAND_MAX*2.0f-1.0f)*margin;

        // Keep away from other airports
        bool tooClose = false;
        for(int i=0;i<numAirports;i++){
            float dx=wx-airports[i].wx, dz=wz-airports[i].wz;
            if(sqrtf(dx*dx+dz*dz)<130.0f){ tooClose=true; break; }
        }
        if(tooClose) continue;

        // Sample average base height over the footprint — skip high terrain
        float avgH = sampleAvgHeight(wx, wz, 30.0f);
        if(avgH > TERRAIN_HEIGHT * 0.45f) continue;

        float heading = ((float)rand()/RAND_MAX) * 3.14159f;
        int   size    = rand()%3;
        float rl = 30.0f + size*15.0f;
        float rw = 3.5f  + size*1.0f;

        Airport *ap = &airports[numAirports];
        generateAirportName(airportNames[numAirports], numAirports);
        numAirports++;
        ap->wx = wx; ap->wz = wz;
        ap->heading   = heading;
        ap->runwayLen = rl;  ap->runwayW = rw;
        ap->groundY   = avgH;
        ap->size      = size;
        ap->numGates  = 2 + size*2;

        // Flatten runway strip and full side zone to the sampled base height
        flattenRect(wx, wz, heading, rl,      rw,       avgH, 20.0f);
        flattenRect(wx, wz, heading, rl*0.9f, rw+50.0f, avgH, 12.0f);
    }
    // normals are recomputed globally by generateTerrainDetail() after this returns
}

// Full terrain + airport pipeline seeded deterministically.
// Safe to call from the network thread — holds terrainMutex for the duration.
void generateAllTerrain(unsigned int seed){
    pthread_mutex_lock(&terrainMutex);
    srand(seed);
    generateTerrainBase();
    generateAirports();
    generateTerrainDetail();
    for(int i=0;i<numAirports;i++){
        Airport *ap=&airports[i];
        flattenRect(ap->wx,ap->wz,ap->heading, ap->runwayLen,      ap->runwayW,       ap->groundY, 18.0f);
        flattenRect(ap->wx,ap->wz,ap->heading, ap->runwayLen*0.9f, ap->runwayW+50.0f, ap->groundY, 12.0f);
    }
    for(int i=0;i<numAirports;i++) spawnParkedPlanesForAirport(&airports[i]);
    pthread_mutex_unlock(&terrainMutex);
}

// Draw a simple box building in local space (centre base at x,0,z).
// hw=half-width(X), hh=full height(Y), hd=half-depth(Z)
static void drawBox(float x,float y,float z, float hw,float hh,float hd,
                    float r,float g,float b){
    glColor3f(r,g,b);
    glBegin(GL_QUADS);
        glNormal3f(0,1,0);
        glVertex3f(x-hw,y+hh,z-hd); glVertex3f(x+hw,y+hh,z-hd);
        glVertex3f(x+hw,y+hh,z+hd); glVertex3f(x-hw,y+hh,z+hd);
        glNormal3f(0,-1,0);
        glVertex3f(x-hw,y,z-hd); glVertex3f(x-hw,y,z+hd);
        glVertex3f(x+hw,y,z+hd); glVertex3f(x+hw,y,z-hd);
        glNormal3f(0,0,1);
        glVertex3f(x-hw,y,z+hd); glVertex3f(x+hw,y,z+hd);
        glVertex3f(x+hw,y+hh,z+hd); glVertex3f(x-hw,y+hh,z+hd);
        glNormal3f(0,0,-1);
        glVertex3f(x-hw,y,z-hd); glVertex3f(x-hw,y+hh,z-hd);
        glVertex3f(x+hw,y+hh,z-hd); glVertex3f(x+hw,y,z-hd);
        glNormal3f(1,0,0);
        glVertex3f(x+hw,y,z-hd); glVertex3f(x+hw,y+hh,z-hd);
        glVertex3f(x+hw,y+hh,z+hd); glVertex3f(x+hw,y,z+hd);
        glNormal3f(-1,0,0);
        glVertex3f(x-hw,y,z-hd); glVertex3f(x-hw,y,z+hd);
        glVertex3f(x-hw,y+hh,z+hd); glVertex3f(x-hw,y+hh,z-hd);
    glEnd();
}

// Draw a pitched roof (ridge along X axis) on top of a box.
static void drawRoof(float x,float y,float z, float hw,float rh,float hd,
                     float r,float g,float b){
    glColor3f(r,g,b);
    // Two sloped faces
    glBegin(GL_TRIANGLES);
        glNormal3f(0, hd, rh); // front slope (+Z face)
        glVertex3f(x-hw,y,z+hd); glVertex3f(x+hw,y,z+hd); glVertex3f(x+hw,y+rh,z);
        glVertex3f(x-hw,y,z+hd); glVertex3f(x+hw,y+rh,z); glVertex3f(x-hw,y+rh,z);
        glNormal3f(0, hd,-rh); // back slope (-Z face)
        glVertex3f(x+hw,y,z-hd); glVertex3f(x-hw,y,z-hd); glVertex3f(x-hw,y+rh,z);
        glVertex3f(x+hw,y,z-hd); glVertex3f(x-hw,y+rh,z); glVertex3f(x+hw,y+rh,z);
    glEnd();
    // Gable ends (triangles, not quads)
    glBegin(GL_TRIANGLES);
        glNormal3f(-1,0,0);
        glVertex3f(x-hw,y,z-hd); glVertex3f(x-hw,y,z+hd); glVertex3f(x-hw,y+rh,z);
        glNormal3f( 1,0,0);
        glVertex3f(x+hw,y,z+hd); glVertex3f(x+hw,y,z-hd); glVertex3f(x+hw,y+rh,z);
    glEnd();
}

// Draw a wind sock on a pole
static void drawWindSock(float x, float y, float z){
    glColor3f(0.85f,0.85f,0.88f); // pole
    glBegin(GL_QUADS);
        glNormal3f(1,0,0);
        glVertex3f(x+0.05f,y,z); glVertex3f(x+0.05f,y+2.5f,z);
        glVertex3f(x+0.05f,y+2.5f,z+0.05f); glVertex3f(x+0.05f,y,z+0.05f);
        glNormal3f(-1,0,0);
        glVertex3f(x,y,z); glVertex3f(x,y,z+0.05f);
        glVertex3f(x,y+2.5f,z+0.05f); glVertex3f(x,y+2.5f,z);
    glEnd();
    // sock cone: orange/white bands
    float bands[4][3]={{0.9f,0.4f,0.1f},{0.95f,0.95f,0.95f},{0.9f,0.4f,0.1f},{0.95f,0.95f,0.95f}};
    for(int b=0;b<4;b++){
        float r0=0.18f-b*0.03f, r1=r0-0.03f;
        float zy=y+2.5f, bh=0.25f;
        glColor3f(bands[b][0],bands[b][1],bands[b][2]);
        glBegin(GL_TRIANGLE_STRIP);
        for(int i=0;i<=8;i++){
            float a=i*3.14159f*2/8;
            float c=cosf(a),s=sinf(a);
            glNormal3f(c,0,s);
            glVertex3f(x+0.025f+c*r0, zy+b*bh,    z+0.025f+s*r0);
            glVertex3f(x+0.025f+c*r1, zy+(b+1)*bh, z+0.025f+s*r1);
        }
        glEnd();
    }
}

// Draw a cylindrical fuel tank
static void drawFuelTank(float x,float y,float z, float radius,float height){
    glColor3f(0.78f,0.78f,0.72f);
    glBegin(GL_TRIANGLE_STRIP);
    for(int i=0;i<=12;i++){
        float a=i*3.14159f*2/12;
        glNormal3f(cosf(a),0,sinf(a));
        glVertex3f(x+cosf(a)*radius, y+height, z+sinf(a)*radius);
        glVertex3f(x+cosf(a)*radius, y,        z+sinf(a)*radius);
    }
    glEnd();
    // caps
    for(int top=0;top<=1;top++){
        glNormal3f(0,top?1:-1,0);
        glBegin(GL_TRIANGLE_FAN);
            glVertex3f(x,y+top*height,z);
            for(int i=0;i<=12;i++){
                float a=i*3.14159f*2/12*(top?-1:1);
                glVertex3f(x+cosf(a)*radius,y+top*height,z+sinf(a)*radius);
            }
        glEnd();
    }
    // red band
    glColor3f(0.80f,0.15f,0.15f);
    glBegin(GL_TRIANGLE_STRIP);
    for(int i=0;i<=12;i++){
        float a=i*3.14159f*2/12;
        glNormal3f(cosf(a),0,sinf(a));
        glVertex3f(x+cosf(a)*(radius+0.01f), y+height*0.55f, z+sinf(a)*(radius+0.01f));
        glVertex3f(x+cosf(a)*(radius+0.01f), y+height*0.45f, z+sinf(a)*(radius+0.01f));
    }
    glEnd();
}

void renderAirport(const Airport *ap){
    float rl = ap->runwayLen;  // half-length
    float rw = ap->runwayW;    // half-width

    glDisable(GL_TEXTURE_2D);

    // Push a matrix: translate to airport centre, rotate by heading, raise to groundY.
    // All subsequent drawing uses local coords: +X = runway forward, +Z = runway side, Y = up.
    glPushMatrix();
    glTranslatef(ap->wx, ap->groundY, ap->wz);
    glRotatef(ap->heading * 57.2958f, 0,1,0);

    // ---- Runway asphalt ----
    glColor3f(0.22f,0.22f,0.22f);
    glNormal3f(0,1,0);
    glBegin(GL_QUADS);
        glVertex3f(-rl, 0.02f,-rw);
        glVertex3f( rl, 0.02f,-rw);
        glVertex3f( rl, 0.02f, rw);
        glVertex3f(-rl, 0.02f, rw);
    glEnd();

    // ---- Centreline dashes (white, 2.5u dash / 2.5u gap) ----
    glColor3f(0.90f,0.90f,0.90f);
    int ndash = (int)(rl * 2.0f / 5.0f);
    for(int d = 0; d < ndash; d++){
        float f0 = -rl + d * 5.0f + 0.5f;
        float f1 = f0 + 2.5f;
        glBegin(GL_QUADS);
            glVertex3f(f0, 0.04f,-0.20f); glVertex3f(f1, 0.04f,-0.20f);
            glVertex3f(f1, 0.04f, 0.20f); glVertex3f(f0, 0.04f, 0.20f);
        glEnd();
    }

    // ---- Threshold markings: 6 stripes, each 0.5u wide, spaced evenly inside rw ----
    glColor3f(0.90f,0.90f,0.90f);
    for(int end = -1; end <= 1; end += 2){
        float fBase  = end * rl;
        float fInner = fBase + end * 5.0f; // inset 5u from threshold
        // Distribute 6 stripes across 80% of runway width
        float spread = rw * 0.80f;
        for(int s = 0; s < 6; s++){
            float side = -spread + s * (spread * 2.0f / 5.0f);
            float sw = 0.30f; // stripe half-width
            glBegin(GL_QUADS);
                glVertex3f(fBase,  0.04f, side-sw); glVertex3f(fInner, 0.04f, side-sw);
                glVertex3f(fInner, 0.04f, side+sw); glVertex3f(fBase,  0.04f, side+sw);
            glEnd();
        }
    }

    // ---- Taxiway: main strip parallel to runway on +Z side ----
    float twW    = 3.5f;           // half-width of taxiway
    float twCenZ = rw + 2.0f + twW; // centreline Z
    float twInner = twCenZ - twW;
    float twOuter = twCenZ + twW;
    float twEndX  = rl * 0.85f;   // taxiway runs this far each direction

    glColor3f(0.25f,0.25f,0.25f);
    glNormal3f(0,1,0);
    glBegin(GL_QUADS); // main parallel strip
        glVertex3f(-twEndX, 0.02f, twInner); glVertex3f( twEndX, 0.02f, twInner);
        glVertex3f( twEndX, 0.02f, twOuter); glVertex3f(-twEndX, 0.02f, twOuter);
    glEnd();

    // Two connector stubs: runway edge (Z=rw) -> taxiway inner (Z=twInner), at each end
    for(int end = -1; end <= 1; end += 2){
        float stubX0 = end * twEndX - end * twW;
        float stubX1 = end * twEndX + end * twW;
        glBegin(GL_QUADS);
            glVertex3f(stubX0, 0.02f, rw);      glVertex3f(stubX1, 0.02f, rw);
            glVertex3f(stubX1, 0.02f, twOuter); glVertex3f(stubX0, 0.02f, twOuter);
        glEnd();
    }

    // Yellow centreline — main strip
    glColor3f(0.90f,0.75f,0.10f);
    for(int d = 0; ; d++){
        float f0 = -twEndX + d * 5.0f + 0.5f;
        float f1 = f0 + 2.5f;
        if(f0 > twEndX) break;
        if(f1 > twEndX) f1 = twEndX;
        glBegin(GL_QUADS);
            glVertex3f(f0, 0.04f, twCenZ-0.12f); glVertex3f(f1, 0.04f, twCenZ-0.12f);
            glVertex3f(f1, 0.04f, twCenZ+0.12f); glVertex3f(f0, 0.04f, twCenZ+0.12f);
        glEnd();
    }
    // Yellow centreline — connectors (along Z axis)
    for(int end = -1; end <= 1; end += 2){
        float cx = end * twEndX;
        for(int d = 0; ; d++){
            float z0 = rw + d * 5.0f + 0.5f;
            float z1 = z0 + 2.5f;
            if(z0 > twOuter) break;
            if(z1 > twOuter) z1 = twOuter;
            glBegin(GL_QUADS);
                glVertex3f(cx-0.12f, 0.04f, z0); glVertex3f(cx+0.12f, 0.04f, z0);
                glVertex3f(cx+0.12f, 0.04f, z1); glVertex3f(cx-0.12f, 0.04f, z1);
            glEnd();
        }
    }

    // ---- Apron: connects from taxiway outer edge to terminal ----
    int   ngates     = 2 + ap->size * 2;  // 2/4/6 gates
    float gateSpacing = 8.0f;
    float apronDepth  = 20.0f;
    float apronZ0 = twOuter;
    float apronZ1 = twOuter + apronDepth;
    float apronHalfX = (ngates - 1) * gateSpacing * 0.5f + 5.0f;

    glColor3f(0.28f,0.28f,0.28f);
    glNormal3f(0,1,0);
    glBegin(GL_QUADS);
        glVertex3f(-apronHalfX, 0.02f, apronZ0); glVertex3f( apronHalfX, 0.02f, apronZ0);
        glVertex3f( apronHalfX, 0.02f, apronZ1); glVertex3f(-apronHalfX, 0.02f, apronZ1);
    glEnd();

    // Taxiway connector from taxiway to apron (short stub bridging any gap)
    glColor3f(0.25f,0.25f,0.25f);
    glBegin(GL_QUADS);
        glVertex3f(-apronHalfX, 0.02f, twInner); glVertex3f( apronHalfX, 0.02f, twInner);
        glVertex3f( apronHalfX, 0.02f, apronZ0); glVertex3f(-apronHalfX, 0.02f, apronZ0);
    glEnd();

    // ---- Gate markings + jetway arms ----
    for(int g = 0; g < ngates; g++){
        float gx = -apronHalfX + 5.0f + g * gateSpacing;
        float gzFront = apronZ0 + 2.0f;   // nose-stop line Z
        float gzBack  = apronZ1 - 1.0f;

        // Painted aircraft stand box (yellow)
        glColor3f(0.90f,0.80f,0.10f);
        glNormal3f(0,1,0);
        // side lines
        glBegin(GL_QUADS);
            glVertex3f(gx-2.8f, 0.05f, gzFront); glVertex3f(gx-2.5f, 0.05f, gzFront);
            glVertex3f(gx-2.5f, 0.05f, gzBack);  glVertex3f(gx-2.8f, 0.05f, gzBack);
            glVertex3f(gx+2.5f, 0.05f, gzFront); glVertex3f(gx+2.8f, 0.05f, gzFront);
            glVertex3f(gx+2.8f, 0.05f, gzBack);  glVertex3f(gx+2.5f, 0.05f, gzBack);
        glEnd();
        // nose-stop bar
        glBegin(GL_QUADS);
            glVertex3f(gx-2.8f, 0.05f, gzFront);       glVertex3f(gx+2.8f, 0.05f, gzFront);
            glVertex3f(gx+2.8f, 0.05f, gzFront+0.30f); glVertex3f(gx-2.8f, 0.05f, gzFront+0.30f);
        glEnd();
        // centreline lead-in (dashed, from taxiway to stand)
        glColor3f(0.90f,0.75f,0.10f);
        for(int d = 0; ; d++){
            float z0 = apronZ0 + d * 3.0f;
            float z1 = z0 + 1.5f;
            if(z0 > gzFront) break;
            if(z1 > gzFront) z1 = gzFront;
            glBegin(GL_QUADS);
                glVertex3f(gx-0.12f, 0.05f, z0); glVertex3f(gx+0.12f, 0.05f, z0);
                glVertex3f(gx+0.12f, 0.05f, z1); glVertex3f(gx-0.12f, 0.05f, z1);
            glEnd();
        }

        // Gate number painted on apron (simple T shape as proxy)
        glColor3f(0.85f,0.85f,0.85f);
        glBegin(GL_QUADS);
            glVertex3f(gx-0.5f,0.05f,gzFront+1.0f); glVertex3f(gx+0.5f,0.05f,gzFront+1.0f);
            glVertex3f(gx+0.5f,0.05f,gzFront+1.3f); glVertex3f(gx-0.5f,0.05f,gzFront+1.3f);
        glEnd();

        // Jetway: horizontal arm from terminal face to aircraft door height
        float jwY    = 2.2f;  // bridge height
        float jwArmZ = apronZ1; // terminal face
        // Support pillar
        glColor3f(0.55f,0.55f,0.58f);
        glBegin(GL_QUADS);
            glNormal3f(1,0,0);
            glVertex3f(gx+0.15f,0,    gzBack-0.5f); glVertex3f(gx+0.15f,jwY,  gzBack-0.5f);
            glVertex3f(gx+0.15f,jwY,  gzBack+0.5f); glVertex3f(gx+0.15f,0,    gzBack+0.5f);
            glNormal3f(-1,0,0);
            glVertex3f(gx-0.15f,0,    gzBack-0.5f); glVertex3f(gx-0.15f,0,    gzBack+0.5f);
            glVertex3f(gx-0.15f,jwY,  gzBack+0.5f); glVertex3f(gx-0.15f,jwY,  gzBack-0.5f);
        glEnd();
        // Horizontal bridge tunnel
        float armLen = gzBack - gzFront - 3.0f;
        if(armLen > 1.0f){
            drawBox(gx, jwY, gzFront + 3.0f + armLen*0.5f,
                    0.8f, 0.9f, armLen*0.5f,  0.60f,0.62f,0.65f);
        }
    }

    // ---- Buildings (all in local space) ----
    float bldgZ = apronZ1; // terminal face flush with apron back edge

    // All sizes: terminal building spans gate width
    drawBox(0, 0, bldgZ + 4.0f,  apronHalfX, 3.5f, 4.0f,  0.82f,0.80f,0.74f);
    drawRoof(0, 3.5f, bldgZ + 4.0f, apronHalfX, 1.2f, 4.0f, 0.55f,0.28f,0.20f);
    // Terminal windows (blue strip along apron-facing facade)
    glColor3f(0.28f,0.44f,0.68f);
    glNormal3f(0,0,-1);
    glBegin(GL_QUADS);
        glVertex3f(-apronHalfX+0.5f, 1.0f, bldgZ+0.02f);
        glVertex3f( apronHalfX-0.5f, 1.0f, bldgZ+0.02f);
        glVertex3f( apronHalfX-0.5f, 2.8f, bldgZ+0.02f);
        glVertex3f(-apronHalfX+0.5f, 2.8f, bldgZ+0.02f);
    glEnd();

    if(ap->size >= 1){ // medium: hangar beside terminal
        float hx = apronHalfX + 7.0f;
        drawBox(hx, 0, bldgZ+5.0f,  6.0f, 4.5f, 5.5f,  0.58f,0.60f,0.63f);
        // hangar door (dark, faces apron)
        glColor3f(0.18f,0.18f,0.20f);
        glNormal3f(0,0,-1);
        glBegin(GL_QUADS);
            glVertex3f(hx-5.0f, 0.02f, bldgZ-0.05f); glVertex3f(hx+5.0f, 0.02f, bldgZ-0.05f);
            glVertex3f(hx+5.0f, 3.8f,  bldgZ-0.05f); glVertex3f(hx-5.0f, 3.8f,  bldgZ-0.05f);
        glEnd();
    }

    if(ap->size >= 2){ // large: control tower + second hangar + fuel tanks
        float tx = apronHalfX + 3.0f;
        // Tower shaft
        drawBox(tx, 0, bldgZ+2.0f,  1.4f, 7.0f, 1.4f,  0.80f,0.80f,0.76f);
        // Tower cab (wider than shaft)
        drawBox(tx, 7.0f, bldgZ+2.0f,  2.2f, 1.8f, 2.2f,  0.74f,0.77f,0.80f);
        // Cab windows on all 4 sides
        glColor3f(0.28f,0.44f,0.68f);
        float cw = 1.8f, cy0 = 7.1f, cy1 = 8.5f;
        glNormal3f(0,0,-1); glBegin(GL_QUADS);
            glVertex3f(tx-cw,cy0,bldgZ-0.22f); glVertex3f(tx+cw,cy0,bldgZ-0.22f);
            glVertex3f(tx+cw,cy1,bldgZ-0.22f); glVertex3f(tx-cw,cy1,bldgZ-0.22f);
        glEnd();
        glNormal3f(0,0,1); glBegin(GL_QUADS);
            glVertex3f(tx+cw,cy0,bldgZ+4.22f); glVertex3f(tx-cw,cy0,bldgZ+4.22f);
            glVertex3f(tx-cw,cy1,bldgZ+4.22f); glVertex3f(tx+cw,cy1,bldgZ+4.22f);
        glEnd();
        glNormal3f(1,0,0); glBegin(GL_QUADS);
            glVertex3f(tx+2.22f,cy0,bldgZ-0.2f); glVertex3f(tx+2.22f,cy0,bldgZ+4.2f);
            glVertex3f(tx+2.22f,cy1,bldgZ+4.2f); glVertex3f(tx+2.22f,cy1,bldgZ-0.2f);
        glEnd();
        glNormal3f(-1,0,0); glBegin(GL_QUADS);
            glVertex3f(tx-2.22f,cy0,bldgZ+4.2f); glVertex3f(tx-2.22f,cy0,bldgZ-0.2f);
            glVertex3f(tx-2.22f,cy1,bldgZ-0.2f); glVertex3f(tx-2.22f,cy1,bldgZ+4.2f);
        glEnd();
        // Second hangar
        float hx2 = -(apronHalfX + 7.0f);
        drawBox(hx2, 0, bldgZ+5.0f,  6.5f, 5.0f, 6.0f,  0.55f,0.57f,0.60f);
        glColor3f(0.18f,0.18f,0.20f);
        glNormal3f(0,0,-1);
        glBegin(GL_QUADS);
            glVertex3f(hx2-5.5f,0.02f,bldgZ-0.05f); glVertex3f(hx2+5.5f,0.02f,bldgZ-0.05f);
            glVertex3f(hx2+5.5f,4.2f, bldgZ-0.05f); glVertex3f(hx2-5.5f,4.2f, bldgZ-0.05f);
        glEnd();
        // Fuel tanks beside second hangar
        drawFuelTank(hx2-9.0f, 0, bldgZ+1.0f, 1.5f, 3.2f);
        drawFuelTank(hx2-12.5f,0, bldgZ+1.0f, 1.5f, 3.2f);
    }

    // ---- Wind sock on pole, end of runway ----
    drawWindSock(rl*0.65f, 0, rw+2.0f);

    // ---- Runway edge lights ----
    int nlights = (int)(rl * 2.0f / 8.0f);
    for(int l = 0; l <= nlights; l++){
        float fwd = -rl + l * (rl * 2.0f / nlights);
        for(int s = -1; s <= 1; s += 2){
            float sz = s * (rw + 0.5f);
            // post
            glColor3f(0.35f,0.35f,0.35f);
            glBegin(GL_QUADS);
                glNormal3f(1,0,0);
                glVertex3f(fwd,      0,   sz);      glVertex3f(fwd+0.06f,0,   sz);
                glVertex3f(fwd+0.06f,0.5f,sz);      glVertex3f(fwd,      0.5f,sz);
            glEnd();
            // light cap
            glColor3f(1.0f,0.92f,0.25f);
            glBegin(GL_QUADS);
                glNormal3f(0,1,0);
                glVertex3f(fwd-0.08f,0.52f,sz-0.08f); glVertex3f(fwd+0.14f,0.52f,sz-0.08f);
                glVertex3f(fwd+0.14f,0.52f,sz+0.08f); glVertex3f(fwd-0.08f,0.52f,sz+0.08f);
            glEnd();
        }
    }

    glPopMatrix();
    glEnable(GL_TEXTURE_2D);
}

void renderAllAirports(){
    for(int i=0;i<numAirports;i++) renderAirport(&airports[i]);
}

void renderTerrain(){
    float offX=-TERRAIN_SIZE*TERRAIN_SCALE*0.5f;
    float offZ=-TERRAIN_SIZE*TERRAIN_SCALE*0.5f;
    glDisable(GL_TEXTURE_2D);
    // Triangle strips: for each row z, emit vertices alternating between row z and z+1
    for(int z=0;z<TERRAIN_SIZE;z++){
        glBegin(GL_TRIANGLE_STRIP);
        for(int x=0;x<=TERRAIN_SIZE;x++){
            // vertex on row z+1 first, then row z (standard strip order)
            glNormal3f(terrainNX[z+1][x], terrainNY[z+1][x], terrainNZ[z+1][x]);
            terrainColor(terrainH[z+1][x], terrainNX[z+1][x], terrainNY[z+1][x], terrainNZ[z+1][x]);
            glVertex3f(offX+x*TERRAIN_SCALE, terrainH[z+1][x], offZ+(z+1)*TERRAIN_SCALE);

            glNormal3f(terrainNX[z][x], terrainNY[z][x], terrainNZ[z][x]);
            terrainColor(terrainH[z][x], terrainNX[z][x], terrainNY[z][x], terrainNZ[z][x]);
            glVertex3f(offX+x*TERRAIN_SCALE, terrainH[z][x], offZ+z*TERRAIN_SCALE);
        }
        glEnd();
    }
    glEnable(GL_TEXTURE_2D);
}

// -------- Procedural Ground --------
void generateCheckerboardTexture() {
    GLubyte texData[TEX_SIZE][TEX_SIZE][3];
    for(int i=0;i<TEX_SIZE;i++)
        for(int j=0;j<TEX_SIZE;j++)
        {
            int c = ((i/8 + j/8) % 2) * 255;
            texData[i][j][0] = texData[i][j][1] = texData[i][j][2] = (GLubyte)c;
        }
    glGenTextures(1,&groundTexture);
    glBindTexture(GL_TEXTURE_2D,groundTexture);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,TEX_SIZE,TEX_SIZE,0,GL_RGB,GL_UNSIGNED_BYTE,texData);
    glBindTexture(GL_TEXTURE_2D,0);
}

// -------- FPS --------
void renderFPS() {
    double currentTime = glutGet(GLUT_ELAPSED_TIME)/1000.0;
    frames++;
    if(currentTime-lastTime>=1.0){ fps=frames/(currentTime-lastTime); frames=0; lastTime=currentTime; }
    char buffer[64]; sprintf(buffer,"FPS: %.1f",fps);

    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); gluOrtho2D(0,resWidth,0,resHeight);
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
    glColor3f(1,1,1); glRasterPos2i(10,resHeight-20);
    for(char *c=buffer;*c;c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,*c);
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);
}

// -------- Network Thread --------
void* networkThreadFunc(void* arg){
    while(running){
        // Receive data first so server can learn client address before sending
        char buffer[256]; struct sockaddr_in from; socklen_t fromlen=sizeof(from); int n;
        while((n=recvfrom(sockfd,buffer,sizeof(buffer),0,(struct sockaddr*)&from,&fromlen))>0){
            int *recvType=(int*)buffer;
            if(*recvType==MSG_CONNECT){
                printf("[LOG] A player connected from %s:%d!\n",
                    inet_ntoa(from.sin_addr), ntohs(from.sin_port));
                if(isServer){
                    otherAddr = from;
                    clientConnected = true;
                    // Send terrain seed so client generates identical terrain
                    char seedPkt[sizeof(int)+sizeof(unsigned int)];
                    int stype = MSG_TERRAIN_SEED;
                    memcpy(seedPkt, &stype, sizeof(int));
                    memcpy(seedPkt+sizeof(int), &terrainSeed, sizeof(unsigned int));
                    sendto(sockfd,seedPkt,sizeof(seedPkt),0,(struct sockaddr*)&otherAddr,sizeof(otherAddr));
                }
            } else if(*recvType==MSG_TERRAIN_SEED && n==(int)(sizeof(int)+sizeof(unsigned int))){
                if(!isServer && !seedReceived){
                    unsigned int seed;
                    memcpy(&seed, buffer+sizeof(int), sizeof(unsigned int));
                    printf("[NET] Received terrain seed %u from server — regenerating terrain\n", seed);
                    generateAllTerrain(seed);
                    seedReceived = true;
                    // Reset player to first airport after terrain regenerates
                    pthread_mutex_lock(&playerMutex);
                    player.x=0; player.y=airports[0].groundY+1.0f; player.z=0;
                    player.pitch=0; player.yaw=0; player.roll=0;
                    pthread_mutex_unlock(&playerMutex);
                }
            } else if(*recvType==MSG_TORNADO && n==(int)(sizeof(int)+sizeof(int)+MAX_TORNADOS*(int)(sizeof(float)*4))){
                if(!isServer){
                    char *p = buffer+sizeof(int);
                    int enabled; memcpy(&enabled,p,sizeof(int)); p+=sizeof(int);
                    pthread_mutex_lock(&tornadoMutex);
                    tornadoEnabled = (bool)enabled;
                    for(int i=0;i<MAX_TORNADOS;i++){
                        memcpy(&tornados[i].x,     p,sizeof(float)); p+=sizeof(float);
                        memcpy(&tornados[i].z,     p,sizeof(float)); p+=sizeof(float);
                        memcpy(&tornados[i].angle, p,sizeof(float)); p+=sizeof(float);
                        memcpy(&tornados[i].height,p,sizeof(float)); p+=sizeof(float);
                        tornados[i].active = tornadoEnabled;
                    }
                    pthread_mutex_unlock(&tornadoMutex);
                }
            } else if(*recvType==MSG_PLAYER_DATA && n==(int)(sizeof(int)+sizeof(float)*14)){
                // x,y,z,pitch,yaw,roll,throttle, walkerX,walkerY,walkerZ,walkerYaw, inPlane,type,isPassenger
                float *rdata=(float*)(buffer+sizeof(int));
                pthread_mutex_lock(&remoteMutex);
                remotePlayers[0].x=rdata[0]; remotePlayers[0].y=rdata[1]; remotePlayers[0].z=rdata[2];
                remotePlayers[0].pitch=rdata[3]; remotePlayers[0].yaw=rdata[4]; remotePlayers[0].roll=rdata[5];
                remotePlayers[0].throttle=rdata[6];
                remotePlayers[0].walkerX=rdata[7]; remotePlayers[0].walkerY=rdata[8]; remotePlayers[0].walkerZ=rdata[9];
                remotePlayers[0].walkerYaw=rdata[10];
                remotePlayers[0].inPlane=(rdata[11]>0.5f);
                remotePlayers[0].type=(PlaneType)(int)rdata[12];
                remotePlayers[0].isPassenger=(rdata[13]>0.5f);
                remotePlayers[0].alive=true;
                pthread_mutex_unlock(&remoteMutex);
                packetsRecv++;
            } else if(*recvType==MSG_EXPLOSION && n==(int)(sizeof(int)+sizeof(float)*4)){
                float *ed=(float*)(buffer+sizeof(int));
                receiveExplosion(ed[0],ed[1],ed[2],ed[3]);
                // Server relays explosion to all other clients
                if(isServer && clientConnected)
                    sendto(sockfd,buffer,n,0,(struct sockaddr*)&otherAddr,sizeof(otherAddr));
            }
        }

        if(!isServer && !seedReceived){
            int msg = MSG_CONNECT;
            sendto(sockfd,&msg,sizeof(msg),0,(struct sockaddr*)&otherAddr,sizeof(otherAddr));
        } else if(isServer ? clientConnected : seedReceived){
            int msgType = MSG_PLAYER_DATA;
            char packet[sizeof(int)+sizeof(float)*14];
            memcpy(packet,&msgType,sizeof(int));
            pthread_mutex_lock(&playerMutex);
            float data[14]={player.x,player.y,player.z,player.pitch,player.yaw,player.roll,player.throttle,
                            walkerX,walkerY,walkerZ,walkerYaw,
                            inPlane?1.0f:0.0f,(float)player.type,isPassenger?1.0f:0.0f};
            pthread_mutex_unlock(&playerMutex);
            memcpy(packet+sizeof(int),data,sizeof(data));
            sendto(sockfd,packet,sizeof(packet),0,(struct sockaddr*)&otherAddr,sizeof(otherAddr));
            packetsSent++;

            if(isServer){
                int tsize = sizeof(int)+sizeof(int)+MAX_TORNADOS*(int)(sizeof(float)*4);
                char tpkt[tsize];
                char *p = tpkt;
                int tmsg = MSG_TORNADO; memcpy(p,&tmsg,sizeof(int)); p+=sizeof(int);
                pthread_mutex_lock(&tornadoMutex);
                int enabled=(int)tornadoEnabled; memcpy(p,&enabled,sizeof(int)); p+=sizeof(int);
                for(int i=0;i<MAX_TORNADOS;i++){
                    memcpy(p,&tornados[i].x,     sizeof(float)); p+=sizeof(float);
                    memcpy(p,&tornados[i].z,     sizeof(float)); p+=sizeof(float);
                    memcpy(p,&tornados[i].angle, sizeof(float)); p+=sizeof(float);
                    memcpy(p,&tornados[i].height,sizeof(float)); p+=sizeof(float);
                }
                pthread_mutex_unlock(&tornadoMutex);
                sendto(sockfd,tpkt,tsize,0,(struct sockaddr*)&otherAddr,sizeof(otherAddr));
            }

            // Flush outbound explosion queue
            pthread_mutex_lock(&explodeMutex);
            while(explodeQueueHead != explodeQueueTail){
                ExplodeQueueEntry eq = explodeQueue[explodeQueueHead];
                explodeQueueHead = (explodeQueueHead+1) % MAX_EXPLODE_QUEUE;
                pthread_mutex_unlock(&explodeMutex);

                int emsg = MSG_EXPLOSION;
                char epkt[sizeof(int)+sizeof(float)*4];
                memcpy(epkt, &emsg, sizeof(int));
                float edata[4] = {eq.x, eq.y, eq.z, eq.scale};
                memcpy(epkt+sizeof(int), edata, sizeof(edata));
                sendto(sockfd,epkt,sizeof(epkt),0,(struct sockaddr*)&otherAddr,sizeof(otherAddr));

                pthread_mutex_lock(&explodeMutex);
            }
            pthread_mutex_unlock(&explodeMutex);
        }

        usleep(16000);
    }
    return NULL;
}

// -------- Weather --------
static void weatherPreset(WeatherType t, float *skyR, float *skyG, float *skyB,
                           float *fog, float *rain, float *wX, float *wZ){
    float h = ((float)rand()/RAND_MAX)*2.0f*(float)M_PI;
    float spd;
    switch(t){
        case WX_CLEAR:
            *skyR=0.50f; *skyG=0.78f; *skyB=1.00f;
            *fog=0.0006f; *rain=0.0f;
            spd=((float)rand()/RAND_MAX)*1.5f;
            break;
        case WX_CLOUDY:
            *skyR=0.55f; *skyG=0.65f; *skyB=0.80f;
            *fog=0.0018f; *rain=0.0f;
            spd=1.0f+((float)rand()/RAND_MAX)*2.0f;
            break;
        case WX_OVERCAST:
            *skyR=0.40f; *skyG=0.44f; *skyB=0.50f;
            *fog=0.0035f; *rain=0.25f;
            spd=2.0f+((float)rand()/RAND_MAX)*3.0f;
            break;
        case WX_STORM:
            *skyR=0.22f; *skyG=0.24f; *skyB=0.28f;
            *fog=0.006f;  *rain=1.0f;
            spd=5.0f+((float)rand()/RAND_MAX)*5.0f;
            break;
        default:
            *skyR=0.5f; *skyG=0.78f; *skyB=1.0f;
            *fog=0.0006f; *rain=0.0f; spd=0; break;
    }
    *wX = cosf(h)*spd;
    *wZ = sinf(h)*spd;
}

void initWeather(){
    weather.type = WX_CLEAR;
    weatherPreset(WX_CLEAR, &weather.skyR,&weather.skyG,&weather.skyB,
                  &weather.fogDensity,&weather.rainIntensity,
                  &weather.windX,&weather.windZ);
    weather.transitionTimer = 30.0f + ((float)rand()/RAND_MAX)*60.0f;
    weather.transitionSpeed = 0.4f;
}

void updateWeather(float dt){
    weather.transitionTimer -= dt;
    if(weather.transitionTimer <= 0.0f){
        // Pick next type with weighted transitions
        int r = rand()%4;
        static const WeatherType next[4][4] = {
            {WX_CLEAR, WX_CLEAR, WX_CLOUDY,   WX_CLOUDY},   // from CLEAR
            {WX_CLEAR, WX_CLOUDY, WX_OVERCAST, WX_OVERCAST}, // from CLOUDY
            {WX_CLOUDY,WX_OVERCAST,WX_OVERCAST,WX_STORM},    // from OVERCAST
            {WX_OVERCAST,WX_OVERCAST,WX_STORM, WX_CLOUDY},   // from STORM
        };
        WeatherType newType = next[weather.type][r];
        weather.type = newType;
        float tR,tG,tB,tFog,tRain,tWX,tWZ;
        weatherPreset(newType,&tR,&tG,&tB,&tFog,&tRain,&tWX,&tWZ);
        float sp = dt * weather.transitionSpeed;
        weather.skyR += (tR - weather.skyR)*sp*60;
        weather.skyG += (tG - weather.skyG)*sp*60;
        weather.skyB += (tB - weather.skyB)*sp*60;
        weather.fogDensity    += (tFog  - weather.fogDensity)*sp*60;
        weather.rainIntensity += (tRain - weather.rainIntensity)*sp*60;
        weather.windX += (tWX - weather.windX)*sp*60;
        weather.windZ += (tWZ - weather.windZ)*sp*60;
        weather.transitionTimer = 20.0f + ((float)rand()/RAND_MAX)*80.0f;
    }
    // Smooth per-frame lerp toward current targets
    float sp = dt * 0.15f;
    float tR,tG,tB,tFog,tRain,tWX,tWZ;
    weatherPreset(weather.type,&tR,&tG,&tB,&tFog,&tRain,&tWX,&tWZ);
    weather.skyR += (tR - weather.skyR)*sp;
    weather.skyG += (tG - weather.skyG)*sp;
    weather.skyB += (tB - weather.skyB)*sp;
    weather.fogDensity    += (tFog  - weather.fogDensity)*sp;
    weather.rainIntensity += (tRain - weather.rainIntensity)*sp;
    weather.windX += (tWX - weather.windX)*sp;
    weather.windZ += (tWZ - weather.windZ)*sp;

    // Storm boosts turbulence target
    if(weather.type == WX_STORM)
        turbulenceTarget = clampf(turbulenceTarget, 0.5f, 1.0f);

    // Apply fog
    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, GL_EXP2);
    float fogCol[4] = {weather.skyR, weather.skyG, weather.skyB, 1.0f};
    glFogfv(GL_FOG_COLOR, fogCol);
    glFogf(GL_FOG_DENSITY, weather.fogDensity);
    glClearColor(weather.skyR, weather.skyG, weather.skyB, 1.0f);
}

// Render rain as streaks in screen space
void renderRain(){
    if(weather.rainIntensity < 0.02f) return;

    if(!rainInited){
        for(int i=0;i<MAX_RAIN;i++){
            rain[i].x = ((float)rand()/RAND_MAX)*2.0f-1.0f; // NDC-ish -1..1
            rain[i].y = ((float)rand()/RAND_MAX)*2.0f-1.0f;
            rain[i].z = ((float)rand()/RAND_MAX); // depth layer (speed variation)
            rain[i].speed = 0.6f + rain[i].z*0.8f;
        }
        rainInited = true;
    }

    // Advance drops (in a normalised 2D screen space)
    float dt = DT;
    float windSlant = weather.windX * 0.002f;
    for(int i=0;i<MAX_RAIN;i++){
        rain[i].y -= rain[i].speed * dt * 1.2f;
        rain[i].x += windSlant;
        if(rain[i].y < -1.0f){ rain[i].y = 1.0f; rain[i].x = ((float)rand()/RAND_MAX)*2.0f-1.0f; }
        if(rain[i].x >  1.0f || rain[i].x < -1.0f) rain[i].x = ((float)rand()/RAND_MAX)*2.0f-1.0f;
    }

    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); gluOrtho2D(-1,1,-1,1);
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    int visible = (int)(MAX_RAIN * weather.rainIntensity);
    float alpha = weather.rainIntensity * 0.55f;
    glColor4f(0.72f, 0.82f, 0.95f, alpha);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for(int i=0;i<visible;i++){
        float len = 0.03f + rain[i].z*0.02f;
        glVertex2f(rain[i].x,             rain[i].y);
        glVertex2f(rain[i].x + windSlant*6, rain[i].y + len);
    }
    glEnd();

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);
}

// -------- Explosions --------
static void updateExplosions(float dt){
    for(int i=0;i<MAX_EXPLOSIONS;i++){
        if(!explosions[i].active) continue;
        explosions[i].timer += dt;
        if(explosions[i].timer >= EXPLOSION_LIFETIME) explosions[i].active = false;
    }
}

static void renderExplosions(){
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);  // additive — bright fire look

    for(int i=0;i<MAX_EXPLOSIONS;i++){
        if(!explosions[i].active) continue;
        float t  = explosions[i].timer / EXPLOSION_LIFETIME; // 0→1
        float cx = explosions[i].x;
        float cy = explosions[i].y;
        float cz = explosions[i].z;

        for(int p=0;p<EXPLOSION_PARTICLES;p++){
            // Particle position: accelerates out then slows (ease-out)
            float dist = explosions[i].ps[p] * (1.0f - (1.0f-t)*(1.0f-t)) * 12.0f;
            float px = cx + explosions[i].px[p] * dist;
            float py = cy + explosions[i].py[p] * dist;
            float pz = cz + explosions[i].pz[p] * dist;

            // Size: grows then shrinks, scaled with particle speed
            float szScale = 0.5f + explosions[i].ps[p] * 0.08f;
            float sz = (0.4f + t * 1.2f) * (1.0f - t*0.6f) * szScale;

            // Colour: white→orange→red→dark smoke
            float r,g,b,a;
            if(t < 0.15f){       // flash — white/yellow
                float ft = t/0.15f;
                r=1.0f; g=1.0f; b=1.0f-ft; a=1.0f;
            } else if(t < 0.45f){ // orange fireball
                float ft=(t-0.15f)/0.30f;
                r=1.0f; g=0.6f-ft*0.4f; b=0.0f; a=1.0f-ft*0.3f;
            } else {              // smoke — dark grey fading out
                float ft=(t-0.45f)/0.55f;
                r=0.25f-ft*0.2f; g=r; b=r; a=0.5f-ft*0.5f;
            }

            glColor4f(r,g,b,a);
            glBegin(GL_QUADS);
            glVertex3f(px-sz, py-sz, pz);
            glVertex3f(px+sz, py-sz, pz);
            glVertex3f(px+sz, py+sz, pz);
            glVertex3f(px-sz, py+sz, pz);
            glEnd();
        }
    }

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
}

// -------- Tornadoes --------
static void spawnTornados(){
    float worldHalf = TERRAIN_SIZE * TERRAIN_SCALE * 0.5f;
    for(int i=0;i<MAX_TORNADOS;i++){
        tornados[i].x      = ((float)rand()/RAND_MAX*2-1)*(worldHalf-30);
        tornados[i].z      = ((float)rand()/RAND_MAX*2-1)*(worldHalf-30);
        tornados[i].angle  = ((float)rand()/RAND_MAX)*2*(float)M_PI;
        tornados[i].height = 0.0f;
        tornados[i].active = true;
        float dir = ((float)rand()/RAND_MAX)*2*(float)M_PI;
        float spd = 8.0f + ((float)rand()/RAND_MAX)*7.0f;
        tornados[i].vx = cosf(dir)*spd;
        tornados[i].vz = sinf(dir)*spd;
    }
}

static void updateTornados(float dt){
    pthread_mutex_lock(&tornadoMutex);
    if(!tornadoEnabled){ pthread_mutex_unlock(&tornadoMutex); return; }

    float worldHalfT = TERRAIN_SIZE * TERRAIN_SCALE * 0.5f;
    for(int i=0;i<MAX_TORNADOS;i++){
        if(!tornados[i].active) continue;

        // Faster spin
        tornados[i].angle += 10.0f*dt;

        // Animate height
        if(tornados[i].height < 60.0f)
            tornados[i].height += 15.0f*dt;

        // Move across terrain
        tornados[i].x += tornados[i].vx * dt;
        tornados[i].z += tornados[i].vz * dt;

        // Bounce off world edges
        if(tornados[i].x >  worldHalfT-20){ tornados[i].x =  worldHalfT-20; tornados[i].vx *= -1; }
        if(tornados[i].x < -worldHalfT+20){ tornados[i].x = -worldHalfT+20; tornados[i].vx *= -1; }
        if(tornados[i].z >  worldHalfT-20){ tornados[i].z =  worldHalfT-20; tornados[i].vz *= -1; }
        if(tornados[i].z < -worldHalfT+20){ tornados[i].z = -worldHalfT+20; tornados[i].vz *= -1; }

        // Slightly steer toward player
        float dx = player.x - tornados[i].x;
        float dz = player.z - tornados[i].z;
        float dist = sqrtf(dx*dx+dz*dz);
        if(dist > 0.1f){
            tornados[i].vx += (dx/dist)*1.5f*dt;
            tornados[i].vz += (dz/dist)*1.5f*dt;
            float spd = sqrtf(tornados[i].vx*tornados[i].vx + tornados[i].vz*tornados[i].vz);
            if(spd > 18.0f){ tornados[i].vx = tornados[i].vx/spd*18.0f; tornados[i].vz = tornados[i].vz/spd*18.0f; }
        }

        // Affect player if close
        if(dist < 40.0f && dist > 0.5f){
            float strength = (1.0f - dist/40.0f) * 18.0f;
            pthread_mutex_lock(&playerMutex);
            player.x -= (dx/dist)*strength*dt;
            player.z -= (dz/dist)*strength*dt;
            player.y += strength*0.6f*dt;
            pthread_mutex_unlock(&playerMutex);
            turbulenceTarget = 1.0f;
        }
    }
    pthread_mutex_unlock(&tornadoMutex);
}

static void renderTornado(Tornado *t){
    float gndY = terrainHeightAt(t->x, t->z);
    float maxH  = t->height;
    int   rings  = 28;
    int   pts    = 12;

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Funnel: wide at top (cloud base), narrow tip at ground — classic tornado shape
    for(int r=0; r<rings; r++){
        float frac  = (float)r/(rings-1);          // 0=ground tip, 1=top cloud
        float y     = gndY + frac * maxH;
        float radius= 1.0f + frac*frac * 9.0f;     // ~1 at tip, ~10 at top
        float phase = t->angle + frac * 6.0f;       // helix twist
        float alpha = 0.12f + frac*0.30f;           // denser at top

        // Colour: dark at tip, lighter/greyer toward cloud
        float rv = 0.20f + frac*0.35f;
        float gv = 0.18f + frac*0.28f;
        float bv = 0.18f + frac*0.28f;
        glColor4f(rv,gv,bv,alpha);

        glBegin(GL_TRIANGLE_STRIP);
        for(int p=0; p<=pts; p++){
            float a  = phase + (float)p/pts * 2.0f*(float)M_PI;
            float frac1 = (r+1 < rings) ? (float)(r+1)/(rings-1) : 1.0f;
            float r1 = 1.0f + frac1*frac1 * 9.0f;
            float y1 = gndY + frac1 * maxH;
            float ph1= t->angle + frac1 * 6.0f;
            float a1 = ph1 + (float)p/pts * 2.0f*(float)M_PI;
            glVertex3f(t->x + cosf(a)*radius, y,  t->z + sinf(a)*radius);
            glVertex3f(t->x + cosf(a1)*r1,    y1, t->z + sinf(a1)*r1);
        }
        glEnd();
    }

    // Debris ring at base — small quads orbiting
    glColor4f(0.20f,0.15f,0.10f,0.7f);
    float debrisR = 9.5f;
    glBegin(GL_QUADS);
    for(int d=0;d<16;d++){
        float a  = t->angle*2.0f + (float)d/16*2*(float)M_PI;
        float cx = t->x + cosf(a)*debrisR;
        float cz = t->z + sinf(a)*debrisR;
        float cy = gndY + 0.5f + sinf(t->angle*3+d)*0.8f;
        float s  = 0.5f;
        glVertex3f(cx-s, cy,   cz-s);
        glVertex3f(cx+s, cy,   cz-s);
        glVertex3f(cx+s, cy+s, cz+s);
        glVertex3f(cx-s, cy+s, cz+s);
    }
    glEnd();

    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}

static void renderAllTornados(){
    pthread_mutex_lock(&tornadoMutex);
    if(tornadoEnabled)
        for(int i=0;i<MAX_TORNADOS;i++)
            if(tornados[i].active) renderTornado(&tornados[i]);
    pthread_mutex_unlock(&tornadoMutex);
}

// Stdin listener thread — reads single chars, toggles tornado on 't'
static void* stdinThreadFunc(void *arg){
    (void)arg;
    // Make stdin non-blocking via select so we can exit cleanly
    while(running){
        fd_set fds; FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
        struct timeval tv = {0, 100000}; // 100 ms poll
        if(select(STDIN_FILENO+1, &fds, NULL, NULL, &tv) > 0){
            char c; if(read(STDIN_FILENO, &c, 1) == 1){
                if(c=='t' || c=='T'){
                    pthread_mutex_lock(&tornadoMutex);
                    tornadoEnabled = !tornadoEnabled;
                    if(tornadoEnabled) spawnTornados();
                    printf("[TORNADO] %s\n", tornadoEnabled ? "ON" : "OFF");
                    fflush(stdout);
                    pthread_mutex_unlock(&tornadoMutex);
                }
            }
        }
    }
    return NULL;
}

// -------- Airport Name Generator --------
void generateAirportName(char *out, int idx){
    static const char *pre[]  = {"North","South","East","West","New","Fort","Port","Lake","Mount","Cedar","Oak","Iron","Sky","River","Blue"};
    static const char *mid[]  = {"Haven","burg","field","wood","ton","vale","ford","ridge","gate","crest","view","cliff","shore","peak","moor"};
    static const char *suf[]  = {"Intl","Regional","Muni","Air","Aero","Sky","Field",""};
    // ICAO-style 4-letter code
    static const char *alpha  = "ABCDEFGHJKLMNOPRSTUVWXYZ";
    int nPre = 15, nMid = 15, nSuf = 8;
    int pi = (idx * 7 + rand()%5)  % nPre;
    int mi = (idx * 13 + rand()%7) % nMid;
    int si = (idx * 3 + rand()%4)  % nSuf;
    char icao[5];
    for(int i=0;i<4;i++) icao[i]=alpha[rand()%23];
    icao[4]='\0';
    if(suf[si][0])
        snprintf(out,32,"%s%s %s [%s]", pre[pi], mid[mi], suf[si], icao);
    else
        snprintf(out,32,"%s%s [%s]", pre[pi], mid[mi], icao);
}

// -------- Map Renderer --------
// World spans [-TERRAIN_SIZE*TERRAIN_SCALE/2 .. +TERRAIN_SIZE*TERRAIN_SCALE/2] in X and Z.
#define MAP_SIZE   560          // map panel pixel size (square)
#define MAP_MARGIN  20          // pixels from bottom-left corner
#define MAP_CELLS   64          // terrain samples across the map (downsampled from 256)

static void renderMap(){
    float worldHalf = TERRAIN_SIZE * TERRAIN_SCALE * 0.5f;  // 256
    // Map panel origin in screen space
    int mx = resWidth/2  - MAP_SIZE/2;
    int my = resHeight/2 - MAP_SIZE/2;

    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    gluOrtho2D(0,resWidth,0,resHeight);
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();

    // Semi-transparent dark background
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.05f,0.05f,0.1f,0.82f);
    glBegin(GL_QUADS);
        glVertex2i(mx,    my);
        glVertex2i(mx+MAP_SIZE, my);
        glVertex2i(mx+MAP_SIZE, my+MAP_SIZE);
        glVertex2i(mx,    my+MAP_SIZE);
    glEnd();

    // Terrain heightmap — draw as quads, MAP_CELLS×MAP_CELLS
    float cellPx = (float)MAP_SIZE / MAP_CELLS;
    for(int tz=0; tz<MAP_CELLS; tz++){
        for(int tx=0; tx<MAP_CELLS; tx++){
            // Sample world position at centre of this map cell
            float wx = -worldHalf + ((tx+0.5f)/MAP_CELLS) * worldHalf*2.0f;
            float wz = -worldHalf + ((tz+0.5f)/MAP_CELLS) * worldHalf*2.0f;
            float h  = terrainHeightAt(wx, wz);
            float t  = h / TERRAIN_HEIGHT;
            // Colour matches terrain shader: water/sand/grass/highland/rock/snow
            float r,g,b;
            if(t < 0.03f)      { r=0.10f; g=0.18f; b=0.52f; }
            else if(t < 0.07f) { r=0.70f; g=0.64f; b=0.44f; }
            else if(t < 0.35f) { r=0.15f; g=0.42f; b=0.10f; }
            else if(t < 0.60f) { r=0.20f; g=0.36f; b=0.12f; }
            else if(t < 0.78f) { r=0.44f; g=0.40f; b=0.32f; }
            else               { r=0.82f; g=0.85f; b=0.90f; }
            glColor4f(r,g,b,1.0f);
            // In OrthoY screen coords: wz maps to Y (north = up)
            float px = mx + (float)tx * cellPx;
            float py = my + (float)(MAP_CELLS-1-tz) * cellPx; // flip Z so north is up
            glBegin(GL_QUADS);
                glVertex2f(px,       py);
                glVertex2f(px+cellPx,py);
                glVertex2f(px+cellPx,py+cellPx);
                glVertex2f(px,       py+cellPx);
            glEnd();
        }
    }

    // Helper: world→map screen coords
    // wx in [-worldHalf, worldHalf] → [mx, mx+MAP_SIZE]
    // wz flipped: wz=-worldHalf → top, wz=+worldHalf → bottom, so py = my + (1 - norm)*MAP_SIZE
#define W2MX(wx) (mx + ((wx)+worldHalf)/(worldHalf*2.0f) * MAP_SIZE)
#define W2MY(wz) (my + (1.0f - ((wz)+worldHalf)/(worldHalf*2.0f)) * MAP_SIZE)

    // Airport markers
    for(int i=0;i<numAirports;i++){
        Airport *ap = &airports[i];
        float ax = W2MX(ap->wx);
        float ay = W2MY(ap->wz);
        float rl = ap->runwayLen / (worldHalf*2.0f) * MAP_SIZE;  // runway half-len in pixels

        // Runway rectangle
        float ch = cosf(ap->heading), sh = sinf(ap->heading);
        float rw = fmaxf(2.0f, ap->runwayW / (worldHalf*2.0f) * MAP_SIZE * 4.0f);
        glColor4f(0.85f,0.85f,0.85f,1.0f);
        glBegin(GL_QUADS);
            // 4 corners: ±len along heading, ±rw perpendicular
            glVertex2f(ax + ch*rl - sh*rw, ay - sh*rl - ch*rw);
            glVertex2f(ax + ch*rl + sh*rw, ay - sh*rl + ch*rw);
            glVertex2f(ax - ch*rl + sh*rw, ay + sh*rl + ch*rw);
            glVertex2f(ax - ch*rl - sh*rw, ay + sh*rl - ch*rw);
        glEnd();

        // Airport circle
        glColor4f(1.0f,0.9f,0.3f,1.0f);
        glBegin(GL_LINE_LOOP);
            for(int k=0;k<16;k++){
                float a = k*(float)(2*M_PI)/16;
                glVertex2f(ax+cosf(a)*5.0f, ay+sinf(a)*5.0f);
            }
        glEnd();

        // Name label
        glColor4f(1.0f,0.95f,0.4f,1.0f);
        glRasterPos2f(ax+7, ay+4);
        for(const char *c=airportNames[i];*c;c++)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12,*c);
    }

    // Remote players — cyan triangles
    pthread_mutex_lock(&remoteMutex);
    for(int i=0;i<MAX_PLAYERS-1;i++){
        if(!remotePlayers[i].alive) continue;
        float rx = W2MX(remotePlayers[i].x);
        float ry = W2MY(remotePlayers[i].z);
        float ya = remotePlayers[i].yaw;
        glColor4f(0.2f,1.0f,1.0f,1.0f);
        glBegin(GL_TRIANGLES);
            glVertex2f(rx + sinf(ya)*8,        ry - cosf(ya)*8);
            glVertex2f(rx + sinf(ya+2.4f)*5,   ry - cosf(ya+2.4f)*5);
            glVertex2f(rx + sinf(ya-2.4f)*5,   ry - cosf(ya-2.4f)*5);
        glEnd();
    }
    pthread_mutex_unlock(&remoteMutex);

    // Player — white arrow
    {
        float px2 = W2MX(player.x);
        float py2 = W2MY(player.z);
        float ya  = player.yaw;
        glColor4f(1,1,1,1);
        glBegin(GL_TRIANGLES);
            glVertex2f(px2 + sinf(ya)*10,        py2 - cosf(ya)*10);
            glVertex2f(px2 + sinf(ya+2.4f)*6,    py2 - cosf(ya+2.4f)*6);
            glVertex2f(px2 + sinf(ya-2.4f)*6,    py2 - cosf(ya-2.4f)*6);
        glEnd();
        // Dot at centre
        glBegin(GL_QUADS);
            glVertex2f(px2-2,py2-2); glVertex2f(px2+2,py2-2);
            glVertex2f(px2+2,py2+2); glVertex2f(px2-2,py2+2);
        glEnd();
    }

    // Border
    glColor4f(0.6f,0.7f,0.9f,1.0f);
    glBegin(GL_LINE_LOOP);
        glVertex2i(mx,my); glVertex2i(mx+MAP_SIZE,my);
        glVertex2i(mx+MAP_SIZE,my+MAP_SIZE); glVertex2i(mx,my+MAP_SIZE);
    glEnd();

    // Title
    glColor4f(0.8f,0.9f,1.0f,1.0f);
    glRasterPos2i(mx+4, my+MAP_SIZE-16);
    const char *title = "MAP  [M to close]";
    for(const char *c=title;*c;c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,*c);

    glDisable(GL_BLEND);
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);
    glEnable(GL_DEPTH_TEST);

#undef W2MX
#undef W2MY
}

// -------- Render Scene --------
void renderScene(){
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // Camera
    float camX, camY, camZ, lookX, lookY, lookZ;
    if(!inPlane){
        // First-person: eye at head height, look forward along walkerYaw
        camX = walkerX; camY = walkerY + WALKER_EYE_H; camZ = walkerZ;
        lookX = walkerX + sinf(walkerYaw);
        lookY = walkerY + WALKER_EYE_H;
        lookZ = walkerZ + cosf(walkerYaw);
    } else if(isPassenger){
        // Side window view: eye offset to the right of the plane
        float rightX = cosf(player.yaw);
        float rightZ = -sinf(player.yaw);
        float sc2 = PLANE_DEFS[player.type].scale;
        camX = player.x + rightX*sc2*0.5f;
        camY = player.y + sc2*0.2f;
        camZ = player.z + rightZ*sc2*0.5f;
        lookX = camX + sinf(player.yaw)*20.0f;
        lookY = camY;
        lookZ = camZ + cosf(player.yaw)*20.0f;
    } else {
        camX = player.x - sinf(player.yaw)*10;
        camY = player.y + 3; if(camY<1.5f) camY=1.5f;
        camZ = player.z - cosf(player.yaw)*10;
        lookX = player.x + sinf(player.yaw)*30;
        lookY = player.y;
        lookZ = player.z + cosf(player.yaw)*30;
    }
    gluLookAt(camX,camY,camZ, lookX,lookY,lookZ, 0,1,0);

    pthread_mutex_lock(&terrainMutex);
    renderTerrain();
    renderAllAirports();
    pthread_mutex_unlock(&terrainMutex);
    renderAllTornados();

    // Parked planes at gates
    for(int i=0;i<numParked;i++)
        if(parkedPlanes[i].active)
            drawPlaneModel(parkedPlanes[i].wx, parkedPlanes[i].wy, parkedPlanes[i].wz,
                           0, parkedPlanes[i].heading, 0, 1.0f, parkedPlanes[i].type);

    // Player plane (always drawn — parked while on foot)
    if(!isPassenger)
        drawPlaneModel(player.x,player.y,player.z,player.pitch,player.yaw,player.roll,gearExtension,player.type);

    // Remote players: plane or walker depending on their state
    pthread_mutex_lock(&remoteMutex);
    for(int i=0;i<MAX_PLAYERS-1;i++){
        if(!remotePlayers[i].alive) continue;
        if(remotePlayers[i].inPlane && !remotePlayers[i].isPassenger)
            drawPlaneModel(remotePlayers[i].x,remotePlayers[i].y,remotePlayers[i].z,
                           remotePlayers[i].pitch,remotePlayers[i].yaw,remotePlayers[i].roll,1.0f,
                           remotePlayers[i].type);
        else if(!remotePlayers[i].inPlane)
            drawWalkerModel(remotePlayers[i].walkerX,remotePlayers[i].walkerY,
                            remotePlayers[i].walkerZ,remotePlayers[i].walkerYaw);
    }
    pthread_mutex_unlock(&remoteMutex);

    renderExplosions();

    // HUD overlay
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); gluOrtho2D(0,resWidth,0,resHeight);
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    float aglHud = player.y - terrainHeightAt(player.x,player.z);
    char buf[128];

#define HUD(x,y,s) do{ glRasterPos2i(x,y); for(const char*_c=(s);*_c;_c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,*_c); }while(0)
#define HUD12(x,y,s) do{ glRasterPos2i(x,y); for(const char*_c=(s);*_c;_c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12,*_c); }while(0)

    // Airspeed & altitude
    glColor3f(1,1,0);
    sprintf(buf, "SPD %4.1f kts  ALT %5.1f m  THR %3.0f%%",
            airspeed*10.0f, aglHud, player.throttle*100.0f);
    HUD(10, resHeight-30, buf);

    // Status line
    static const char *wxNames[] = {"CLEAR","CLOUDY","OVERCAST","STORM"};
    glColor3f(0.7f,1.0f,0.7f);
    sprintf(buf, "%s | %s | GEAR %s | %s | WND %.1f",
            PLANE_DEFS[player.type].name,
            isPassenger ? "PASSENGER" : (onGround ? "ON GROUND" : "AIRBORNE"),
            gearDeployed ? "DOWN" : "UP",
            wxNames[weather.type],
            sqrtf(weather.windX*weather.windX + weather.windZ*weather.windZ)*10.0f);
    HUD(10, resHeight-54, buf);

    // Gear warning: airborne, gear up, low altitude
    if(inPlane && !onGround && !gearDeployed && aglHud < 40.0f){
        glColor3f(1,0.1f,0.1f);
        HUD(resWidth/2-80, resHeight/2-60, "GEAR NOT DOWN");
    }

    // On-foot / enter-exit prompts
    if(!inPlane){
        float dx = walkerX-player.x, dz = walkerZ-player.z;
        float distToPlane = sqrtf(dx*dx+dz*dz);
        glColor3f(0.6f,1.0f,0.6f);
        if(distToPlane <= ENTER_DIST)
            HUD(resWidth/2-80, resHeight/2-40, "[F] Enter aircraft");
        else {
            sprintf(buf, "On foot  |  plane %.0f m away", distToPlane);
            HUD(10, resHeight-78, buf);
        }
    } else if(onGround && airspeed < 0.5f){
        glColor3f(0.7f,0.9f,1.0f);
        HUD(resWidth/2-70, resHeight/2-40, "[F] Exit aircraft");
    }

    // Takeoff prompt when on ground and slow
    if(onGround && airspeed < TAKEOFF_SPEED){
        glColor3f(0.6f,0.9f,1.0f);
        sprintf(buf, "Hold W + throttle UP to take off  (%.0f / %.0f kts)",
                airspeed*10.0f, TAKEOFF_SPEED*10.0f);
        HUD(resWidth/2-200, resHeight/2-60, buf);
    }

    // ---- Compass tape (top-centre) ----
    {
        // Convert yaw to compass heading: yaw=0 → facing +Z (South in our coord),
        // yaw increases clockwise. We define heading as degrees from North where
        // North = -Z, so hdg = fmodf(yaw * 57.2958f + 180.0f, 360.0f)
        float hdgDeg = fmodf(player.yaw * 57.2958f + 180.0f, 360.0f);
        if(hdgDeg < 0) hdgDeg += 360.0f;

        // Heading readout box
        int tapeW = 320, tapeH = 28;
        int tapeCX = resWidth/2, tapeBY = resHeight - 2;  // bottom-y of tape
        int tapeLeft  = tapeCX - tapeW/2;
        int tapeRight = tapeCX + tapeW/2;

        // Background
        glColor4f(0.05f,0.05f,0.1f,0.75f);
        glBegin(GL_QUADS);
            glVertex2i(tapeLeft,  tapeBY - tapeH);
            glVertex2i(tapeRight, tapeBY - tapeH);
            glVertex2i(tapeRight, tapeBY);
            glVertex2i(tapeLeft,  tapeBY);
        glEnd();

        // Heading numeric above tape
        glColor4f(1,1,0.2f,1);
        sprintf(buf, "%03.0f°", hdgDeg);
        HUD(tapeCX - 18, tapeBY - tapeH - 2, buf);

        // Tape ticks: each degree = tapeW/60 px (show ±30° window)
        float pxPerDeg = (float)tapeW / 60.0f;
        // Enable scissor so ticks don't bleed outside the tape
        glEnable(GL_SCISSOR_TEST);
        glScissor(tapeLeft, tapeBY - tapeH, tapeW, tapeH);

        for(int d = -40; d <= 40; d++){
            float tickDeg = hdgDeg + d;
            if(tickDeg < 0)   tickDeg += 360.0f;
            if(tickDeg >= 360.0f) tickDeg -= 360.0f;
            float px = tapeCX + d * pxPerDeg;
            int ideg = (int)roundf(tickDeg);
            int tickTop;
            if(ideg % 30 == 0){
                tickTop = tapeBY - tapeH + 4;
                // Cardinal / intercardinal label
                const char *lbl = NULL;
                if(ideg==0||ideg==360) lbl="N";
                else if(ideg==45)  lbl="NE";
                else if(ideg==90)  lbl="E";
                else if(ideg==135) lbl="SE";
                else if(ideg==180) lbl="S";
                else if(ideg==225) lbl="SW";
                else if(ideg==270) lbl="W";
                else if(ideg==315) lbl="NW";
                if(lbl){
                    glColor4f(1,1,1,1);
                    HUD12((int)(px)-4, tapeBY - tapeH + 14, lbl);
                } else {
                    sprintf(buf,"%d",ideg);
                    glColor4f(0.7f,0.7f,0.7f,1);
                    HUD12((int)(px)-6, tapeBY - tapeH + 14, buf);
                }
            } else if(ideg % 10 == 0){
                tickTop = tapeBY - tapeH + 10;
            } else {
                continue; // skip non-10° ticks
            }
            glColor4f(0.85f,0.85f,0.85f,1);
            glBegin(GL_LINES);
                glVertex2f(px, (float)tickTop);
                glVertex2f(px, (float)(tapeBY-2));
            glEnd();
        }
        // Centre marker (triangle pointing down)
        glColor4f(1,0.8f,0,1);
        glBegin(GL_TRIANGLES);
            glVertex2f(tapeCX,       (float)(tapeBY - tapeH));
            glVertex2f(tapeCX - 5,   (float)(tapeBY - tapeH + 10));
            glVertex2f(tapeCX + 5,   (float)(tapeBY - tapeH + 10));
        glEnd();

        glDisable(GL_SCISSOR_TEST);
    }

    // ---- Airport name callouts ----
    // Project from the actual camera position/orientation used by gluLookAt.
    {
        // Camera position matches gluLookAt above — use same logic
        float camX, camY, camZ, fwdX, fwdY, fwdZ;
        if(!inPlane){
            camX = walkerX; camY = walkerY + WALKER_EYE_H; camZ = walkerZ;
            fwdX = sinf(walkerYaw); fwdY = 0.0f; fwdZ = cosf(walkerYaw);
        } else {
            camX = player.x - sinf(player.yaw)*10;
            camY = player.y + 3; if(camY<1.5f) camY=1.5f;
            camZ = player.z - cosf(player.yaw)*10;
            fwdX = sinf(player.yaw);
            fwdY = 0.0f;
            fwdZ = cosf(player.yaw);
        }
        float fwdLen = sqrtf(fwdX*fwdX + fwdY*fwdY + fwdZ*fwdZ);
        fwdX /= fwdLen; fwdY /= fwdLen; fwdZ /= fwdLen;

        // Right = normalize(fwd × worldUp)
        float rX = fwdZ, rY = 0.0f, rZ = -fwdX;
        float rLen = sqrtf(rX*rX + rZ*rZ);
        rX /= rLen; rZ /= rLen;

        // Up = right × fwd
        float uX = rY*fwdZ - rZ*fwdY;
        float uY = rZ*fwdX - rX*fwdZ;
        float uZ = rX*fwdY - rY*fwdX;

        float aspect   = (float)resWidth / resHeight;
        float vFovHalf = 30.0f * (float)M_PI / 180.0f;
        float hFovHalf = atanf(tanf(vFovHalf) * aspect);

        for(int i=0;i<numAirports;i++){
            // Vector from camera to airport
            float dx = airports[i].wx - camX;
            float dy = airports[i].groundY - camY;
            float dz = airports[i].wz - camZ;
            float dist3d = sqrtf(dx*dx + dy*dy + dz*dz);
            if(dist3d < 2.0f || dist3d > 350.0f) continue;

            // Must be in front of camera
            float dotFwd = dx*fwdX + dy*fwdY + dz*fwdZ;
            if(dotFwd <= 0.0f) continue;

            // Project onto camera right/up axes
            float dotR = dx*rX + dy*rY + dz*rZ;
            float dotU = dx*uX + dy*uY + dz*uZ;

            // Angular offsets
            float angH = atanf(dotR / dotFwd);
            float angV = atanf(dotU / dotFwd);

            // Strict FOV cull
            if(fabsf(angH) >= hFovHalf) continue;
            if(fabsf(angV) >= vFovHalf) continue;

            // Screen position
            float sx = resWidth/2  + (angH / hFovHalf) * (resWidth/2);
            float sy = resHeight/2 + (angV / vFovHalf) * (resHeight/2);

            int lblW = 150, lblH = 30;
            if(sx < lblW/2 || sx > resWidth-lblW/2) continue;
            if(sy < lblH   || sy > resHeight-lblH)  continue;

            // Distance in world-units from player (for display)
            float dist2d = sqrtf((airports[i].wx-player.x)*(airports[i].wx-player.x) +
                                 (airports[i].wz-player.z)*(airports[i].wz-player.z));
            float alpha = (dist3d > 150.0f) ? 1.0f - (dist3d-150.0f)/200.0f : 1.0f;

            // Background pill
            glColor4f(0.05f, 0.05f, 0.18f, alpha*0.78f);
            glBegin(GL_QUADS);
                glVertex2f(sx-lblW/2, sy-lblH/2); glVertex2f(sx+lblW/2, sy-lblH/2);
                glVertex2f(sx+lblW/2, sy+lblH/2); glVertex2f(sx-lblW/2, sy+lblH/2);
            glEnd();
            glColor4f(0.5f,0.7f,1.0f, alpha*0.6f);
            glBegin(GL_LINE_LOOP);
                glVertex2f(sx-lblW/2, sy-lblH/2); glVertex2f(sx+lblW/2, sy-lblH/2);
                glVertex2f(sx+lblW/2, sy+lblH/2); glVertex2f(sx-lblW/2, sy+lblH/2);
            glEnd();

            glColor4f(1.0f, 0.95f, 0.4f, alpha);
            HUD12((int)(sx-lblW/2+4), (int)(sy+4), airportNames[i]);

            sprintf(buf, "%.0f nm", dist2d * 0.054f);
            glColor4f(0.65f, 0.88f, 1.0f, alpha);
            HUD12((int)(sx-lblW/2+4), (int)(sy-12), buf);
        }
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);

    if(mapOpen) renderMap();

    renderRain();
    renderFPS();
    // Swap is handled by the caller so menus can overlay before presenting
}

// -------- Physics --------
void updatePhysics(float dt){
    pthread_mutex_lock(&playerMutex);

    if(!inPlane){
        // ---- On-foot walker ----
        if(keys[XK_a]) walkerYaw -= 1.5f*dt;
        if(keys[XK_d]) walkerYaw += 1.5f*dt;
        float moveX = 0, moveZ = 0;
        if(keys[XK_w]){ moveX += sinf(walkerYaw)*WALKER_SPEED*dt; moveZ += cosf(walkerYaw)*WALKER_SPEED*dt; }
        if(keys[XK_s]){ moveX -= sinf(walkerYaw)*WALKER_SPEED*dt; moveZ -= cosf(walkerYaw)*WALKER_SPEED*dt; }
        walkerX += moveX;
        walkerZ += moveZ;
        walkerY  = terrainHeightAt(walkerX, walkerZ) + 1.0f;

        // F key: enter own plane, a parked plane, or board a remote airliner as passenger
        if(keys[XK_f]){
            if(!enterKeyHeld){
                enterKeyHeld = true;
                float dx = walkerX - player.x, dz = walkerZ - player.z;
                if(sqrtf(dx*dx+dz*dz) <= ENTER_DIST){
                    isPassenger = false;
                    inPlane = true;
                } else {
                    // Board remote airliner as passenger
                    bool boarded = false;
                    pthread_mutex_lock(&remoteMutex);
                    for(int i=0;i<MAX_PLAYERS-1;i++){
                        if(!remotePlayers[i].alive || !remotePlayers[i].inPlane) continue;
                        if(remotePlayers[i].type != PLANE_AIRLINER) continue;
                        float rdx = walkerX - remotePlayers[i].x;
                        float rdz = walkerZ - remotePlayers[i].z;
                        if(sqrtf(rdx*rdx+rdz*rdz) <= ENTER_DIST*2.5f){
                            isPassenger = true; inPlane = true; boarded = true; break;
                        }
                    }
                    pthread_mutex_unlock(&remoteMutex);
                    if(!boarded){
                        // Enter any parked plane
                        for(int i=0;i<numParked;i++){
                            if(!parkedPlanes[i].active) continue;
                            float pdx = walkerX - parkedPlanes[i].wx;
                            float pdz = walkerZ - parkedPlanes[i].wz;
                            if(sqrtf(pdx*pdx+pdz*pdz) <= ENTER_DIST){
                                player.x     = parkedPlanes[i].wx;
                                player.y     = parkedPlanes[i].wy;
                                player.z     = parkedPlanes[i].wz;
                                player.yaw   = parkedPlanes[i].heading;
                                player.pitch = 0.0f; player.roll = 0.0f;
                                player.type  = parkedPlanes[i].type;
                                airspeed = 0.0f; onGround = true;
                                isPassenger = false;
                                parkedPlanes[i].active = false;
                                inPlane = true;
                                break;
                            }
                        }
                    }
                }
            }
        } else enterKeyHeld = false;

        pthread_mutex_unlock(&playerMutex);
        return;
    }

    // Passenger mode — track pilot's plane, F to exit (if on ground)
    if(isPassenger){
        pthread_mutex_lock(&remoteMutex);
        for(int i=0;i<MAX_PLAYERS-1;i++){
            if(!remotePlayers[i].alive || !remotePlayers[i].inPlane) continue;
            if(remotePlayers[i].type != PLANE_AIRLINER) continue;
            // Mirror pilot's position/attitude
            player.x     = remotePlayers[i].x;
            player.y     = remotePlayers[i].y;
            player.z     = remotePlayers[i].z;
            player.pitch = remotePlayers[i].pitch;
            player.yaw   = remotePlayers[i].yaw;
            player.roll  = remotePlayers[i].roll;
            player.type  = PLANE_AIRLINER;
            onGround     = (remotePlayers[i].y <= terrainHeightAt(remotePlayers[i].x,remotePlayers[i].z)+2.0f);
            break;
        }
        pthread_mutex_unlock(&remoteMutex);
        if(keys[XK_f]){
            if(!enterKeyHeld && onGround){
                enterKeyHeld = true;
                inPlane = false; isPassenger = false;
                walkerX = player.x + cosf(player.yaw)*4.0f;
                walkerZ = player.z - sinf(player.yaw)*4.0f;
                walkerY = terrainHeightAt(walkerX,walkerZ)+1.0f;
                walkerYaw = player.yaw;
            }
        } else enterKeyHeld = false;
        pthread_mutex_unlock(&playerMutex);
        return;
    }

    // Throttle
    if(keys[XK_Up])   player.throttle += 0.5f*dt;
    if(keys[XK_Down]) player.throttle -= 0.5f*dt;
    player.throttle = clampf(player.throttle, 0.0f, 1.0f);

    // G key: toggle gear (debounced, blocked on ground)
    if(keys[XK_g]){ if(!gearKeyHeld && !onGround){ gearDeployed=!gearDeployed; gearKeyHeld=true; } }
    else gearKeyHeld=false;
    // Auto-deploy gear on touchdown approach (below 10 units AGL)
    float groundBelow = terrainHeightAt(player.x, player.z);
    float agl = player.y - groundBelow;
    if(agl < 10.0f && !onGround && !gearDeployed) gearDeployed = true;
    // Animate gear at 2 units/sec
    float gearTarget = gearDeployed ? 1.0f : 0.0f;
    float gearStep   = 2.0f * dt;
    if(gearExtension < gearTarget) gearExtension = clampf(gearExtension+gearStep,0.0f,1.0f);
    else if(gearExtension > gearTarget) gearExtension = clampf(gearExtension-gearStep,0.0f,1.0f);

    float groundY = groundBelow + 1.0f;  // wheel contact height

    // F key: exit plane when stopped on ground
    if(keys[XK_f]){
        if(!enterKeyHeld && onGround && airspeed < 0.5f){
            enterKeyHeld = true;
            inPlane  = false;
            // Spawn walker beside the plane (to the right)
            walkerX   = player.x + cosf(player.yaw)*3.0f;
            walkerZ   = player.z - sinf(player.yaw)*3.0f;
            walkerY   = groundBelow + 1.0f;
            walkerYaw = player.yaw;
        }
    } else enterKeyHeld = false;

    if(onGround){
        // ---- Ground roll ----
        // A/D = rudder (yaw)
        if(keys[XK_a]) player.yaw += 0.8f*dt;
        if(keys[XK_d]) player.yaw -= 0.8f*dt;
        player.pitch = 0.0f;
        player.roll  = 0.0f;

        // Accelerate / brake
        float targetSpeed = player.throttle * MAX_SPEED;
        if(airspeed < targetSpeed)
            airspeed += PLANE_DEFS[player.type].accel * dt;
        else
            airspeed -= BRAKE_DECEL * dt;
        airspeed = clampf(airspeed, 0.0f, MAX_SPEED);

        player.vx = sinf(player.yaw) * airspeed;
        player.vy = 0.0f;
        player.vz = cosf(player.yaw) * airspeed;

        player.x += player.vx * dt;
        player.y  = groundY;
        player.z += player.vz * dt;

        // Lift-off when fast enough and nose up (W held)
        if(airspeed >= TAKEOFF_SPEED && keys[XK_w]){
            onGround = false;
            player.pitch =  0.15f;  // gentle nose-up attitude at lift-off
            player.vy    =  airspeed * sinf(player.pitch);
        }
    } else {
        // ---- In flight ----
        if(keys[XK_w]) player.pitch += 1.2f*dt;
        if(keys[XK_s]) player.pitch -= 1.2f*dt;
        if(keys[XK_a]) player.roll  -= 1.0f*dt;
        if(keys[XK_d]) player.roll  += 1.0f*dt;
        player.pitch = clampf(player.pitch, -1.2f, 1.2f);
        player.roll  = clampf(player.roll,  -1.4f, 1.4f);

        // Update turbulence intensity: slowly drift toward a random target
        turbulenceTimer -= dt;
        if(turbulenceTimer <= 0.0f){
            turbulenceTarget = ((float)rand()/RAND_MAX) * ((float)rand()/RAND_MAX); // bias toward calm
            turbulenceTimer  = 4.0f + ((float)rand()/RAND_MAX) * 8.0f; // change every 4–12 s
        }
        turbulenceLevel += (turbulenceTarget - turbulenceLevel) * dt * 0.4f;

        float turb = turbulenceLevel * TURBULENCE_MAX;
        player.pitch += ((float)rand()/RAND_MAX-0.5f)*turb;
        player.roll  += ((float)rand()/RAND_MAX-0.5f)*turb;
        player.yaw   += ((float)rand()/RAND_MAX-0.5f)*turb*0.5f;
        player.yaw   -= sinf(player.roll)*dt;

        float thrustSpeed = player.throttle * MAX_SPEED;
        // Converge current airspeed toward throttle target
        float accel = (thrustSpeed > airspeed) ? PLANE_DEFS[player.type].accel : -2.0f;
        airspeed += accel * dt;
        airspeed  = clampf(airspeed, 0.0f, MAX_SPEED);

        // Extra sink below stall speed
        float stallSink = 0.0f;
        if(airspeed < STALL_SPEED)
            stallSink = (STALL_SPEED - airspeed) * 2.0f;

        player.vx =  sinf(player.yaw)*cosf(player.pitch)*airspeed;
        player.vy =  sinf(player.pitch)*airspeed - GRAVITY*dt - stallSink;
        player.vz =  cosf(player.yaw)*cosf(player.pitch)*airspeed;

        player.x += (player.vx + weather.windX)*dt;
        player.y +=  player.vy*dt;
        player.z += (player.vz + weather.windZ)*dt;

        // Touchdown
        if(player.y <= groundY){
            float impactVy = player.vy;  // negative = downward
            player.y  = groundY;
            player.vy = 0.0f;

            bool hardCrash = (gearExtension < 0.9f && airspeed > 1.5f)   // gear-up belly landing
                          || (impactVy < -8.0f);                          // too-hard vertical impact

            if(hardCrash){
                // Big crash explosion — multiple blasts for drama
                triggerExplosionEx(player.x,        player.y+1.0f,  player.z,        3.5f);
                triggerExplosionEx(player.x+2.0f,   player.y+0.5f,  player.z+1.5f,   2.5f);
                triggerExplosionEx(player.x-1.5f,   player.y+0.8f,  player.z-2.0f,   2.0f);
                airspeed  = 0.0f;
                player.vx = player.vy = player.vz = 0.0f;
                onGround  = true;
                gearDeployed = true;
                player.pitch = 0.0f;
                player.roll  = 0.0f;
            } else if(gearExtension > 0.9f){
                // Normal landing
                onGround  = true;
                airspeed  = sqrtf(player.vx*player.vx + player.vz*player.vz);
                player.pitch = 0.0f;
                player.roll  = 0.0f;
            } else {
                // Gear-up, slow speed (e.g. stall onto runway) — no explosion, just stop
                airspeed  = 0.0f;
                player.vx = player.vy = player.vz = 0.0f;
                onGround  = true;
                gearDeployed = true;
            }
        }
    }

    pthread_mutex_unlock(&playerMutex);
}

// -------- Fullscreen toggle via EWMH --------
static void applyFullscreen(){
    Atom wmState     = XInternAtom(dpy, "_NET_WM_STATE", False);
    Atom wmFullscreen= XInternAtom(dpy, "_NET_WM_STATE_FULLSCREEN", False);
    XEvent xev = {0};
    xev.type                 = ClientMessage;
    xev.xclient.window       = win;
    xev.xclient.message_type = wmState;
    xev.xclient.format       = 32;
    xev.xclient.data.l[0]    = fullscreen ? 1 : 0;  // 1=add, 0=remove
    xev.xclient.data.l[1]    = (long)wmFullscreen;
    xev.xclient.data.l[2]    = 0;
    XSendEvent(dpy, DefaultRootWindow(dpy), False,
               SubstructureNotifyMask|SubstructureRedirectMask, &xev);
    XFlush(dpy);
}

// -------- Menu Rendering --------

// Draw a filled rectangle (no depth test)
static void menuRect(float x, float y, float w, float h, float r, float g, float b, float a){
    glColor4f(r,g,b,a);
    glBegin(GL_QUADS);
    glVertex2f(x,y); glVertex2f(x+w,y); glVertex2f(x+w,y+h); glVertex2f(x,y+h);
    glEnd();
}
static void menuRectOutline(float x,float y,float w,float h,float r,float g,float b){
    glColor3f(r,g,b);
    glBegin(GL_LINE_LOOP);
    glVertex2f(x,y); glVertex2f(x+w,y); glVertex2f(x+w,y+h); glVertex2f(x,y+h);
    glEnd();
}
static void menuText(float x, float y, const char *s, void *font){
    glRasterPos2f(x,y);
    for(const char *c=s;*c;c++) glutBitmapCharacter(font,*c);
}

// Returns 1 if (mx,my) is inside the rect (OpenGL coords: y=0 at bottom)
static int inRect(int mx, int my, float rx, float ry, float rw, float rh){
    return mx>=rx && mx<=rx+rw && my>=ry && my<=ry+rh;
}

// Start 2D ortho mode
static void begin2D(){
    glDisable(GL_DEPTH_TEST); glDisable(GL_LIGHTING);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    gluOrtho2D(0,resWidth,0,resHeight);
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
}
static void end2D(){
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);
    glDisable(GL_BLEND); glEnable(GL_DEPTH_TEST); glEnable(GL_LIGHTING);
}

// Button dimensions
#define BTN_W   260
#define BTN_H    38
#define SML_W    44   // small arrow / +/- button width
#define SML_H    32
#define BTN_X   ((resWidth-BTN_W)/2)
#define LBL_X   (BTN_X)
#define CTL_X   (BTN_X+140)  // right column start for settings controls

// Draw a full-width button centred on screen, returns 1 if hovered
static int menuButton(float bx, float by, const char *label, int mx, int my){
    int hover = inRect(mx,my,bx,by,BTN_W,BTN_H);
    menuRect(bx,by,BTN_W,BTN_H, hover?0.28f:0.14f, hover?0.48f:0.28f, hover?0.72f:0.50f, 0.93f);
    menuRectOutline(bx,by,BTN_W,BTN_H, 0.45f,0.70f,1.0f);
    glColor3f(1,1,1);
    int tw=0; for(const char *c=label;*c;c++) tw+=glutBitmapWidth(GLUT_BITMAP_HELVETICA_18,*c);
    menuText(bx+(BTN_W-tw)*0.5f, by+12, label, GLUT_BITMAP_HELVETICA_18);
    return hover;
}

// Small fixed-width button (arrow / +/-), returns 1 if hovered
static int menuSmlButton(float bx, float by, const char *label, int mx, int my){
    int hover = inRect(mx,my,bx,by,SML_W,SML_H);
    menuRect(bx,by,SML_W,SML_H, hover?0.28f:0.14f, hover?0.48f:0.28f, hover?0.72f:0.50f, 0.93f);
    menuRectOutline(bx,by,SML_W,SML_H, 0.45f,0.70f,1.0f);
    glColor3f(1,1,1);
    int tw=0; for(const char *c=label;*c;c++) tw+=glutBitmapWidth(GLUT_BITMAP_HELVETICA_18,*c);
    menuText(bx+(SML_W-tw)*0.5f, by+9, label, GLUT_BITMAP_HELVETICA_18);
    return hover;
}

// Checkbox helper
static void menuCheckbox(float cx, float cy, bool checked, const char *label, int mx, int my){
    int hover = inRect(mx,my,cx,cy,20,20);
    menuRect(cx,cy,20,20, hover?0.25f:0.12f, hover?0.45f:0.25f, hover?0.68f:0.45f, 0.93f);
    menuRectOutline(cx,cy,20,20, 0.45f,0.70f,1.0f);
    if(checked){
        glColor3f(0.2f,1.0f,0.4f);
        menuText(cx+4, cy+5, "X", GLUT_BITMAP_HELVETICA_18);
    }
    glColor3f(1,1,1);
    menuText(cx+28, cy+4, label, GLUT_BITMAP_HELVETICA_18);
}

// Horizontal divider
static void menuDivider(float y){
    glColor4f(0.3f,0.5f,0.7f,0.6f);
    glBegin(GL_LINES);
    glVertex2f((float)BTN_X, y); glVertex2f((float)(BTN_X+BTN_W), y);
    glEnd();
}

// Persistent hover tracking (updated each frame via mouse motion)
static int menuMouseX = 0, menuMouseY = 0;

void renderMenu(){
    begin2D();

    // Background
    menuRect(0,0,(float)resWidth,(float)resHeight, 0.04f,0.07f,0.13f,1.0f);

    // Title
    const char *title = "FLIGHT SIMULATOR";
    int tw=0; for(const char *c=title;*c;c++) tw+=glutBitmapWidth(GLUT_BITMAP_TIMES_ROMAN_24,*c);
    glColor3f(0.55f,0.82f,1.0f);
    menuText((resWidth-tw)/2.0f, resHeight-70, title, GLUT_BITMAP_TIMES_ROMAN_24);

    menuDivider(resHeight-85);

    int mx=menuMouseX, my=menuMouseY;

    if(menuState==MENU_MAIN){
        // Buttons stacked from centre
        float cy = resHeight/2.0f + 100;
        float gap = 48;
        menuButton(BTN_X, cy,         "Single Player", mx,my);
        menuButton(BTN_X, cy-gap,     "Host Server",   mx,my);
        menuButton(BTN_X, cy-gap*2,   "Join Server",   mx,my);
        menuButton(BTN_X, cy-gap*3,   "Settings",      mx,my);
        menuButton(BTN_X, cy-gap*4,   "Quit",          mx,my);

        menuDivider(cy - gap*4 - 16);

        // Tornado checkbox beneath buttons
        float cbY = cy - gap*4 - 46;
        menuCheckbox(BTN_X, cbY, tornadoEnabled, "Tornado Mode  (single player / host only)", mx,my);

    } else if(menuState==MENU_SETTINGS){
        // Panel centred vertically
        float top = resHeight*0.80f;

        // Section heading
        glColor3f(0.6f,0.85f,1.0f);
        int htw=0; const char *ht="Settings";
        for(const char *c=ht;*c;c++) htw+=glutBitmapWidth(GLUT_BITMAP_TIMES_ROMAN_24,*c);
        menuText((resWidth-htw)/2.0f, top, ht, GLUT_BITMAP_TIMES_ROMAN_24);
        menuDivider(top-14);

        float row = top - 40;
        float rowGap = 44;

        // ── Fullscreen ─────────────────────────────────────
        glColor3f(0.85f,0.85f,0.85f);
        menuText(LBL_X, row+4, "Fullscreen:", GLUT_BITMAP_HELVETICA_18);
        menuCheckbox(CTL_X, row, fullscreen, "", mx,my);
        row -= rowGap;

        // ── Resolution ─────────────────────────────────────
        glColor3f(0.85f,0.85f,0.85f);
        menuText(LBL_X, row+8, "Resolution:", GLUT_BITMAP_HELVETICA_18);

        char resBuf[32];
        snprintf(resBuf,sizeof(resBuf),"%dx%d",resOptions[resIndex][0],resOptions[resIndex][1]);
        float arrowLeftX  = CTL_X;
        float valX        = CTL_X + SML_W + 6;
        float arrowRightX = valX + 90;
        menuSmlButton(arrowLeftX,  row, "<", mx,my);
        glColor3f(1,1,1);
        menuText(valX, row+8, resBuf, GLUT_BITMAP_HELVETICA_18);
        menuSmlButton(arrowRightX, row, ">", mx,my);
        row -= rowGap;

        // ── FPS Cap ────────────────────────────────────────
        glColor3f(0.85f,0.85f,0.85f);
        menuText(LBL_X, row+8, "FPS Cap:", GLUT_BITMAP_HELVETICA_18);

        char _fpstmp[16];
        if(fpsCap>0) snprintf(_fpstmp,sizeof(_fpstmp),"%d",fpsCap);
        const char *fpsStr = fpsCap==0 ? "Uncapped" : _fpstmp;
        float fpsMinusX = CTL_X;
        float fpsPlusX  = CTL_X + SML_W + 52;
        menuSmlButton(fpsMinusX, row, "-", mx,my);
        glColor3f(1,1,1);
        int fw=0; for(const char *c=fpsStr;*c;c++) fw+=glutBitmapWidth(GLUT_BITMAP_HELVETICA_18,*c);
        menuText(CTL_X + SML_W + 6 + (46-fw)/2.0f, row+8, fpsStr, GLUT_BITMAP_HELVETICA_18);
        menuSmlButton(fpsPlusX, row, "+", mx,my);
        row -= rowGap + 8;

        menuDivider(row + rowGap - 4);

        // ── Back ───────────────────────────────────────────
        menuButton(BTN_X, row - 14, "Back", mx,my);

    } else if(menuState==MENU_JOIN){
        float cy = resHeight/2.0f + 60;

        glColor3f(0.6f,0.85f,1.0f);
        int htw=0; const char *ht="Join Server";
        for(const char *c=ht;*c;c++) htw+=glutBitmapWidth(GLUT_BITMAP_TIMES_ROMAN_24,*c);
        menuText((resWidth-htw)/2.0f, cy+80, ht, GLUT_BITMAP_TIMES_ROMAN_24);
        menuDivider(cy+66);

        // IP label + input box on same row
        glColor3f(0.85f,0.85f,0.85f);
        menuText(LBL_X, cy+26, "Server IP:", GLUT_BITMAP_HELVETICA_18);

        float ibx=CTL_X, iby=cy+12;
        float ibw=160, ibh=28;
        menuRect(ibx,iby,ibw,ibh,
            joinIPFocus?0.12f:0.08f, joinIPFocus?0.22f:0.12f, joinIPFocus?0.38f:0.22f, 0.96f);
        menuRectOutline(ibx,iby,ibw,ibh,
            joinIPFocus?0.5f:0.32f, joinIPFocus?0.78f:0.55f, joinIPFocus?1.0f:0.75f);
        glColor3f(1,1,1);
        menuText(ibx+6, iby+7, joinIP, GLUT_BITMAP_HELVETICA_18);
        // cursor blink
        if(joinIPFocus){
            int cw=0; for(const char *c=joinIP;*c;c++) cw+=glutBitmapWidth(GLUT_BITMAP_HELVETICA_18,*c);
            glColor3f(0.7f,0.9f,1.0f);
            glBegin(GL_LINES);
            glVertex2f(ibx+6+cw, iby+4); glVertex2f(ibx+6+cw, iby+24);
            glEnd();
        }

        // Error message
        if(joinFailed){
            const char *err = "No server found. Check IP and try again.";
            int ew=0; for(const char *c=err;*c;c++) ew+=glutBitmapWidth(GLUT_BITMAP_HELVETICA_18,*c);
            glColor3f(1.0f,0.25f,0.25f);
            menuText((resWidth-ew)/2.0f, cy-18, err, GLUT_BITMAP_HELVETICA_18);
        }

        menuButton(BTN_X, cy-66,  "Connect", mx,my);
        menuButton(BTN_X, cy-116, "Back",    mx,my);
    }

    end2D();
}

// Called on mouse click (mx/my in X11 coords — y=0 at top; convert to GL coords)
void handleMenuClick(int mx, int my_x11){
    int my = resHeight - my_x11;

    if(menuState==MENU_MAIN){
        float cy = resHeight/2.0f + 100;
        float gap = 48;
        if(inRect(mx,my, BTN_X,cy,        BTN_W,BTN_H)){ isServer=true;  menuState=MENU_PLAYING; } // Single Player
        else if(inRect(mx,my, BTN_X,cy-gap,   BTN_W,BTN_H)){ isServer=true;  menuState=MENU_PLAYING; } // Host Server
        else if(inRect(mx,my, BTN_X,cy-gap*2, BTN_W,BTN_H)){ menuState=MENU_JOIN; joinIPFocus=true; joinFailed=false; } // Join
        else if(inRect(mx,my, BTN_X,cy-gap*3, BTN_W,BTN_H)){ menuState=MENU_SETTINGS; } // Settings
        else if(inRect(mx,my, BTN_X,cy-gap*4, BTN_W,BTN_H)){ running=false; } // Quit
        // Tornado checkbox
        float cbY = cy - gap*4 - 46;
        if(inRect(mx,my, BTN_X,cbY, 20,20)){
            pthread_mutex_lock(&tornadoMutex);
            tornadoEnabled = !tornadoEnabled;
            if(tornadoEnabled && gameStarted && isServer){
                spawnTornados();
            } else if(!tornadoEnabled){
                for(int i=0;i<MAX_TORNADOS;i++) tornados[i].active=false;
            }
            pthread_mutex_unlock(&tornadoMutex);
        }

    } else if(menuState==MENU_SETTINGS){
        float top  = resHeight*0.80f;
        float row  = top - 40;
        float rowG = 44;

        // Fullscreen checkbox
        if(inRect(mx,my, CTL_X,row, 20,20)){ fullscreen = !fullscreen; applyFullscreen(); }
        row -= rowG;

        // Resolution < >
        float arrowLeftX  = CTL_X;
        float arrowRightX = CTL_X + SML_W + 6 + 90;
        if(inRect(mx,my, arrowLeftX,row, SML_W,SML_H)){
            if(resIndex>0) resIndex--;
            resWidth=resOptions[resIndex][0]; resHeight=resOptions[resIndex][1];
            XResizeWindow(dpy,win,resWidth,resHeight);
            glViewport(0,0,resWidth,resHeight);
            glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluPerspective(60,(double)resWidth/resHeight,0.5,1200.0);
            glMatrixMode(GL_MODELVIEW);
        }
        if(inRect(mx,my, arrowRightX,row, SML_W,SML_H)){
            if(resIndex<NUM_RES_OPTIONS-1) resIndex++;
            resWidth=resOptions[resIndex][0]; resHeight=resOptions[resIndex][1];
            XResizeWindow(dpy,win,resWidth,resHeight);
            glViewport(0,0,resWidth,resHeight);
            glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluPerspective(60,(double)resWidth/resHeight,0.5,1200.0);
            glMatrixMode(GL_MODELVIEW);
        }
        row -= rowG;

        // FPS cap - +
        float fpsMinusX = CTL_X;
        float fpsPlusX  = CTL_X + SML_W + 52;
        if(inRect(mx,my, fpsMinusX,row, SML_W,SML_H)){ if(fpsCap>5) fpsCap-=5; else fpsCap=0; }
        if(inRect(mx,my, fpsPlusX, row, SML_W,SML_H)){ if(fpsCap==0) fpsCap=5; else if(fpsCap<240) fpsCap+=5; }
        row -= rowG + 8;

        // Back
        if(inRect(mx,my, BTN_X, row-14, BTN_W,BTN_H)) menuState=MENU_MAIN;

    } else if(menuState==MENU_JOIN){
        float cy  = resHeight/2.0f + 60;
        float ibx = CTL_X, iby = cy+12;
        joinIPFocus = inRect(mx,my, ibx,iby, 160,28);
        if(inRect(mx,my, BTN_X,cy-66,  BTN_W,BTN_H)){
            // Attempt to connect
            isServer=false; joinFailed=false; joinTimer=0.0f;
            menuState=MENU_PLAYING;
        }
        if(inRect(mx,my, BTN_X,cy-116, BTN_W,BTN_H)) menuState=MENU_MAIN;
    }
}

// -------- Input --------
void handleInput(XEvent *e){
    // Window resize / fullscreen — keep resWidth/resHeight in sync
    if(e->type==ConfigureNotify){
        int nw = e->xconfigure.width, nh = e->xconfigure.height;
        if(nw>0 && nh>0 && (nw!=resWidth || nh!=resHeight)){
            resWidth=nw; resHeight=nh;
            glViewport(0,0,resWidth,resHeight);
            glMatrixMode(GL_PROJECTION); glLoadIdentity();
            gluPerspective(60,(double)resWidth/resHeight,0.5,1200.0);
            glMatrixMode(GL_MODELVIEW);
        }
    }
    // Track mouse position for hover effects (X11 y=0 at top, convert to GL coords)
    if(e->type==MotionNotify){
        menuMouseX = e->xmotion.x;
        menuMouseY = resHeight - e->xmotion.y;
    }
    if(e->type==ButtonPress && e->xbutton.button==Button1){
        if(menuState!=MENU_PLAYING){
            handleMenuClick(e->xbutton.x, e->xbutton.y);
        }
    }
    if(e->type==KeyPress){
        KeySym k=XLookupKeysym(&e->xkey,0);
        // IP text input when in JOIN menu
        if(menuState==MENU_JOIN && joinIPFocus){
            if(k==XK_BackSpace){
                if(joinIPLen>0){ joinIPLen--; joinIP[joinIPLen]='\0'; }
            } else if(k==XK_Escape){
                menuState=MENU_MAIN;
            } else if(k==XK_Return){
                isServer=false; menuState=MENU_PLAYING;
            } else {
                char buf[8]; int n=XLookupString(&e->xkey,buf,sizeof(buf),NULL,NULL);
                if(n==1 && joinIPLen<(int)sizeof(joinIP)-2){
                    joinIP[joinIPLen++]=buf[0]; joinIP[joinIPLen]='\0';
                }
            }
            return;
        }
        if(k<65536) keys[k]=true;
        if(k==XK_Escape){
            if(menuState==MENU_PLAYING) menuState=MENU_MAIN;
            else running=false;
        }
        if(k==XK_m || k==XK_M){ if(!mapKeyHeld){ mapOpen=!mapOpen; mapKeyHeld=true; } }
    }
    if(e->type==KeyRelease){
        KeySym k=XLookupKeysym(&e->xkey,0);
        if(k<65536) keys[k]=false;
        if(k==XK_m || k==XK_M) mapKeyHeld=false;
    }
}

// forward decl
void startGame();

// -------- Game Loop --------
void gameLoop(){
    XEvent e;
    while(running){
        while(XPending(dpy)){ XNextEvent(dpy,&e); handleInput(&e); }

        if(menuState != MENU_PLAYING){
            if(gameStarted){
                // Paused: draw frozen world, then overlay menu, then present
                renderScene();
            } else {
                glClearColor(0.04f,0.07f,0.13f,1.0f);
                glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
            }
            renderMenu();   // draws on top of whatever is in the back buffer
            glXSwapBuffers(dpy,win);
            usleep(16000);
            continue;
        }

        // First frame after switching to MENU_PLAYING — start the game engine
        if(!gameStarted){
            gameStarted = true;
            startGame();
            // Spawn tornadoes if enabled (single player or server)
            if(tornadoEnabled && isServer){
                pthread_mutex_lock(&tornadoMutex);
                spawnTornados();
                pthread_mutex_unlock(&tornadoMutex);
            }
            glClearColor(0.5f,0.8f,1.0f,1.0f);
        }

        // Client join timeout — if seed not received after JOIN_TIMEOUT seconds, go back
        if(!isServer && !seedReceived){
            joinTimer += DT;
            if(joinTimer >= JOIN_TIMEOUT){
                // Connection failed — return to join menu with error message
                joinFailed   = true;
                menuState    = MENU_JOIN;
                gameStarted  = false;
                seedReceived = false;
                joinTimer    = 0.0f;
                // Close socket so it can be re-bound on next attempt
                close(sockfd); sockfd = -1;
                continue;
            }
        }

        updateWeather(DT);
        updateTornados(DT);
        updateExplosions(DT);
        updatePhysics(DT);
        renderScene();
        glXSwapBuffers(dpy,win);
        if(fpsCap > 0) usleep(1000000/fpsCap);
    }
}

// -------- Init X11/GL --------
void initX11GL(){
    dpy = XOpenDisplay(NULL);
    if(!dpy){ fprintf(stderr,"Cannot open display\n"); exit(1);}
    Window root = DefaultRootWindow(dpy);
    XVisualInfo *vi = glXChooseVisual(dpy,0,(int[]){GLX_RGBA,GLX_DEPTH_SIZE,24,GLX_DOUBLEBUFFER,None});
    Colormap cmap = XCreateColormap(dpy, root, vi->visual, AllocNone);
    XSetWindowAttributes swa; swa.colormap=cmap; swa.event_mask=ExposureMask|KeyPressMask|KeyReleaseMask|ButtonPressMask|PointerMotionMask|StructureNotifyMask;
    win = XCreateWindow(dpy, root,0,0,WIDTH,HEIGHT,0,vi->depth,InputOutput,vi->visual,CWColormap|CWEventMask,&swa);
    XMapWindow(dpy,win);
    glc = glXCreateContext(dpy,vi,NULL,GL_TRUE);
    glXMakeCurrent(dpy,win,glc);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_TEXTURE_2D);
    glShadeModel(GL_SMOOTH);

    // Sun direction (slightly from above-front)
    GLfloat lightPos[]  = { 0.4f, 1.0f, 0.6f, 0.0f };  // directional
    GLfloat ambient[]   = { 0.25f,0.25f,0.30f,1.0f };
    GLfloat diffuse[]   = { 0.85f,0.82f,0.75f,1.0f };
    GLfloat specular[]  = { 0.6f, 0.6f, 0.6f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);

    // Material specular response
    GLfloat matSpec[] = { 0.5f,0.5f,0.5f,1.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  matSpec);
    glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 48.0f);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    glClearColor(0.5f,0.8f,1.0f,1.0f);
    generateCheckerboardTexture();
}

// -------- Network Init --------
// serverPort: port the server listens on
// clientPort: port the client binds to locally (must differ from serverPort on same machine)
void initNetwork(const char* serverIp, int serverPort, int clientPort){
    sockfd = socket(AF_INET, SOCK_DGRAM,0);
    if(sockfd<0){ perror("socket"); exit(1);}

    // Both sides bind to their own listen port
    struct sockaddr_in bindAddr={0};
    bindAddr.sin_family=AF_INET;
    bindAddr.sin_addr.s_addr=INADDR_ANY;
    bindAddr.sin_port = htons(isServer ? serverPort : clientPort);
    if(bind(sockfd,(struct sockaddr*)&bindAddr,sizeof(bindAddr))<0){ perror("bind"); exit(1); }
    printf("[NET] %s bound to port %d | target server: %s:%d\n",
        isServer ? "Server" : "Client",
        isServer ? serverPort : clientPort,
        serverIp, serverPort);

    // Client pre-fills server address; server waits to learn client address on first MSG_CONNECT
    if(!isServer){
        otherAddr.sin_family=AF_INET;
        otherAddr.sin_port=htons(serverPort);
        inet_pton(AF_INET,serverIp,&otherAddr.sin_addr);
    }

    fcntl(sockfd,F_SETFL,O_NONBLOCK);

    // Client will announce itself repeatedly in the network thread until seed is received
}

// -------- Start Game (called once menu selection is made) --------
static pthread_t netThread;
void startGame(){
    // Single-player: no networking needed, just generate terrain
    bool doNetwork = (menuState == MENU_PLAYING && (isServer || !isServer));
    // isServer was already set by the menu click handler

    if(isServer){
        terrainSeed = (unsigned int)time(NULL);
        generateAllTerrain(terrainSeed);
        printf("[NET] Server terrain seed: %u\n", terrainSeed);
    } else {
        terrainSeed = 0;
        generateAllTerrain(terrainSeed);
        printf("[NET] Client: waiting for terrain seed from server...\n");
    }

    player.x=0; player.y=airports[0].groundY+1.0f; player.z=0;
    player.pitch=0; player.yaw=0; player.roll=0; player.throttle=0.0f; player.type=PLANE_FIGHTER;
    onGround=true; airspeed=0.0f;
    for(int i=0;i<MAX_ENEMIES;i++){ enemies[i].x=(rand()%20-10); enemies[i].y=5; enemies[i].z=(rand()%20-10); enemies[i].alive=true; }

    initWeather();
    initNetwork(joinIP, 5000, 5001);

    if(pthread_create(&netThread,NULL,networkThreadFunc,NULL)!=0){ perror("pthread_create"); exit(1); }
    pthread_t stdinThread; pthread_create(&stdinThread, NULL, stdinThreadFunc, NULL);
    (void)doNetwork;
}

// -------- Main --------
int main(){
    int argc=1; char *argv[1]={(char*)"game"}; glutInit(&argc,argv);
    srand(time(NULL)); initX11GL();

    // Viewport & projection (use initial resWidth/resHeight)
    glViewport(0,0,resWidth,resHeight);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluPerspective(60,(double)resWidth/resHeight,0.5,1200.0);
    glMatrixMode(GL_MODELVIEW);

    gameLoop();

    running=false;
    pthread_join(netThread,NULL);

    glXMakeCurrent(dpy,0,0); glXDestroyContext(dpy,glc); XDestroyWindow(dpy,win); XCloseDisplay(dpy);
    return 0;
}