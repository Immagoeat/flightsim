// flight_sim_multiplayer_logging.c
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>
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
#include <sys/wait.h>
#include <signal.h>

#ifndef M_PI
#define M_PI 3.14159265f
#endif
#define WIDTH 1024
#define HEIGHT 768
#define DT 0.016f
#define MAX_PLAYERS 2
#define TEX_SIZE 64
#define MSG_PLAYER_DATA  0
#define MSG_CONNECT      1
#define MSG_TERRAIN_SEED 2
#define MSG_TORNADO      3
#define MSG_EXPLOSION    4
#define MSG_CHAT         5
#define MSG_VOICE        6
// Turbulence is now dynamic — see turbulenceLevel global
#define TURBULENCE_MAX 0.004f   // peak per-frame angular kick (was 0.01)
#define GRAVITY 9.8f
#define TERRAIN_SIZE 512       // grid cells per side (must be power-of-2)
#define TERRAIN_SCALE 10.0f    // world units per cell => 5120x5120 world
#define TERRAIN_HEIGHT 280.0f  // max terrain height — towering peaks
#define MAX_AIRPORTS 24

typedef enum { PLANE_FIGHTER=0, PLANE_PROP=1, PLANE_AIRLINER=2 } PlaneType;
typedef struct {
    float maxSpeed, takeoffSpeed, stallSpeed, brakeDecel, accel;
    float scale;    // visual scale multiplier
    const char *name;
} PlaneStats;
static const PlaneStats PLANE_DEFS[3] = {
    /* FIGHTER  */ { 80.0f, 18.0f, 8.0f,  10.0f, 28.0f, 1.0f, "F/A-18 Hornet"   },
    /* PROP     */ { 35.0f,  9.0f, 4.5f,   7.0f, 14.0f, 0.7f, "Cessna 172"      },
    /* AIRLINER */ { 60.0f, 22.0f, 12.0f,  8.0f, 12.0f, 2.2f, "Boeing 737"      },
};

typedef struct { float x,y,z; float vx,vy,vz; float pitch,yaw,roll; float throttle; PlaneType type; } Plane;
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
#define MAX_PARKED 32
#define PLANE_RESPAWN_TIME 60.0f   // seconds before an empty slot spawns a new plane
typedef struct {
    float wx,wy,wz, heading;
    bool  active;
    PlaneType type;
    float respawnTimer;  // counts down; when <=0 and !active, spawn a new plane
} ParkedPlane;
ParkedPlane parkedPlanes[MAX_PARKED];
int numParked = 0;

// NPCs
#define MAX_NPCS 32
typedef enum {
    NPC_WALK=0,   // wandering airport apron on foot
    NPC_BOARD,    // walking toward a plane to board
    NPC_TAKEOFF,  // rolling down runway, climbing out
    NPC_FLY,      // flying a circuit
    NPC_APPROACH, // descending to land
    NPC_LAND,     // on runway rolling to stop
    NPC_DEPLANE   // walking away from plane after landing
} NpcState;
typedef struct {
    float x,y,z,yaw;          // on-foot position/heading
    float px,py,pz;            // plane position when flying
    float ppitch,pyaw,proll;   // plane orientation when flying
    float pspeed;              // plane airspeed when flying
    float pthrottle;
    PlaneType ptype;
    NpcState  state;
    float     stateTimer;      // countdown for current state
    float     wanderTimer;     // time until next wander direction
    float     wanderYaw;       // current wander heading
    int       airportIdx;      // home airport index
    float     homeX,homeZ;     // apron spawn position
    bool      alive;
    // circuit waypoint when flying
    float     wpX,wpZ,wpAlt;
    int       wpIdx;
} Npc;
Npc npcs[MAX_NPCS];
int numNpcs = 0;

Plane player;
RemotePlane remotePlayers[MAX_PLAYERS-1];

Display *dpy;
Window win;
GLXContext glc;
bool running=true;
bool keys[65536];
GLuint groundTexture;
// Mouse look
static bool  mouseCaptured = false;
static float mouseSensX    = 0.0015f;
static float mouseSensY    = 0.0012f;
static int   mouseWarpX, mouseWarpY; // window centre, set at game start
// Per-biome detail textures:
// 0=water 1=sand/desert 2=grass 3=rock 4=snow 5=tundra 6=wetland/marsh 7=canyon/red-rock
#define NUM_BIOME_TEX 8
static GLuint biomeTex[NUM_BIOME_TEX];

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
#define WALKER_SPEED   14.0f
#define ENTER_DIST     6.0f   // max distance to enter a plane
// Time of day (0=midnight, 0.25=dawn, 0.5=noon, 0.75=dusk, 1=midnight)
static float gameTime = 0.35f;   // start near dawn
static float gameTimeSpeed = 0.0015f; // 1 real second = ~5 sim minutes
// Cockpit view
static bool cockpitView = false;
static bool cockpitKeyHeld = false;
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
typedef enum { MENU_MAIN, MENU_SETTINGS, MENU_JOIN, MENU_PLAYING, MENU_DEAD } MenuState;
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
#define JOIN_TIMEOUT 15.0f   // seconds before showing "no server found" (allow internet latency)

// ---- Text Chat ----
#define CHAT_MAX_MSGS   20
#define CHAT_MSG_LEN    80
#define CHAT_FADE_TIME  8.0f   // seconds before message fades
typedef struct {
    char   text[CHAT_MSG_LEN];
    float  timer;   // countdown to fade
} ChatMsg;
static ChatMsg  chatLog[CHAT_MAX_MSGS];
static int      chatCount    = 0;
static bool     chatOpen     = false;
static char     chatInput[CHAT_MSG_LEN] = {0};
static int      chatInputLen = 0;
static bool     chatKeyHeld  = false;

static void chatAddMsg(const char *txt){
    if(chatCount >= CHAT_MAX_MSGS){
        memmove(&chatLog[0], &chatLog[1], sizeof(ChatMsg)*(CHAT_MAX_MSGS-1));
        chatCount = CHAT_MAX_MSGS-1;
    }
    strncpy(chatLog[chatCount].text, txt, CHAT_MSG_LEN-1);
    chatLog[chatCount].text[CHAT_MSG_LEN-1]='\0';
    chatLog[chatCount].timer = CHAT_FADE_TIME;
    chatCount++;
}

// chatSend is defined later (after network globals) — see chatSendImpl
static char chatPendingMsg[CHAT_MSG_LEN] = {0};
static bool chatHasPending = false;
static void chatSend(const char *msg){
    char full[CHAT_MSG_LEN+8];
    snprintf(full, sizeof(full), "[me] %s", msg);
    chatAddMsg(full);
    strncpy(chatPendingMsg, msg, CHAT_MSG_LEN-1);
    chatHasPending = true;  // flushed in game loop
}

// ---- Proximity Voice Chat — globals only here; functions defined after AL types ----
#define VOICE_SAMPLE_RATE   16000
#define VOICE_FRAME_MS      20
#define VOICE_FRAME_SAMPLES (VOICE_SAMPLE_RATE * VOICE_FRAME_MS / 1000)
#define VOICE_MAX_BYTES     512
#define VOICE_PROX_DIST     200.0f
#define OPUS_APPLICATION_VOIP 2048
#define OPUS_OK 0
#define SND_PCM_FORMAT_S16_LE  2
#define SND_PCM_ACCESS_RW_INTERLEAVED 3

typedef struct OpusEncoder OpusEncoder;
typedef struct OpusDecoder OpusDecoder;
typedef void*         snd_pcm_t;
typedef unsigned long snd_pcm_uframes_t;

static void *opusLib = NULL;
static OpusEncoder* (*opus_encoder_create)(int,int,int,int*) = NULL;
static int (*opus_encode)(OpusEncoder*,const short*,int,unsigned char*,int) = NULL;
static OpusDecoder* (*opus_decoder_create)(int,int,int*) = NULL;
static int (*opus_decode)(OpusDecoder*,const unsigned char*,int,short*,int,int) = NULL;
static void *alsaLib = NULL;
static int  (*alsa_pcm_open)(snd_pcm_t**,const char*,int,int) = NULL;
static int  (*alsa_pcm_set_params)(snd_pcm_t*,int,int,unsigned int,unsigned int,int,unsigned int) = NULL;
static long (*alsa_pcm_readi)(snd_pcm_t*,void*,snd_pcm_uframes_t) = NULL;
static int  (*alsa_pcm_close)(snd_pcm_t*) = NULL;
static int  (*alsa_pcm_recover)(snd_pcm_t*,int,int) = NULL;

static bool       voiceReady      = false;
static OpusEncoder *voiceEnc      = NULL;
static OpusDecoder *voiceDec      = NULL;
static snd_pcm_t  *voicePcm       = NULL;
static pthread_t   voiceCapThread;
static bool        voiceCapRunning = false;
static bool        voiceTx         = false;
static unsigned int voiceSource    = 0;
static unsigned int voiceBuffers[8];
static int         voiceBufHead    = 0;
#define VOICE_HDR (sizeof(int)+sizeof(float)*3+sizeof(int))

// ---- Trees ----
#define MAX_TREES 18000
typedef struct {
    float x, y, z;
    float h;          // height
    float r;          // canopy radius
    int   variant;    // 0=pine, 1=oak, 2=palm (biome-dependent)
} Tree;
static Tree  trees[MAX_TREES];
static int   numTrees = 0;

// ---- Clouds ----
#define MAX_CLOUDS 500
typedef struct {
    float x, y, z;          // world position
    float r;                 // puff radius
    float brightness;        // 0.7..1.0
} Cloud;
static Cloud clouds[MAX_CLOUDS];
static int   numClouds = 0;

// ---- LAN Server Discovery ----
#define LAN_DISCOVERY_PORT 5002
#define LAN_ANNOUNCE_INTERVAL 2.0f
#define LAN_SERVER_TIMEOUT    6.0f
#define MAX_LAN_SERVERS 8
typedef struct {
    char ip[64];
    float seenTimer;    // counts down; remove when <= 0
} LanServer;
static LanServer lanServers[MAX_LAN_SERVERS];
static int       numLanServers = 0;
static float     lanAnnounceTimer = 0.0f;
static int       lanSock = -1;          // UDP socket for discovery
static pthread_t lanThread;
static pthread_mutex_t lanMutex = PTHREAD_MUTEX_INITIALIZER;
static bool      lanThreadRunning = false;
static int       selectedServer = -1;   // index into lanServers for Join UI

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
#define MAX_EXPLOSIONS 8
#define EXPLOSION_LIFETIME 0.5f
#define EXPLOSION_PARTICLES 2000
typedef struct {
    float x, y, z;
    float timer;        // counts up from 0 to EXPLOSION_LIFETIME
    bool  active;
    float speedScale;
    // Per-particle: direction, speed, type
    float px[EXPLOSION_PARTICLES];  // x direction
    float py[EXPLOSION_PARTICLES];  // y direction
    float pz[EXPLOSION_PARTICLES];  // z direction
    float ps[EXPLOSION_PARTICLES];  // speed scalar
    int   pt[EXPLOSION_PARTICLES];  // type: 0=stem, 1=cap, 2=base ring
} Explosion;
static Explosion explosions[MAX_EXPLOSIONS];

// Outbound explosion queue (written by physics, flushed by network thread)
#define MAX_EXPLODE_QUEUE 16
typedef struct { float x,y,z,scale; } ExplodeQueueEntry;
static ExplodeQueueEntry explodeQueue[MAX_EXPLODE_QUEUE];
static int explodeQueueHead = 0, explodeQueueTail = 0;
static pthread_mutex_t explodeMutex = PTHREAD_MUTEX_INITIALIZER;

// ---- Missiles ----
#define MAX_MISSILES 8
#define MISSILE_SPEED   220.0f   // world units/s
#define MISSILE_RANGE   600.0f   // max travel distance before self-destruct
#define MISSILE_HIT_R   5.0f    // hit-sphere radius against NPC planes
typedef struct {
    float x,y,z;          // current position
    float vx,vy,vz;       // velocity (normalised dir * MISSILE_SPEED)
    float dist;           // distance travelled so far
    bool  active;
} Missile;
static Missile missiles[MAX_MISSILES];
static float missileCooldown = 0.0f;   // seconds until next shot allowed
static bool  missileKeyHeld  = false;
#define MISSILE_COOLDOWN 0.6f

// ============================================================
// Sound — OpenAL via dlopen (no compile-time dep)
// ============================================================
#include <dlfcn.h>
#include <stdint.h>

#define AL_FORMAT_MONO16    0x1101
#define AL_BUFFER           0x1009
#define AL_SOURCE_STATE     0x1010
#define AL_PLAYING          0x1012
#define AL_LOOPING          0x1007
#define AL_GAIN                 0x100A
#define AL_PITCH                0x1003
#define AL_BUFFERS_PROCESSED    0x1016
#define AL_TRUE             1
#define AL_FALSE            0
#define ALC_DEFAULT_DEVICE_SPECIFIER 0x1004

typedef unsigned int ALuint;
typedef int          ALint;
typedef float        ALfloat;
typedef unsigned int ALenum;
typedef void*        ALCdevice;
typedef void*        ALCcontext;
typedef int          ALCboolean;

// Minimal function pointer set
static ALCdevice* (*palcOpenDevice)(const char*)        = NULL;
static ALCcontext*(*palcCreateContext)(ALCdevice*,const int*) = NULL;
static ALCboolean (*palcMakeContextCurrent)(ALCcontext*) = NULL;
static void       (*palGenBuffers)(ALint,ALuint*)        = NULL;
static void       (*palGenSources)(ALint,ALuint*)        = NULL;
static void       (*palBufferData)(ALuint,ALenum,const void*,ALint,ALint) = NULL;
static void       (*palSourcei)(ALuint,ALenum,ALint)     = NULL;
static void       (*palSourcef)(ALuint,ALenum,ALfloat)   = NULL;
static void       (*palSourcePlay)(ALuint)               = NULL;
static void       (*palSourceStop)(ALuint)               = NULL;
static void       (*palGetSourcei)(ALuint,ALenum,ALint*) = NULL;
static void       (*palSourceQueueBuffers)(ALuint,ALint,const ALuint*) = NULL;
static void       (*palSourceUnqueueBuffers)(ALuint,ALint,ALuint*)     = NULL;

static bool alAvailable = false;

// Sound IDs
typedef enum {
    SND_EXPLOSION=0,
    SND_MISSILE,
    SND_LAND,
    SND_GEAR,
    SND_COUNT
} SoundId;

static ALuint alBufs[SND_COUNT];
static ALuint alSrcs[SND_COUNT];
static ALuint alEngSrc;    // engine: looping source
static ALuint alEngBuf;    // engine: buffer (rebuilt when pitch changes)

#define SND_RATE 22050

static short *sndAlloc(int n){ return (short*)malloc(n*sizeof(short)); }

static short *synthBuf(int frames, float freq, float vol, bool noise_only){
    short *b = sndAlloc(frames);
    float phase = 0.0f, dp = 2.0f*(float)M_PI*freq/SND_RATE;
    for(int i=0;i<frames;i++){
        float s = 0.0f;
        if(!noise_only){
            s  = sinf(phase)*0.50f + sinf(phase*2)*0.25f
               + sinf(phase*3)*0.12f + sinf(phase*4)*0.08f;
        }
        s += ((rand()%1000)/500.0f - 1.0f) * (noise_only ? 0.8f : 0.04f);
        b[i] = (short)(s * vol * 10000.0f);
        phase += dp; if(phase>6.28318f) phase-=6.28318f;
    }
    return b;
}

static void alLoadBuf(ALuint buf, short *data, int frames){
    palBufferData(buf, AL_FORMAT_MONO16, data, frames*2, SND_RATE);
    free(data);
}

static void sndBuildEngine(float throttle){
    float freq = 55.0f + throttle * 180.0f;
    float vol  = 0.40f + throttle * 0.45f;
    int frames = SND_RATE / 2;   // 0.5s loop
    short *b = synthBuf(frames, freq, vol, false);
    // fade loop endpoints to avoid click
    int fade = SND_RATE/100;
    for(int i=0;i<fade;i++){
        float t=(float)i/fade;
        b[i]=(short)(b[i]*t);
        b[frames-1-i]=(short)(b[frames-1-i]*t);
    }
    palSourceStop(alEngSrc);
    palBufferData(alEngBuf, AL_FORMAT_MONO16, b, frames*2, SND_RATE);
    free(b);
    palSourcei(alEngSrc, AL_BUFFER, (ALint)alEngBuf);
    palSourcei(alEngSrc, AL_LOOPING, AL_TRUE);
    palSourcef(alEngSrc, AL_GAIN, 0.7f);
}

static void sndInit(){
    void *lib = dlopen("libopenal.so.1", RTLD_LAZY);
    if(!lib){ fprintf(stderr,"[snd] OpenAL not found, no audio\n"); return; }
#define LOAD(sym,name) sym = dlsym(lib,name); if(!sym){ fprintf(stderr,"[snd] missing %s\n",name); return; }
    LOAD(palcOpenDevice,       "alcOpenDevice")
    LOAD(palcCreateContext,    "alcCreateContext")
    LOAD(palcMakeContextCurrent,"alcMakeContextCurrent")
    LOAD(palGenBuffers,        "alGenBuffers")
    LOAD(palGenSources,        "alGenSources")
    LOAD(palBufferData,        "alBufferData")
    LOAD(palSourcei,           "alSourcei")
    LOAD(palSourcef,           "alSourcef")
    LOAD(palSourcePlay,        "alSourcePlay")
    LOAD(palSourceStop,        "alSourceStop")
    LOAD(palGetSourcei,            "alGetSourcei")
    LOAD(palSourceQueueBuffers,    "alSourceQueueBuffers")
    LOAD(palSourceUnqueueBuffers,  "alSourceUnqueueBuffers")
#undef LOAD
    ALCdevice  *dev = palcOpenDevice(NULL);
    if(!dev){ fprintf(stderr,"[snd] alcOpenDevice failed\n"); return; }
    ALCcontext *ctx = palcCreateContext(dev, NULL);
    palcMakeContextCurrent(ctx);

    palGenBuffers(SND_COUNT, alBufs);
    palGenSources(SND_COUNT, alSrcs);
    palGenBuffers(1, &alEngBuf);
    palGenSources(1, &alEngSrc);

    // Explosion: 1.2s noise + 40Hz rumble with decay
    {
        int frames = (int)(SND_RATE*1.2f);
        short *b = sndAlloc(frames);
        for(int i=0;i<frames;i++){
            float t=i/(float)SND_RATE, env=expf(-t*4.0f);
            float s=((rand()%32767)/16383.5f-1.0f) + sinf(2.f*(float)M_PI*40.f*t)*0.5f;
            b[i]=(short)(s*env*28000.0f);
        }
        alLoadBuf(alBufs[SND_EXPLOSION], b, frames);
    }
    // Missile: 0.4s rising hiss
    {
        int frames = (int)(SND_RATE*0.4f);
        short *b = sndAlloc(frames);
        for(int i=0;i<frames;i++){
            float t=i/(float)SND_RATE, env=t/0.4f;
            float s=((rand()%32767)/16383.5f-1.0f)*0.6f
                   +sinf(2.f*(float)M_PI*(400.f+1200.f*env)*t)*0.3f;
            b[i]=(short)(s*env*22000.0f);
        }
        alLoadBuf(alBufs[SND_MISSILE], b, frames);
    }
    // Landing thud: 0.3s
    {
        int frames = (int)(SND_RATE*0.3f);
        short *b = sndAlloc(frames);
        for(int i=0;i<frames;i++){
            float t=i/(float)SND_RATE, env=expf(-t*12.0f);
            b[i]=(short)(sinf(2.f*(float)M_PI*80.f*t)*env*28000.0f);
        }
        alLoadBuf(alBufs[SND_LAND], b, frames);
    }
    // Gear clunk: 0.2s
    {
        int frames = (int)(SND_RATE*0.2f);
        short *b = sndAlloc(frames);
        for(int i=0;i<frames;i++){
            float t=i/(float)SND_RATE, env=expf(-t*20.0f);
            float s=sinf(2.f*(float)M_PI*320.f*t)+((rand()%1000)/500.0f-1.0f)*0.3f;
            b[i]=(short)(s*env*18000.0f);
        }
        alLoadBuf(alBufs[SND_GEAR], b, frames);
    }

    for(int i=0;i<SND_COUNT;i++){
        palSourcei(alSrcs[i], AL_BUFFER, (ALint)alBufs[i]);
        palSourcef(alSrcs[i], AL_GAIN, 1.0f);
    }

    sndBuildEngine(0.0f);
    alAvailable = true;
}

static void sndPlay(SoundId id){
    if(!alAvailable) return;
    ALint state; palGetSourcei(alSrcs[id], AL_SOURCE_STATE, &state);
    if(state == AL_PLAYING) palSourceStop(alSrcs[id]);
    palSourcePlay(alSrcs[id]);
}

static float sndLastThrottle = -1.0f;
static void sndUpdateEngine(float throttle, bool running){
    if(!alAvailable) return;
    if(!running){ palSourceStop(alEngSrc); sndLastThrottle=-1.0f; return; }
    // Rebuild buffer only when throttle changes by >2%
    if(fabsf(throttle - sndLastThrottle) > 0.02f){
        sndBuildEngine(throttle);
        sndLastThrottle = throttle;
    }
    ALint state; palGetSourcei(alEngSrc, AL_SOURCE_STATE, &state);
    if(state != AL_PLAYING) palSourcePlay(alEngSrc);
}
// ---- Proximity Voice Chat — implementation (after AL types) ----
static bool voiceInit(){
    opusLib = dlopen("libopus.so.0", RTLD_LAZY);
    if(!opusLib) opusLib = dlopen("libopus.so", RTLD_LAZY);
    if(!opusLib){ fprintf(stderr,"[VOICE] libopus not found — voice disabled\n"); return false; }
    opus_encoder_create = dlsym(opusLib,"opus_encoder_create");
    opus_encode         = dlsym(opusLib,"opus_encode");
    opus_decoder_create = dlsym(opusLib,"opus_decoder_create");
    opus_decode         = dlsym(opusLib,"opus_decode");
    if(!opus_encoder_create||!opus_encode||!opus_decoder_create||!opus_decode){
        fprintf(stderr,"[VOICE] Opus symbols missing\n"); return false; }

    alsaLib = dlopen("libasound.so.2", RTLD_LAZY);
    if(!alsaLib){ fprintf(stderr,"[VOICE] libasound not found — voice disabled\n"); return false; }
    alsa_pcm_open       = dlsym(alsaLib,"snd_pcm_open");
    alsa_pcm_set_params = dlsym(alsaLib,"snd_pcm_set_params");
    alsa_pcm_readi      = dlsym(alsaLib,"snd_pcm_readi");
    alsa_pcm_close      = dlsym(alsaLib,"snd_pcm_close");
    alsa_pcm_recover    = dlsym(alsaLib,"snd_pcm_recover");

    int err;
    voiceEnc = opus_encoder_create(VOICE_SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP, &err);
    if(err != OPUS_OK){ fprintf(stderr,"[VOICE] Encoder init failed\n"); return false; }
    voiceDec = opus_decoder_create(VOICE_SAMPLE_RATE, 1, &err);
    if(err != OPUS_OK){ fprintf(stderr,"[VOICE] Decoder init failed\n"); return false; }

    if(alsa_pcm_open(&voicePcm, "default", 1, 0) < 0){
        fprintf(stderr,"[VOICE] ALSA capture open failed\n"); return false; }
    if(alsa_pcm_set_params(voicePcm, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
                           1, VOICE_SAMPLE_RATE, 1, VOICE_FRAME_MS*1000) < 0){
        fprintf(stderr,"[VOICE] ALSA set_params failed\n"); return false; }

    if(palGenSources){ palGenSources(1, (ALuint*)&voiceSource); }
    if(palGenBuffers){ palGenBuffers(8, (ALuint*)voiceBuffers); }

    voiceReady = true;
    fprintf(stderr,"[VOICE] Ready — hold V to transmit (Opus %d Hz)\n", VOICE_SAMPLE_RATE);
    return true;
}

static void voiceReceive(const char *buf, int n){
    if(!voiceReady || n < (int)VOICE_HDR) return;
    const char *p = buf + sizeof(int);
    float rx2, ry2, rz2;
    memcpy(&rx2, p,   sizeof(float)); p += sizeof(float);
    memcpy(&ry2, p,   sizeof(float)); p += sizeof(float);
    memcpy(&rz2, p,   sizeof(float)); p += sizeof(float);
    int payLen; memcpy(&payLen, p, sizeof(int)); p += sizeof(int);
    if(payLen <= 0 || payLen > VOICE_MAX_BYTES || n < (int)(VOICE_HDR+payLen)) return;

    float myX = inPlane ? player.x : walkerX;
    float myY = inPlane ? player.y : walkerY;
    float myZ = inPlane ? player.z : walkerZ;
    float dx2=rx2-myX, dy2=ry2-myY, dz2=rz2-myZ;
    float dist = sqrtf(dx2*dx2+dy2*dy2+dz2*dz2);
    if(dist > VOICE_PROX_DIST) return;
    float gain = 1.0f - dist/VOICE_PROX_DIST;

    short pcmOut[VOICE_FRAME_SAMPLES];
    int frames = opus_decode(voiceDec, (const unsigned char*)p, payLen,
                             pcmOut, VOICE_FRAME_SAMPLES, 0);
    if(frames <= 0) return;

    if(!palBufferData || !palSourceQueueBuffers || !voiceSource) return;

    // Dequeue any AL buffers the source has already finished playing.
    {
        ALint processed = 0;
        palGetSourcei((ALuint)voiceSource, AL_BUFFERS_PROCESSED, &processed);
        while(processed-- > 0){
            ALuint tmp;
            palSourceUnqueueBuffers((ALuint)voiceSource, 1, &tmp);
        }
    }

    ALuint vbuf = (ALuint)voiceBuffers[voiceBufHead % 8];
    voiceBufHead++;
    palBufferData(vbuf, AL_FORMAT_MONO16, pcmOut, frames*2, VOICE_SAMPLE_RATE);
    palSourceQueueBuffers((ALuint)voiceSource, 1, &vbuf);
    if(palSourcef)  palSourcef((ALuint)voiceSource, AL_GAIN, gain*0.9f);
    // Only call Play if not already playing (re-starting a playing streaming source
    // resets the queue position).
    {
        ALint state = 0;
        palGetSourcei((ALuint)voiceSource, AL_SOURCE_STATE, &state);
        if(state != AL_PLAYING) palSourcePlay((ALuint)voiceSource);
    }
}

static void* voiceCapThreadFunc(void *arg){
    (void)arg;
    // These are defined later in the TU — access via extern
    extern int sockfd;
    extern struct sockaddr_in otherAddr;
    extern bool isServer, clientConnected, seedReceived;
    extern pthread_mutex_t playerMutex;
    extern bool inPlane; extern float walkerX,walkerY,walkerZ;
    short pcmIn[VOICE_FRAME_SAMPLES];
    unsigned char encoded[VOICE_MAX_BYTES];
    while(voiceCapRunning){
        if(!voiceTx){ usleep(5000); continue; }
        long frames = alsa_pcm_readi(voicePcm, pcmIn, VOICE_FRAME_SAMPLES);
        if(frames < 0){ alsa_pcm_recover(voicePcm,(int)frames,0); continue; }
        int encLen = opus_encode(voiceEnc, pcmIn, (int)frames, encoded, VOICE_MAX_BYTES);
        if(encLen <= 0) continue;

        char pkt[sizeof(int)+sizeof(float)*3+sizeof(int)+VOICE_MAX_BYTES];
        char *wp = pkt;
        int type = MSG_VOICE; memcpy(wp,&type,sizeof(int)); wp+=sizeof(int);
        pthread_mutex_lock(&playerMutex);
        float px2=inPlane?player.x:walkerX, py2=inPlane?player.y:walkerY, pz2=inPlane?player.z:walkerZ;
        pthread_mutex_unlock(&playerMutex);
        memcpy(wp,&px2,4); wp+=4; memcpy(wp,&py2,4); wp+=4; memcpy(wp,&pz2,4); wp+=4;
        memcpy(wp,&encLen,sizeof(int)); wp+=sizeof(int);
        memcpy(wp,encoded,encLen);
        int total=(int)(wp-pkt)+encLen;
        if((isServer&&clientConnected)||(!isServer&&seedReceived))
            sendto(sockfd,pkt,total,0,(struct sockaddr*)&otherAddr,sizeof(otherAddr));
    }
    return NULL;
}

static void voiceStart(){
    if(!voiceReady) return;
    voiceCapRunning=true;
    pthread_create(&voiceCapThread,NULL,voiceCapThreadFunc,NULL);
}

// ============================================================
static void queueExplosionBroadcast(float x, float y, float z, float scale){
    pthread_mutex_lock(&explodeMutex);
    int next = (explodeQueueTail+1) % MAX_EXPLODE_QUEUE;
    if(next != explodeQueueHead){  // drop if full
        explodeQueue[explodeQueueTail] = (ExplodeQueueEntry){x,y,z,scale};
        explodeQueueTail = next;
    }
    pthread_mutex_unlock(&explodeMutex);
}

static void initExplosionParticles(Explosion *e, float speedScale){
    e->speedScale = speedScale;
    // 0..34%  = stem (straight up, tight cone)
    // 35..64% = cap  (upward then outward curl)
    // 65..79% = base ring (radial outward, low)
    // 80..99% = dust (very slow expanding ground cloud)
    int stemCount = (int)(EXPLOSION_PARTICLES * 0.35f);
    int capCount  = (int)(EXPLOSION_PARTICLES * 0.30f);
    int ringCount = (int)(EXPLOSION_PARTICLES * 0.15f);
    // remaining = dust
    for(int p=0;p<EXPLOSION_PARTICLES;p++){
        float th = ((float)rand()/RAND_MAX)*2.0f*(float)M_PI;
        if(p < stemCount){
            float spread = 0.18f;
            float ph = ((float)rand()/RAND_MAX)*spread;
            e->px[p] = sinf(ph)*cosf(th);
            e->py[p] = cosf(ph);
            e->pz[p] = sinf(ph)*sinf(th);
            e->ps[p] = (3.0f + ((float)rand()/RAND_MAX)*3.0f) * speedScale;
            e->pt[p] = 0;
        } else if(p < stemCount+capCount){
            float outward = 0.5f + ((float)rand()/RAND_MAX)*0.5f;
            float upward  = 0.3f + ((float)rand()/RAND_MAX)*0.4f;
            float mag = sqrtf(outward*outward + upward*upward);
            e->px[p] = (outward/mag)*cosf(th);
            e->py[p] =  upward/mag;
            e->pz[p] = (outward/mag)*sinf(th);
            e->ps[p] = (2.5f + ((float)rand()/RAND_MAX)*2.5f) * speedScale;
            e->pt[p] = 1;
        } else if(p < stemCount+capCount+ringCount){
            float ph = ((float)rand()/RAND_MAX)*0.35f;
            e->px[p] = cosf(ph)*cosf(th);
            e->py[p] = sinf(ph)*0.3f;
            e->pz[p] = cosf(ph)*sinf(th);
            e->ps[p] = (3.0f + ((float)rand()/RAND_MAX)*4.0f) * speedScale;
            e->pt[p] = 2;
        } else {
            // Dust: flat ring, very slow drift, large soft particles
            e->px[p] = cosf(th);
            e->py[p] = 0.02f + ((float)rand()/RAND_MAX)*0.06f; // barely lifts
            e->pz[p] = sinf(th);
            e->ps[p] = (0.5f + ((float)rand()/RAND_MAX)*1.5f) * speedScale;
            e->pt[p] = 3;
        }
    }
}

static void triggerExplosionEx(float x, float y, float z, float speedScale){
    for(int i=0;i<MAX_EXPLOSIONS;i++){
        if(!explosions[i].active){
            explosions[i].x = x; explosions[i].y = y; explosions[i].z = z;
            explosions[i].timer = 0.0f;
            explosions[i].active = true;
            initExplosionParticles(&explosions[i], speedScale);
            queueExplosionBroadcast(x, y, z, speedScale);
            sndPlay(SND_EXPLOSION);
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
            initExplosionParticles(&explosions[i], speedScale);
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
static char localIP[64] = "?.?.?.?";  // populated at network init
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
    // Windshield frame
    glColor3f(0.35f,0.35f,0.38f);
    glBegin(GL_QUADS);
        glNormal3f(0,0.5f,1);
        glVertex3f(-0.115f,0.12f,0.66f); glVertex3f( 0.115f,0.12f,0.66f);
        glVertex3f( 0.115f,0.30f,0.49f); glVertex3f(-0.115f,0.30f,0.49f);
    glEnd();
    // Windshield glass
    glColor3f(0.18f,0.28f,0.50f);
    // split into two panes with center post
    for(int p=-1;p<=1;p+=2){
        float x0=p*0.01f, x1=p*0.10f;
        if(p<0){ x0=-0.10f; x1=-0.01f; } else { x0=0.01f; x1=0.10f; }
        glBegin(GL_QUADS);
            glNormal3f(0,0.5f,1);
            glVertex3f(x0,0.14f,0.65f); glVertex3f(x1,0.14f,0.65f);
            glVertex3f(x1,0.28f,0.50f); glVertex3f(x0,0.28f,0.50f);
        glEnd();
    }
    // Spinner hub (round nose piece for propeller)
    glColor3f(0.60f,0.60f,0.58f);
    drawOctSegment(0.00f,0.00f,1.14f, 0.06f,0.06f,1.11f);
    drawOctCap(0.00f,0.00f,1.14f, 0,0,1);
    // Propeller — 3-blade with proper twist and hub
    glColor3f(0.20f,0.18f,0.14f);
    for(int b=0;b<3;b++){
        glPushMatrix();
        glRotatef(b*120.0f, 0,0,1);
        // Blade with chord taper and slight twist
        glBegin(GL_QUADS);
            glNormal3f(0,0,1);
            // root to tip
            glVertex3f(-0.045f, 0.04f, 1.12f); glVertex3f( 0.045f, 0.04f,1.12f);
            glVertex3f( 0.035f, 0.10f, 1.12f); glVertex3f(-0.035f, 0.10f,1.12f);

            glVertex3f(-0.035f, 0.10f, 1.12f); glVertex3f( 0.035f, 0.10f,1.12f);
            glVertex3f( 0.020f, 0.55f, 1.12f); glVertex3f(-0.020f, 0.55f,1.12f);
        glEnd();
        // blade tip cap
        glBegin(GL_TRIANGLES);
            glNormal3f(0,1,0);
            glVertex3f(-0.020f,0.55f,1.12f);
            glVertex3f( 0.020f,0.55f,1.12f);
            glVertex3f( 0.000f,0.60f,1.12f);
        glEnd();
        // blade backface (slightly darker)
        glColor3f(0.14f,0.12f,0.10f);
        glBegin(GL_QUADS);
            glNormal3f(0,0,-1);
            glVertex3f( 0.045f, 0.04f, 1.118f); glVertex3f(-0.045f, 0.04f,1.118f);
            glVertex3f(-0.035f, 0.10f, 1.118f); glVertex3f( 0.035f, 0.10f,1.118f);

            glVertex3f( 0.035f, 0.10f, 1.118f); glVertex3f(-0.035f, 0.10f,1.118f);
            glVertex3f(-0.020f, 0.55f, 1.118f); glVertex3f( 0.020f, 0.55f,1.118f);
        glEnd();
        glColor3f(0.20f,0.18f,0.14f);
        glPopMatrix();
    }
    // Door outline (right side)
    glColor3f(0.60f,0.60f,0.60f);
    glBegin(GL_LINE_LOOP);
        glVertex3f( 0.185f, -0.06f,  0.20f);
        glVertex3f( 0.185f,  0.10f,  0.20f);
        glVertex3f( 0.185f,  0.10f, -0.10f);
        glVertex3f( 0.185f, -0.06f, -0.10f);
    glEnd();
    // Registration markings — dark hash marks on tail (faked as a stripe)
    glColor3f(0.10f,0.10f,0.10f);
    glBegin(GL_QUADS);
        glNormal3f(1,0,0);
        glVertex3f( 0.185f,-0.03f,-0.42f); glVertex3f( 0.185f,-0.03f,-0.55f);
        glVertex3f( 0.185f, 0.03f,-0.55f); glVertex3f( 0.185f, 0.03f,-0.42f);
    glEnd();
    glBegin(GL_QUADS);
        glNormal3f(-1,0,0);
        glVertex3f(-0.185f,-0.03f,-0.42f); glVertex3f(-0.185f, 0.03f,-0.42f);
        glVertex3f(-0.185f, 0.03f,-0.55f); glVertex3f(-0.185f,-0.03f,-0.55f);
    glEnd();
    // Fixed tricycle gear — always present, strut compresses when on ground (gear→1)
    // Strut compression: on ground strut is shorter, wheel is closer to fuselage
    float strutCompress = gear * 0.06f;  // up to 6% shorter when on ground

    // Main gear legs + wheel fairings
    for(int s=-1;s<=1;s+=2){
        float legBot = -0.28f + strutCompress;   // bottom of strut (rises when compressed)
        glColor3f(0.35f,0.35f,0.38f);
        // Vertical strut
        glBegin(GL_QUADS);
            glNormal3f(s,0,0);
            glVertex3f(s*0.05f, 0.0f,  0.05f); glVertex3f(s*0.05f, legBot, 0.05f);
            glVertex3f(s*0.05f, legBot,-0.05f); glVertex3f(s*0.05f, 0.0f, -0.05f);
        glEnd();
        // Axle strut (horizontal arm to wheel)
        glBegin(GL_QUADS);
            glNormal3f(0,-1,0);
            glVertex3f(s*0.05f, legBot, 0.04f); glVertex3f(s*0.22f, legBot, 0.04f);
            glVertex3f(s*0.22f, legBot,-0.04f); glVertex3f(s*0.05f, legBot,-0.04f);
        glEnd();
        // Wheel fairing (teardrop streamline cover)
        glColor3f(0.85f,0.85f,0.82f);
        glPushMatrix(); glTranslatef(s*0.22f, legBot, 0.0f);
        drawOctSegment(0.11f,0.09f, 0.12f, 0.11f,0.09f,-0.14f);
        drawOctCap(0.11f,0.09f,  0.12f, 0,0, 1);
        drawOctCap(0.11f,0.09f, -0.14f, 0,0,-1);
        // Main wheel tire (inside fairing)
        glColor3f(0.10f,0.10f,0.10f);
        for(int side=-1;side<=1;side+=2){
            glNormal3f(0,0,side);
            glBegin(GL_TRIANGLE_FAN);
                glVertex3f(0,0,side*0.055f);
                for(int i=0;i<=10;i++){ float a=i*6.28318f/10;
                    glVertex3f(cosf(a)*0.095f,sinf(a)*0.095f,side*0.055f); }
            glEnd();
        }
        glBegin(GL_TRIANGLE_STRIP);
        for(int i=0;i<=10;i++){ float a=i*6.28318f/10;
            glNormal3f(cosf(a),sinf(a),0);
            glVertex3f(cosf(a)*0.095f,sinf(a)*0.095f, 0.055f);
            glVertex3f(cosf(a)*0.095f,sinf(a)*0.095f,-0.055f);
        }
        glEnd();
        // Hub cap
        glColor3f(0.55f,0.55f,0.58f);
        glNormal3f(0,0,1);
        glBegin(GL_TRIANGLE_FAN);
            glVertex3f(0,0,0.057f);
            for(int i=0;i<=6;i++){ float a=i*6.28318f/6;
                glVertex3f(cosf(a)*0.035f,sinf(a)*0.035f,0.057f); }
        glEnd();
        glPopMatrix();
        glColor3f(0.35f,0.35f,0.38f);
    }
    // Nose gear (steerable front leg)
    {
        float nlegBot = -0.22f + strutCompress;
        glColor3f(0.35f,0.35f,0.38f);
        // Strut
        glBegin(GL_QUADS);
            glNormal3f(1,0,0);
            glVertex3f( 0.025f, 0.0f, 0.025f); glVertex3f( 0.025f, nlegBot, 0.025f);
            glVertex3f( 0.025f, nlegBot,-0.025f); glVertex3f( 0.025f, 0.0f,-0.025f);
            glNormal3f(-1,0,0);
            glVertex3f(-0.025f, 0.0f, 0.025f); glVertex3f(-0.025f, 0.0f,-0.025f);
            glVertex3f(-0.025f, nlegBot,-0.025f); glVertex3f(-0.025f, nlegBot, 0.025f);
            glNormal3f(0,0, 1);
            glVertex3f(-0.025f, 0.0f, 0.025f); glVertex3f( 0.025f, 0.0f, 0.025f);
            glVertex3f( 0.025f, nlegBot, 0.025f); glVertex3f(-0.025f, nlegBot, 0.025f);
            glNormal3f(0,0,-1);
            glVertex3f(-0.025f, 0.0f,-0.025f); glVertex3f(-0.025f, nlegBot,-0.025f);
            glVertex3f( 0.025f, nlegBot,-0.025f); glVertex3f( 0.025f, 0.0f,-0.025f);
        glEnd();
        // Nose wheel
        glColor3f(0.10f,0.10f,0.10f);
        glPushMatrix(); glTranslatef(0.0f, nlegBot, 0.0f);
        float nwr=0.065f, nwt=0.040f;
        for(int side=-1;side<=1;side+=2){
            glNormal3f(0,0,side);
            glBegin(GL_TRIANGLE_FAN);
                glVertex3f(0,0,side*nwt);
                for(int i=0;i<=8;i++){ float a=i*6.28318f/8;
                    glVertex3f(cosf(a)*nwr,sinf(a)*nwr,side*nwt); }
            glEnd();
        }
        glBegin(GL_TRIANGLE_STRIP);
        for(int i=0;i<=8;i++){ float a=i*6.28318f/8;
            glNormal3f(cosf(a),sinf(a),0);
            glVertex3f(cosf(a)*nwr,sinf(a)*nwr, nwt);
            glVertex3f(cosf(a)*nwr,sinf(a)*nwr,-nwt);
        }
        glEnd();
        glColor3f(0.55f,0.55f,0.58f);
        glNormal3f(0,0,1);
        glBegin(GL_TRIANGLE_FAN);
            glVertex3f(0,0,nwt+0.004f);
            for(int i=0;i<=6;i++){ float a=i*6.28318f/6;
                glVertex3f(cosf(a)*0.025f,sinf(a)*0.025f,nwt+0.004f); }
        glEnd();
        glPopMatrix();
    }
    // Tail skid (small rubber pad under tail)
    glColor3f(0.15f,0.15f,0.15f);
    glBegin(GL_QUADS);
        glNormal3f(0,-1,0);
        glVertex3f(-0.025f,-0.10f,-0.82f); glVertex3f( 0.025f,-0.10f,-0.82f);
        glVertex3f( 0.025f,-0.10f,-0.88f); glVertex3f(-0.025f,-0.10f,-0.88f);
    glEnd();
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
    // 4 engine pods with fan faces and pylons
    for(int e=0;e<4;e++){
        float ex = (e<2 ? 0.90f : 1.70f) * (e%2==0 ? 1 : -1);
        glPushMatrix(); glTranslatef(ex, -0.14f, 0.10f);
        // Engine pylon (connects wing to pod)
        glColor3f(0.78f,0.78f,0.80f);
        glBegin(GL_QUADS);
            glNormal3f(0,1,0);
            glVertex3f(-0.025f,0.10f, 0.20f); glVertex3f( 0.025f,0.10f, 0.20f);
            glVertex3f( 0.025f,0.10f,-0.30f); glVertex3f(-0.025f,0.10f,-0.30f);
        glEnd();
        glBegin(GL_QUADS);
            glNormal3f(1,0,0);
            glVertex3f(0.025f,0.10f, 0.20f); glVertex3f(0.025f,-0.01f, 0.20f);
            glVertex3f(0.025f,-0.01f,-0.30f); glVertex3f(0.025f,0.10f,-0.30f);
        glEnd();
        glBegin(GL_QUADS);
            glNormal3f(-1,0,0);
            glVertex3f(-0.025f,0.10f, 0.20f); glVertex3f(-0.025f,0.10f,-0.30f);
            glVertex3f(-0.025f,-0.01f,-0.30f); glVertex3f(-0.025f,-0.01f, 0.20f);
        glEnd();
        // Nacelle body
        glColor3f(0.40f,0.40f,0.45f);
        drawOctSegment(0.12f,0.10f, 0.55f, 0.12f,0.10f,-0.55f);
        // Intake lip (forward nacelle flare)
        glColor3f(0.55f,0.55f,0.60f);
        drawOctSegment(0.12f,0.10f, 0.55f, 0.15f,0.13f, 0.65f);
        drawOctCap(0.15f,0.13f, 0.65f, 0,0,1);
        // Fan face (visible inside intake)
        glColor3f(0.18f,0.18f,0.20f);
        glBegin(GL_TRIANGLE_FAN);
            glNormal3f(0,0,1);
            glVertex3f(0,0,0.55f);
            for(int i=0;i<=10;i++){ float a=i*6.28318f/10;
                glVertex3f(cosf(a)*0.115f,sinf(a)*0.115f,0.55f); }
        glEnd();
        // Fan blades
        glColor3f(0.30f,0.30f,0.34f);
        for(int fb=0;fb<8;fb++){
            float fa=fb*6.28318f/8;
            glBegin(GL_QUADS);
                glNormal3f(0,0,1);
                glVertex3f(cosf(fa)*0.025f,sinf(fa)*0.025f,0.552f);
                glVertex3f(cosf(fa+0.18f)*0.025f,sinf(fa+0.18f)*0.025f,0.552f);
                glVertex3f(cosf(fa+0.12f)*0.110f,sinf(fa+0.12f)*0.110f,0.552f);
                glVertex3f(cosf(fa)*0.110f,sinf(fa)*0.110f,0.552f);
            glEnd();
        }
        // Exhaust nozzle
        glColor3f(0.28f,0.28f,0.30f);
        drawOctSegment(0.12f,0.10f,-0.55f, 0.08f,0.07f,-0.70f);
        drawOctCap(0.08f,0.07f,-0.70f, 0,0,-1);
        // Exhaust plug (dark centre)
        glColor3f(0.15f,0.15f,0.16f);
        glBegin(GL_TRIANGLE_FAN);
            glNormal3f(0,0,-1);
            glVertex3f(0,0,-0.66f);
            for(int i=0;i<=8;i++){ float a=i*6.28318f/8;
                glVertex3f(cosf(a)*0.05f,sinf(a)*0.05f,-0.66f); }
        glEnd();
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
    // Winglets (angled tip fins at each wingtip)
    glColor3f(0.10f,0.30f,0.75f);
    for(int s=-1;s<=1;s+=2){
        float wx = s*2.20f;
        // winglet side faces
        for(int side=-1;side<=1;side+=2){
            glNormal3f(0,0,side);
            glBegin(GL_QUADS);
                glVertex3f(wx,         0.00f, side*0.035f + (side>0?-0.60f:-1.20f));
                glVertex3f(wx,         0.00f, side*0.035f + (side>0?-1.20f:-0.60f));
                glVertex3f(wx+s*0.00f, 0.38f, (side>0?-0.72f:-1.08f));
                glVertex3f(wx+s*0.00f, 0.38f, (side>0?-1.08f:-0.72f));
            glEnd();
        }
        // winglet leading edge
        glBegin(GL_QUADS); glNormal3f(0,0,1);
            glVertex3f(wx,        0.00f,-0.60f); glVertex3f(wx,        0.38f,-0.72f);
            glVertex3f(wx,        0.38f,-0.72f); glVertex3f(wx,        0.00f,-0.60f);
        glEnd();
        // winglet top cap
        glBegin(GL_QUADS); glNormal3f(0,1,0);
            glVertex3f(wx-0.035f, 0.38f,-0.72f); glVertex3f(wx+0.035f, 0.38f,-0.72f);
            glVertex3f(wx+0.035f, 0.38f,-1.08f); glVertex3f(wx-0.035f, 0.38f,-1.08f);
        glEnd();
    }
    // Window frames (thin border around each pane)
    glColor3f(0.75f,0.75f,0.78f);
    for(int w=0;w<10;w++){
        float wz = 1.30f - w * 0.28f;
        if(wz < -0.90f) break;
        for(int side=-1;side<=1;side+=2){
            float wx2 = side*0.437f;
            glBegin(GL_LINE_LOOP);
                glVertex3f(wx2, 0.07f, wz+0.07f);
                glVertex3f(wx2, 0.07f, wz-0.07f);
                glVertex3f(wx2, 0.21f, wz-0.07f);
                glVertex3f(wx2, 0.21f, wz+0.07f);
            glEnd();
        }
    }
    // Nose radome tip (darker composite cone)
    glColor3f(0.22f,0.22f,0.24f);
    drawOctSegment(0.00f,0.00f, 2.40f, 0.10f,0.10f, 2.20f);
    drawOctCap(0.00f,0.00f, 2.40f, 0,0,1);
    // Landing gear — retracts/deploys with gear param (0=up, 1=down)
    if(gear > 0.001f){
        float deployAngle = gear * 90.0f;
        float compress = gear * 0.08f; // strut compression when on ground

        // ---- Nose gear (folds forward into nose bay) ----
        glPushMatrix();
        glTranslatef(0.0f, -0.26f, 1.60f);
        glRotatef(-(90.0f - deployAngle), 1, 0, 0);
        float nsh = 0.50f * gear - compress;
        glColor3f(0.35f,0.35f,0.38f);
        // Strut (front + back + sides)
        glBegin(GL_QUADS);
            glNormal3f( 1,0,0);
            glVertex3f( 0.05f, 0.0f, -0.025f); glVertex3f( 0.05f, -nsh, -0.025f);
            glVertex3f( 0.05f, -nsh,  0.025f); glVertex3f( 0.05f, 0.0f,  0.025f);
            glNormal3f(-1,0,0);
            glVertex3f(-0.05f, 0.0f, -0.025f); glVertex3f(-0.05f, 0.0f,  0.025f);
            glVertex3f(-0.05f, -nsh,  0.025f); glVertex3f(-0.05f, -nsh, -0.025f);
            glNormal3f(0,0, 1);
            glVertex3f(-0.05f, 0.0f, 0.025f); glVertex3f( 0.05f, 0.0f, 0.025f);
            glVertex3f( 0.05f, -nsh, 0.025f); glVertex3f(-0.05f, -nsh, 0.025f);
            glNormal3f(0,0,-1);
            glVertex3f(-0.05f, 0.0f,-0.025f); glVertex3f(-0.05f, -nsh,-0.025f);
            glVertex3f( 0.05f, -nsh,-0.025f); glVertex3f( 0.05f, 0.0f,-0.025f);
        glEnd();
        // Nose wheel (dual, narrow)
        glTranslatef(0.0f, -nsh, 0.0f);
        glColor3f(0.10f,0.10f,0.10f);
        float nwr = 0.10f, nwt = 0.045f;
        for(int w = -1; w <= 1; w += 2){
            glPushMatrix(); glTranslatef(0, 0, w * 0.06f);
            for(int side=-1;side<=1;side+=2){
                glNormal3f(0,0,side);
                glBegin(GL_TRIANGLE_FAN);
                    glVertex3f(0,0,side*nwt);
                    for(int i=0;i<=10;i++){ float a=i*6.28318f/10;
                        glVertex3f(cosf(a)*nwr,sinf(a)*nwr,side*nwt); }
                glEnd();
            }
            glBegin(GL_TRIANGLE_STRIP);
            for(int i=0;i<=10;i++){ float a=i*6.28318f/10;
                glNormal3f(cosf(a),sinf(a),0);
                glVertex3f(cosf(a)*nwr,sinf(a)*nwr, nwt);
                glVertex3f(cosf(a)*nwr,sinf(a)*nwr,-nwt);
            }
            glEnd();
            // Hub
            glColor3f(0.52f,0.52f,0.55f);
            glNormal3f(0,0,1);
            glBegin(GL_TRIANGLE_FAN);
                glVertex3f(0,0,nwt+0.004f);
                for(int i=0;i<=6;i++){ float a=i*6.28318f/6;
                    glVertex3f(cosf(a)*0.04f,sinf(a)*0.04f,nwt+0.004f); }
            glEnd();
            glColor3f(0.10f,0.10f,0.10f);
            glPopMatrix();
        }
        glPopMatrix();

        // ---- Main gear (bogies, fold inward under wing) ----
        for(int s = -1; s <= 1; s += 2){
            glPushMatrix();
            glTranslatef(s * 0.58f, -0.26f, -0.10f);
            glRotatef(s * (90.0f - deployAngle), 0, 0, 1);
            glColor3f(0.35f,0.35f,0.38f);
            float sh = 0.55f * gear - compress;

            // Main strut
            glBegin(GL_QUADS);
                glNormal3f( 1,0,0);
                glVertex3f( 0.05f, 0.0f,-0.05f); glVertex3f( 0.05f,-sh,-0.05f);
                glVertex3f( 0.05f,-sh, 0.05f);   glVertex3f( 0.05f, 0.0f, 0.05f);
                glNormal3f(-1,0,0);
                glVertex3f(-0.05f, 0.0f,-0.05f); glVertex3f(-0.05f, 0.0f, 0.05f);
                glVertex3f(-0.05f,-sh, 0.05f);   glVertex3f(-0.05f,-sh,-0.05f);
                glNormal3f(0,0, 1);
                glVertex3f(-0.05f, 0.0f, 0.05f); glVertex3f( 0.05f, 0.0f, 0.05f);
                glVertex3f( 0.05f,-sh,  0.05f);  glVertex3f(-0.05f,-sh,  0.05f);
                glNormal3f(0,0,-1);
                glVertex3f(-0.05f, 0.0f,-0.05f); glVertex3f(-0.05f,-sh,-0.05f);
                glVertex3f( 0.05f,-sh,-0.05f);   glVertex3f( 0.05f, 0.0f,-0.05f);
            glEnd();

            // Bogie beam (horizontal axle bar)
            glTranslatef(0.0f, -sh, 0.0f);
            glBegin(GL_QUADS);
                glNormal3f(0,0,1);
                glVertex3f(-0.04f, 0.03f, 0.04f); glVertex3f( 0.04f, 0.03f, 0.04f);
                glVertex3f( 0.04f,-0.03f, 0.04f); glVertex3f(-0.04f,-0.03f, 0.04f);
                glNormal3f(0,0,-1);
                glVertex3f(-0.04f,-0.03f,-0.24f); glVertex3f( 0.04f,-0.03f,-0.24f);
                glVertex3f( 0.04f, 0.03f,-0.24f); glVertex3f(-0.04f, 0.03f,-0.24f);
                glNormal3f(0,1,0);
                glVertex3f(-0.04f, 0.03f,-0.24f); glVertex3f( 0.04f, 0.03f,-0.24f);
                glVertex3f( 0.04f, 0.03f, 0.04f); glVertex3f(-0.04f, 0.03f, 0.04f);
                glNormal3f(0,-1,0);
                glVertex3f(-0.04f,-0.03f, 0.04f); glVertex3f( 0.04f,-0.03f, 0.04f);
                glVertex3f( 0.04f,-0.03f,-0.24f); glVertex3f(-0.04f,-0.03f,-0.24f);
            glEnd();

            // 4 wheels: 2 axle positions × 2 per axle
            float axleZ[2] = { 0.02f, -0.22f };
            float wr = 0.13f, wt = 0.055f;
            for(int ax = 0; ax < 2; ax++){
                for(int w = -1; w <= 1; w += 2){
                    glPushMatrix(); glTranslatef(0, 0, axleZ[ax] + w * 0.10f);
                    glColor3f(0.10f,0.10f,0.10f);
                    for(int side=-1;side<=1;side+=2){
                        glNormal3f(0,0,side);
                        glBegin(GL_TRIANGLE_FAN);
                            glVertex3f(0,0,side*wt);
                            for(int i=0;i<=10;i++){ float a=i*6.28318f/10;
                                glVertex3f(cosf(a)*wr,sinf(a)*wr,side*wt); }
                        glEnd();
                    }
                    glBegin(GL_TRIANGLE_STRIP);
                    for(int i=0;i<=10;i++){ float a=i*6.28318f/10;
                        glNormal3f(cosf(a),sinf(a),0);
                        glVertex3f(cosf(a)*wr,sinf(a)*wr, wt);
                        glVertex3f(cosf(a)*wr,sinf(a)*wr,-wt);
                    }
                    glEnd();
                    // Hub cap
                    glColor3f(0.52f,0.52f,0.55f);
                    glNormal3f(0,0,1);
                    glBegin(GL_TRIANGLE_FAN);
                        glVertex3f(0,0,wt+0.005f);
                        for(int i=0;i<=6;i++){ float a=i*6.28318f/6;
                            glVertex3f(cosf(a)*0.05f,sinf(a)*0.05f,wt+0.005f); }
                    glEnd();
                    glPopMatrix();
                }
            }
            glPopMatrix();
        }

        // Gear bay doors
        float doorAngle = gear * 70.0f;
        glColor3f(0.88f,0.88f,0.90f);
        // Nose doors (two halves)
        for(int s=-1;s<=1;s+=2){
            glPushMatrix();
            glTranslatef(s*0.06f, -0.26f, 1.60f);
            glRotatef(s*doorAngle, 0, 0, 1);
            glNormal3f(0,-1,0);
            glBegin(GL_QUADS);
                glVertex3f(0,0, 0.25f); glVertex3f(s*0.14f,0, 0.25f);
                glVertex3f(s*0.14f,0,-0.08f); glVertex3f(0,0,-0.08f);
            glEnd();
            glPopMatrix();
        }
        // Main gear doors
        for(int s=-1;s<=1;s+=2){
            glPushMatrix();
            glTranslatef(s*0.58f, -0.26f, -0.10f);
            glRotatef(-s*doorAngle, 1, 0, 0);
            glNormal3f(0,-1,0);
            glBegin(GL_QUADS);
                glVertex3f(-s*0.20f,0, 0.28f); glVertex3f(s*0.06f,0, 0.28f);
                glVertex3f(s*0.06f,0,-0.30f);  glVertex3f(-s*0.20f,0,-0.30f);
            glEnd();
            glPopMatrix();
        }
    }
}

// Draw a nav/strobe light glow at local position (lx,ly,lz) with given colour and radius.
static void drawNavLight(float lx, float ly, float lz,
                         float r, float g, float b, float radius){
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glColor4f(r, g, b, 0.85f);
    glBegin(GL_TRIANGLE_FAN);
        glVertex3f(lx, ly, lz);
        for(int i=0;i<=10;i++){
            float a = i * 6.28318f / 10.0f;
            glVertex3f(lx + cosf(a)*radius, ly + sinf(a)*radius, lz);
        }
    glEnd();
    glEnable(GL_LIGHTING); glDisable(GL_BLEND);
}

// Draw a full set of nav+strobe+beacon lights in the current model-local space.
// wingtipX: half-span (lights at ±wingtipX, wingtipY, wingtipZ)
// tailZ: −Z position of white tail light
// topY: Y of red rotating beacon
static void drawPlaneNavLights(float wingtipX, float wingtipY, float wingtipZ,
                                float tailZ, float topY){
    float t = (float)clock() / (float)CLOCKS_PER_SEC;
    // Steady nav: red left, green right
    drawNavLight(-wingtipX, wingtipY, wingtipZ,  1.0f, 0.05f, 0.05f, 0.06f);
    drawNavLight( wingtipX, wingtipY, wingtipZ,  0.05f, 1.0f, 0.05f, 0.06f);
    // White tail light
    drawNavLight(0.0f, 0.0f, tailZ,  1.0f, 1.0f, 1.0f, 0.05f);
    // Strobe: sharp 60ms flash every 1.5s on each wingtip and tail
    float strobe = fmodf(t, 1.5f);
    if(strobe < 0.06f){
        float si = 1.0f - strobe/0.06f;
        drawNavLight(-wingtipX, wingtipY, wingtipZ, si, si, si, 0.12f);
        drawNavLight( wingtipX, wingtipY, wingtipZ, si, si, si, 0.12f);
        drawNavLight(0.0f, 0.0f, tailZ,             si, si, si, 0.10f);
    }
    // Red anti-collision beacon: pulses at ~1 Hz on top of fuselage
    float beacon = 0.5f + 0.5f*sinf(t * 6.28318f * 1.0f);
    drawNavLight(0.0f, topY, 0.0f,  1.0f, 0.05f, 0.05f, 0.05f + beacon*0.07f);
}

void drawPlaneModel(float x,float y,float z,float pitch,float yaw,float roll,float gear,PlaneType type){
    glPushMatrix();
    glTranslatef(x,y,z);
    glRotatef(yaw*57.2958f,0,1,0);
    glRotatef(-pitch*57.2958f,1,0,0);
    glRotatef(roll*57.2958f,0,0,1);
    float sc = PLANE_DEFS[type].scale;
    glScalef(sc,sc,sc);
    if(type==PLANE_PROP){
        drawPropPlane(gear);
        drawPlaneNavLights(1.20f, 0.0f, 0.0f, -1.10f, 0.55f);
        glPopMatrix(); return;
    }
    if(type==PLANE_AIRLINER){
        drawAirliner(gear);
        drawPlaneNavLights(2.20f, 0.0f, -0.10f, -2.20f, 0.70f);
        glPopMatrix(); return;
    }

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

    // ---- WING FENCES (vortex fences at ~60% span) ----
    glColor3f(0.54f,0.55f,0.61f);
    for(int side=-1;side<=1;side+=2){
        float fx = side*0.93f;
        glBegin(GL_QUADS);
            glNormal3f(side,0,0);
            glVertex3f(fx, 0.0f,  0.02f); glVertex3f(fx, 0.0f, -0.32f);
            glVertex3f(fx, 0.06f,-0.32f); glVertex3f(fx, 0.06f, 0.02f);
        glEnd();
        // top cap of fence
        glBegin(GL_QUADS);
            glNormal3f(0,1,0);
            glVertex3f(fx-side*0.003f,0.06f, 0.02f); glVertex3f(fx+side*0.003f,0.06f, 0.02f);
            glVertex3f(fx+side*0.003f,0.06f,-0.32f);  glVertex3f(fx-side*0.003f,0.06f,-0.32f);
        glEnd();
    }

    // ---- UNDERWING STORES (2 missiles each side) ----
    glColor3f(0.40f,0.42f,0.48f);
    for(int side=-1;side<=1;side+=2){
        for(int w=0;w<2;w++){
            float sx = side*(0.50f + w*0.45f);
            glPushMatrix(); glTranslatef(sx, -0.045f, 0.0f);
            // missile body
            drawOctSegment(0.025f,0.025f, 0.28f, 0.025f,0.025f,-0.28f);
            drawOctCap(0.025f,0.025f, 0.28f, 0,0,1);
            // cone nose
            drawOctSegment(0.025f,0.025f, 0.28f, 0.0f,0.0f, 0.38f);
            drawOctCap(0.0f,0.0f, 0.38f, 0,0,1);
            // small fins
            glColor3f(0.38f,0.40f,0.46f);
            for(int f2=0;f2<4;f2++){
                glPushMatrix(); glRotatef(f2*90.0f, 0,0,1);
                glBegin(GL_QUADS); glNormal3f(0,1,0);
                    glVertex3f(-0.005f, 0.025f,-0.20f); glVertex3f( 0.005f, 0.025f,-0.20f);
                    glVertex3f( 0.005f, 0.07f, -0.26f); glVertex3f(-0.005f, 0.07f, -0.26f);
                glEnd();
                glPopMatrix();
            }
            glColor3f(0.40f,0.42f,0.48f);
            glPopMatrix();
        }
    }

    // ---- EXHAUST NOZZLES (twin) ----
    for(int side=-1;side<=1;side+=2){
        float nx2 = side*0.10f;
        glPushMatrix(); glTranslatef(nx2, -0.02f, -1.30f);
        // outer nozzle ring
        glColor3f(0.28f,0.28f,0.30f);
        drawOctSegment(0.07f,0.06f, 0.0f, 0.07f,0.06f,-0.10f);
        // petals (slightly darker flaps)
        glColor3f(0.22f,0.22f,0.24f);
        for(int p=0;p<8;p++){
            float a0=p*3.14159f*2/8, a1=(p+1)*3.14159f*2/8;
            glBegin(GL_QUADS); glNormal3f(0,0,-1);
                glVertex3f(cosf(a0)*0.065f, sinf(a0)*0.06f, -0.10f);
                glVertex3f(cosf(a1)*0.065f, sinf(a1)*0.06f, -0.10f);
                glVertex3f(cosf(a1)*0.045f, sinf(a1)*0.04f, -0.14f);
                glVertex3f(cosf(a0)*0.045f, sinf(a0)*0.04f, -0.14f);
            glEnd();
        }
        // afterburner glow (additive orange disc)
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glDisable(GL_LIGHTING);
        glColor4f(1.0f,0.5f,0.05f,0.45f);
        glBegin(GL_TRIANGLE_FAN);
            glVertex3f(0,0,-0.12f);
            for(int i=0;i<=12;i++){ float a=i*3.14159f*2/12;
                glVertex3f(cosf(a)*0.05f, sinf(a)*0.04f, -0.12f); }
        glEnd();
        glEnable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glPopMatrix();
    }

    // ---- PANEL LINES (fine surface detail) ----
    glColor3f(0.50f,0.51f,0.57f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
        // fuselage access panels
        glVertex3f(-0.23f, 0.0f, 0.60f); glVertex3f( 0.23f, 0.0f, 0.60f);
        glVertex3f(-0.23f, 0.0f, 0.10f); glVertex3f( 0.23f, 0.0f, 0.10f);
        glVertex3f(-0.23f, 0.0f,-0.40f); glVertex3f( 0.23f, 0.0f,-0.40f);
        // wing panel joints
        glVertex3f( 0.23f, 0.03f, 0.00f); glVertex3f( 0.85f, 0.01f,-0.40f);
        glVertex3f(-0.23f, 0.03f, 0.00f); glVertex3f(-0.85f, 0.01f,-0.40f);
        // leading edge seam
        glVertex3f( 0.23f, 0.03f, 0.12f); glVertex3f( 1.55f,-0.01f,-0.28f);
        glVertex3f(-0.23f, 0.03f, 0.12f); glVertex3f(-1.55f,-0.01f,-0.28f);
    glEnd();

    // ---- VHF ANTENNA (dorsal blade) ----
    glColor3f(0.30f,0.30f,0.33f);
    glBegin(GL_QUADS);
        glNormal3f(1,0,0);
        glVertex3f( 0.004f, 0.22f, 0.30f); glVertex3f( 0.004f, 0.22f,-0.10f);
        glVertex3f( 0.004f, 0.32f,-0.02f); glVertex3f( 0.004f, 0.32f, 0.20f);
        glNormal3f(-1,0,0);
        glVertex3f(-0.004f, 0.22f, 0.30f); glVertex3f(-0.004f, 0.32f, 0.20f);
        glVertex3f(-0.004f, 0.32f,-0.02f); glVertex3f(-0.004f, 0.22f,-0.10f);
        glNormal3f(0,1,0);
        glVertex3f(-0.004f, 0.32f, 0.20f); glVertex3f( 0.004f, 0.32f, 0.20f);
        glVertex3f( 0.004f, 0.32f,-0.02f); glVertex3f(-0.004f, 0.32f,-0.02f);
    glEnd();

    // ---- INTAKE LIPS (beveled forward edge of engine inlets) ----
    glColor3f(0.60f,0.61f,0.67f);
    for(int side=-1;side<=1;side+=2){
        float bx = side*0.72f;
        glPushMatrix(); glTranslatef(bx, -0.10f, 0.46f);
        drawOctSegment(0.11f,0.09f, 0.0f, 0.09f,0.07f,-0.05f);
        drawOctCap(0.11f,0.09f, 0.0f, 0,0,1);
        glPopMatrix();
    }

    // ---- NAV / STROBE / BEACON LIGHTS ----
    drawPlaneNavLights(1.55f, -0.05f, -0.28f, -1.30f, 0.32f);

    glPopMatrix();
}

// -------- Terrain --------
#define TN (TERRAIN_SIZE+1)
static float terrainH[TN][TN];
static float terrainNX[TN][TN];
static float terrainNY[TN][TN];
static float terrainNZ[TN][TN];
// Pre-baked per-vertex colors and biome index (computed once after terrain gen)
static unsigned char terrainCR[TN][TN];
static unsigned char terrainCG[TN][TN];
static unsigned char terrainCB[TN][TN];
static unsigned char terrainBiome[TN][TN]; // 0=water 1=sand 2=grass 3=highland 4=rock 5=snow


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
    terrainH[0][0]       = TERRAIN_HEIGHT*0.40f;
    terrainH[0][TN-1]    = TERRAIN_HEIGHT*0.55f;
    terrainH[TN-1][0]    = TERRAIN_HEIGHT*0.50f;
    terrainH[TN-1][TN-1] = TERRAIN_HEIGHT*0.60f;

    // Higher roughness = dramatic peaks and deep valleys, jurassic style
    float roughness = 0.68f, scale = TERRAIN_HEIGHT;
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
static void renderNpcs(); // forward declaration
static void updateNpcs(float dt); // forward declaration
static void renderMissiles(); // forward declaration
static void updateMissiles(float dt); // forward declaration
static void fireMissile(); // forward declaration
static void lanBroadcastPresence(); // forward declaration
static void updateLanServers(float dt); // forward declaration
static bool voiceInit(); // forward declaration
static void voiceStart(); // forward declaration
static void voiceReceive(const char *buf, int n); // forward declaration
static void menuRect(float x,float y,float w,float h,float r,float g,float b,float a); // forward decl
static void menuRectOutline(float x,float y,float w,float h,float r,float g,float b); // forward decl

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
            // Regional mountain mask — low-frequency noise decides where mountains appear.
            // mountainMask near 0 = flat plains/valleys, near 1 = full mountain range.
            float mountainMask = fbm(fx*1.1f + 7.7f, fz*1.1f + 3.3f, 3);
            mountainMask = clampf((mountainMask - 0.35f) * 2.5f, 0.0f, 1.0f);
            mountainMask = mountainMask * mountainMask; // sharpen transition

            // Jurassic ridge spines, only in mountain regions
            float ridge = fbm(fx*3.5f, fz*3.5f, 6);
            ridge = 1.0f - fabsf(ridge * 2.0f - 1.0f);
            ridge = ridge * ridge * ridge;
            float ridge2 = fbm(fx*2.1f + 5.3f, fz*1.7f + 2.9f, 4);
            ridge2 = 1.0f - fabsf(ridge2);
            ridge2 = ridge2 * ridge2 * 0.4f;

            // Gentle rolling hills everywhere (low amplitude)
            float rolling = fbm(fx*2.0f + 1.1f, fz*2.0f + 8.5f, 4);
            rolling = rolling * rolling * 0.3f;

            // Fine surface texture everywhere
            float detail = fbm(fx*18.0f, fz*18.0f, 4) * 0.6f;

            terrainH[z][x] += mountainMask * (ridge * TERRAIN_HEIGHT * 0.65f
                                            + ridge2 * TERRAIN_HEIGHT * 0.25f)
                            + (1.0f - mountainMask) * rolling * TERRAIN_HEIGHT * 0.18f
                            + detail * 4.0f;
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

// Bake terrain colors into byte arrays so renderTerrain() avoids per-vertex recalc.
// Called once at the end of generateAllTerrain() and after any re-flattening.
static void bakeTerrainColors(){
    for(int z=0;z<TN;z++)
        for(int x=0;x<TN;x++){
            float h2 = terrainH[z][x];
            float ny = terrainNY[z][x];
            float slope = 1.0f - ny;
            float t = h2 / TERRAIN_HEIGHT;

            // Spatial biome selector: low-frequency noise to create region variety
            float wx2 = (x - TERRAIN_SIZE*0.5f) * TERRAIN_SCALE;
            float wz2 = (z - TERRAIN_SIZE*0.5f) * TERRAIN_SCALE;
            float bx = wx2 / (TERRAIN_SIZE * TERRAIN_SCALE * 0.5f); // -1..1
            float bz = wz2 / (TERRAIN_SIZE * TERRAIN_SCALE * 0.5f);
            float biomeNoise = fbm(bx*1.8f + 3.3f, bz*1.8f + 1.7f, 3); // 0..1

            float r,g,b;
            int biome;

            if(t < 0.03f){
                // Water
                r=0.15f; g=0.28f; b=0.62f; biome=0;
            } else if(t < 0.07f){
                // Shoreline — sand or muddy wetland by region
                if(biomeNoise > 0.55f){ r=0.55f; g=0.60f; b=0.45f; biome=6; } // wetland shore
                else                  { r=0.76f; g=0.70f; b=0.50f; biome=1; } // sand shore
            } else if(t < 0.12f){
                // Low flat — desert, wetland, or grass by region
                if(biomeNoise < 0.28f){      r=0.80f; g=0.68f; b=0.42f; biome=1; } // desert/sand
                else if(biomeNoise < 0.48f){ r=0.48f; g=0.58f; b=0.34f; biome=6; } // wetland
                else {                       r=0.22f; g=0.50f; b=0.16f; biome=2; } // grass
            } else if(t < 0.38f){
                // Mid elevations — grass, canyon, or tundra
                if(biomeNoise < 0.30f){      r=0.72f; g=0.42f; b=0.22f; biome=7; } // canyon/desert
                else if(biomeNoise > 0.72f){ r=0.50f; g=0.55f; b=0.42f; biome=5; } // tundra
                else {                       r=0.22f; g=0.50f; b=0.16f; biome=2; } // grass
            } else if(t < 0.62f){
                // Highland — grass fading to rock, or canyon walls
                if(biomeNoise < 0.32f){ r=0.65f; g=0.38f; b=0.20f; biome=7; } // canyon wall
                else {                  r=0.28f; g=0.43f; b=0.18f; biome=2; } // highland grass
            } else if(t < 0.80f){
                // Rock zone
                r=0.50f; g=0.46f; b=0.38f; biome=3;
            } else {
                // Snow caps
                r=0.88f; g=0.90f; b=0.93f; biome=4;
            }

            // Steep slopes always become rock regardless of biome
            if(slope > 0.30f){
                float blend = clampf((slope-0.30f)/0.35f, 0.0f, 1.0f);
                r=r*(1-blend)+0.48f*blend;
                g=g*(1-blend)+0.44f*blend;
                b=b*(1-blend)+0.37f*blend;
                if(blend > 0.5f) biome=3;
            }

            terrainCR[z][x]=(unsigned char)(r*255);
            terrainCG[z][x]=(unsigned char)(g*255);
            terrainCB[z][x]=(unsigned char)(b*255);
            terrainBiome[z][x]=(unsigned char)biome;
        }
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
        pp->respawnTimer = 0.0f;
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
            if(sqrtf(dx*dx+dz*dz)<220.0f){ tooClose=true; break; }
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

static void spawnNpcsForAirports(){
    numNpcs = 0;
    for(int ai=0;ai<numAirports && numNpcs<MAX_NPCS;ai++){
        Airport *ap = &airports[ai];
        int count = 2 + ap->size * 2; // 2-6 NPCs per airport
        float twW    = 3.5f;
        float twCenZ = ap->runwayW + 2.0f + twW;
        float apronZ0 = twCenZ + twW;
        float apronHalfX = (2 + ap->size*2 - 1) * 8.0f * 0.5f + 5.0f;
        for(int n=0;n<count && numNpcs<MAX_NPCS;n++){
            Npc *npc = &npcs[numNpcs++];
            memset(npc, 0, sizeof(*npc));
            // Scatter across apron
            float lx = ((float)rand()/RAND_MAX - 0.5f) * apronHalfX * 2.0f;
            float lz = apronZ0 + ((float)rand()/RAND_MAX) * 10.0f;
            float wx, wz;
            apLocalToWorld(ap, lx, lz, &wx, &wz);
            npc->x = wx;
            npc->z = wz;
            npc->y = ap->groundY;
            npc->yaw = ((float)rand()/RAND_MAX) * 6.2832f;
            npc->homeX = wx; npc->homeZ = wz;
            npc->airportIdx = ai;
            npc->state = NPC_WALK;
            npc->wanderTimer = 1.0f + ((float)rand()/RAND_MAX) * 3.0f;
            npc->wanderYaw   = npc->yaw;
            npc->stateTimer  = 8.0f + ((float)rand()/RAND_MAX) * 20.0f; // walk for a while before boarding
            npc->alive = true;
        }
    }
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
    spawnNpcsForAirports();
    bakeTerrainColors();
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
                glVertex3f(fwd,0,sz); glVertex3f(fwd+0.06f,0,sz);
                glVertex3f(fwd+0.06f,0.5f,sz); glVertex3f(fwd,0.5f,sz);
            glEnd();
            // light lens (solid yellow)
            glColor3f(1.0f,0.92f,0.25f);
            glBegin(GL_QUADS);
                glNormal3f(0,1,0);
                glVertex3f(fwd-0.08f,0.52f,sz-0.08f); glVertex3f(fwd+0.14f,0.52f,sz-0.08f);
                glVertex3f(fwd+0.14f,0.52f,sz+0.08f); glVertex3f(fwd-0.08f,0.52f,sz+0.08f);
            glEnd();
            // glow halo (additive blend)
            glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            glDisable(GL_LIGHTING);
            glColor4f(1.0f, 0.85f, 0.1f, 0.35f);
            float hr = 0.5f;
            glBegin(GL_TRIANGLE_FAN);
                glVertex3f(fwd+0.03f, 0.55f, sz);
                for(int gi=0;gi<=8;gi++){ float ga=gi*6.28318f/8;
                    glVertex3f(fwd+0.03f+cosf(ga)*hr, 0.55f+sinf(ga)*hr*0.4f, sz+sinf(ga)*hr); }
            glEnd();
            glEnable(GL_LIGHTING); glDisable(GL_BLEND);
        }
    }

    // ---- Threshold lights (red at each runway end) ----
    for(int end=-1;end<=1;end+=2){
        for(int s=-1;s<=1;s+=2){
            float sz = s*(rw*0.7f);
            float fx = end*rl;
            glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            glDisable(GL_LIGHTING);
            glColor4f(1.0f, 0.1f, 0.05f, 0.6f);
            glBegin(GL_TRIANGLE_FAN);
                glVertex3f(fx, 0.6f, sz);
                for(int gi=0;gi<=8;gi++){ float ga=gi*6.28318f/8;
                    glVertex3f(fx+cosf(ga)*0.5f, 0.6f+sinf(ga)*0.3f, sz+sinf(ga)*0.5f); }
            glEnd();
            glEnable(GL_LIGHTING); glDisable(GL_BLEND);
        }
    }

    // ---- PAPI (Precision Approach Path Indicator) — left side of threshold ----
    // Calibration angles (degrees): light 0=outermost=highest, light 3=innermost=lowest.
    // White when above calibration angle, red when below.
    // Standard 3-deg path: all white=too high, 2W2R=on glidepath, all red=too low.
    {
        static const float papiAngles[4] = { 4.5f, 3.5f, 2.5f, 2.0f };
        float papiX = rl - 5.0f;
        float papiZ = -(rw + 3.5f);

        // Transform player world pos into airport local coords to get elevation angle.
        // Airport local: c = cos(heading), s = sin(heading)
        // world->local: fwd = c*(px-apx) - s*(pz-apz), but we only need horizontal
        // distance from the PAPI threshold position to the player.
        // Convert PAPI local position to world for the distance calc.
        float ch = cosf(ap->heading), sh = sinf(ap->heading);
        // PAPI centre local = (papiX, 0.5, papiZ)
        float papiWX = ap->wx + ch*papiX + sh*papiZ;
        float papiWZ = ap->wz - sh*papiX + ch*papiZ;
        float papiWY = ap->groundY + 0.5f;

        float dX = player.x - papiWX;
        float dZ = player.z - papiWZ;
        float horizDist = sqrtf(dX*dX + dZ*dZ);
        float elevAngleDeg = 0.0f;
        if(horizDist > 5.0f){
            elevAngleDeg = atan2f(player.y - papiWY, horizDist) * 57.2958f;
        }

        for(int p=0;p<4;p++){
            float pz2 = papiZ - p*2.2f;
            // White if player is above this light's calibration angle, red if below.
            int isWhite = (elevAngleDeg >= papiAngles[p]);
            float pr = isWhite ? 0.95f : 1.0f;
            float pg = isWhite ? 0.95f : 0.05f;
            float pb = isWhite ? 0.90f : 0.05f;
            glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            glDisable(GL_LIGHTING);
            glColor4f(pr, pg, pb, 0.85f);
            glBegin(GL_TRIANGLE_FAN);
                glVertex3f(papiX, 0.5f, pz2);
                for(int gi=0;gi<=8;gi++){ float ga=gi*6.28318f/8;
                    glVertex3f(papiX+cosf(ga)*0.4f, 0.5f+sinf(ga)*0.25f, pz2+sinf(ga)*0.4f); }
            glEnd();
            glEnable(GL_LIGHTING); glDisable(GL_BLEND);
        }
    }

    // ---- Taxiway edge lights (blue) ----
    {
        int ntw = (int)(twEndX*2.0f / 10.0f);
        for(int l=0;l<=ntw;l++){
            float fx = -twEndX + l*(twEndX*2.0f/ntw);
            for(int side=0;side<2;side++){
                float sz = (side==0) ? twInner-0.3f : twOuter+0.3f;
                glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
                glDisable(GL_LIGHTING);
                glColor4f(0.15f, 0.35f, 1.0f, 0.45f);
                glBegin(GL_TRIANGLE_FAN);
                    glVertex3f(fx, 0.4f, sz);
                    for(int gi=0;gi<=6;gi++){ float ga=gi*6.28318f/6;
                        glVertex3f(fx+cosf(ga)*0.3f, 0.4f, sz+sinf(ga)*0.3f); }
                glEnd();
                glEnable(GL_LIGHTING); glDisable(GL_BLEND);
            }
        }
    }

    // ---- Apron flood lights (bright white on tall poles at apron corners) ----
    {
        float floodPoles[4][2] = {
            {-apronHalfX, apronZ0}, {apronHalfX, apronZ0},
            {-apronHalfX, apronZ1}, {apronHalfX, apronZ1}
        };
        for(int fp=0;fp<4;fp++){
            float fx = floodPoles[fp][0], fz = floodPoles[fp][1];
            // Pole
            glColor3f(0.4f,0.4f,0.4f);
            glBegin(GL_QUADS);
                glNormal3f(1,0,0);
                glVertex3f(fx,0,fz); glVertex3f(fx+0.12f,0,fz);
                glVertex3f(fx+0.12f,6.5f,fz); glVertex3f(fx,6.5f,fz);
            glEnd();
            // Flood glow
            glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            glDisable(GL_LIGHTING);
            glColor4f(1.0f, 0.97f, 0.88f, 0.30f);
            glBegin(GL_TRIANGLE_FAN);
                glVertex3f(fx+0.06f, 6.6f, fz);
                for(int gi=0;gi<=10;gi++){ float ga=gi*6.28318f/10;
                    glVertex3f(fx+0.06f+cosf(ga)*1.5f, 6.6f+sinf(ga)*0.5f, fz+sinf(ga)*1.5f); }
            glEnd();
            glEnable(GL_LIGHTING); glDisable(GL_BLEND);
        }
    }

    // ---- Tower rotating beacon (alternating white/green) ----
    if(ap->size >= 2){
        float tx2 = apronHalfX + 3.0f;
        float bz   = bldgZ + 2.0f;
        float beaconY = 9.2f;
        // Beacon rotates with world time
        float beaconAngle = fmodf((float)clock() / (float)CLOCKS_PER_SEC * 2.0f * 3.14159f * 0.5f, 3.14159f*2.0f);
        float bx2 = tx2 + cosf(beaconAngle)*2.5f;
        float bz2 = bz  + sinf(beaconAngle)*2.5f;
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glDisable(GL_LIGHTING);
        // White flash
        glColor4f(1.0f, 1.0f, 1.0f, 0.70f);
        glBegin(GL_TRIANGLE_FAN);
            glVertex3f(tx2, beaconY, bz);
            for(int gi=0;gi<=10;gi++){ float ga=gi*6.28318f/10;
                glVertex3f(tx2+cosf(ga)*2.0f, beaconY, bz+sinf(ga)*2.0f); }
        glEnd();
        // Green sweep in opposite direction
        glColor4f(0.0f, 1.0f, 0.2f, 0.50f);
        glBegin(GL_TRIANGLE_FAN);
            glVertex3f(tx2, beaconY, bz);
            for(int gi=0;gi<=10;gi++){ float ga=gi*6.28318f/10;
                glVertex3f(bx2+cosf(ga)*1.5f, beaconY, bz2+sinf(ga)*1.5f); }
        glEnd();
        glEnable(GL_LIGHTING); glDisable(GL_BLEND);
    }

    glPopMatrix();
    glEnable(GL_TEXTURE_2D);
}

void renderAllAirports(){
    float eyeX = inPlane ? player.x : walkerX;
    float eyeZ = inPlane ? player.z : walkerZ;
    for(int i=0;i<numAirports;i++){
        float dx=airports[i].wx-eyeX, dz=airports[i].wz-eyeZ;
        if(dx*dx+dz*dz < 1000.0f*1000.0f)
            renderAirport(&airports[i]);
    }
}

void renderTerrain(){
    float offX = -TERRAIN_SIZE*TERRAIN_SCALE*0.5f;
    float offZ = -TERRAIN_SIZE*TERRAIN_SCALE*0.5f;
    const float invTile = 1.0f/8.0f;

    glEnable(GL_TEXTURE_2D);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    int boundBiome = -1;
    for(int z=0;z<TERRAIN_SIZE;z++){
        int rb  = terrainBiome[z  ][TERRAIN_SIZE/2];
        int rb1 = terrainBiome[z+1][TERRAIN_SIZE/2];
        int rowBiome = rb1>rb ? rb1 : rb;
        if(rowBiome != boundBiome){
            glBindTexture(GL_TEXTURE_2D, biomeTex[rowBiome]);
            boundBiome = rowBiome;
        }
        glBegin(GL_TRIANGLE_STRIP);
        for(int x=0;x<=TERRAIN_SIZE;x++){
            float wx  = offX + x*TERRAIN_SCALE;
            float wz1 = offZ + (z+1)*TERRAIN_SCALE;
            float wz0 = offZ +  z   *TERRAIN_SCALE;
            float u   = wx * invTile;
            glNormal3f(terrainNX[z+1][x], terrainNY[z+1][x], terrainNZ[z+1][x]);
            glColor3ub(terrainCR[z+1][x], terrainCG[z+1][x], terrainCB[z+1][x]);
            glTexCoord2f(u, wz1*invTile); glVertex3f(wx, terrainH[z+1][x], wz1);
            glNormal3f(terrainNX[z][x], terrainNY[z][x], terrainNZ[z][x]);
            glColor3ub(terrainCR[z][x], terrainCG[z][x], terrainCB[z][x]);
            glTexCoord2f(u, wz0*invTile); glVertex3f(wx, terrainH[z][x], wz0);
        }
        glEnd();
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glDisable(GL_TEXTURE_2D);
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

// -------- Biome Detail Textures --------
// Each texture is 256×256, generated procedurally with layered noise for surface detail.
// They tile at a fine world-space scale and are modulated with the vertex biome color.
#define BTEX_SIZE 256

// Simple integer hash for deterministic noise (no srand dependency)
static float btexNoise(int x, int z, int seed){
    unsigned int h = (unsigned int)(x*1619 + z*31337 + seed*7919);
    h ^= h>>16; h *= 0x45d9f3b; h ^= h>>16;
    return (float)(h & 0xFFFF) / 65535.0f; // [0,1]
}

// Bilinear smooth noise on the integer grid
static float btexSmooth(float x, float z, int seed){
    int ix=(int)floorf(x), iz=(int)floorf(z);
    float fx=x-ix, fz=z-iz;
    fx=fx*fx*(3-2*fx); fz=fz*fz*(3-2*fz);
    float v00=btexNoise(ix,  iz,  seed);
    float v10=btexNoise(ix+1,iz,  seed);
    float v01=btexNoise(ix,  iz+1,seed);
    float v11=btexNoise(ix+1,iz+1,seed);
    return v00*(1-fx)*(1-fz)+v10*fx*(1-fz)+v01*(1-fx)*fz+v11*fx*fz;
}

// fBm: sum octaves of smooth noise, returns [0,1]
static float btexFbm(float x, float z, int octaves, int seed){
    float val=0, amp=0.5f, freq=1.0f, maxAmp=0;
    for(int i=0;i<octaves;i++){
        val += btexSmooth(x*freq, z*freq, seed+i*997)*amp;
        maxAmp += amp; amp*=0.5f; freq*=2.0f;
    }
    return val/maxAmp;
}

static void generateBiomeTextures(){
    glGenTextures(NUM_BIOME_TEX, biomeTex);
    static GLubyte buf[BTEX_SIZE][BTEX_SIZE][3];

    // Helper: upload and set wrap/filter
#define UPLOAD_TEX(idx) \
    glBindTexture(GL_TEXTURE_2D, biomeTex[idx]); \
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR); \
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR); \
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT); \
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT); \
    gluBuild2DMipmaps(GL_TEXTURE_2D,GL_RGB,BTEX_SIZE,BTEX_SIZE,GL_RGB,GL_UNSIGNED_BYTE,buf);

    // 0 — Water: deep ocean colour with surface ripple/caustic variation
    for(int z=0;z<BTEX_SIZE;z++) for(int x=0;x<BTEX_SIZE;x++){
        float fx=x/(float)BTEX_SIZE*10.0f, fz=z/(float)BTEX_SIZE*10.0f;
        float coarse = btexFbm(fx,fz,4,1001);
        // Directional ripple pattern
        float rip1 = 0.5f+0.5f*sinf(fx*3.1f + coarse*5.0f);
        float rip2 = 0.5f+0.5f*sinf(fz*2.7f + coarse*4.2f + 1.0f);
        float ripple = rip1*0.55f + rip2*0.45f;
        // Shallow highlight from specular blobs
        float spec2 = btexSmooth(fx*6,fz*6,1099);
        float highlight = (spec2 > 0.78f) ? (spec2-0.78f)*3.5f : 0.0f;
        float depth = 0.30f + coarse*0.10f + ripple*0.08f;
        buf[z][x][0]=(unsigned char)clampf((depth*0.30f + highlight*0.40f)*255,0,255);
        buf[z][x][1]=(unsigned char)clampf((depth*0.55f + highlight*0.50f)*255,0,255);
        buf[z][x][2]=(unsigned char)clampf((depth*0.95f + highlight*0.70f)*255,0,255);
    }
    UPLOAD_TEX(0)

    // 1 — Sand/Dirt: warm dune texture with fine grain and shadow pockets
    for(int z=0;z<BTEX_SIZE;z++) for(int x=0;x<BTEX_SIZE;x++){
        float fx=x/(float)BTEX_SIZE*20.0f, fz=z/(float)BTEX_SIZE*20.0f;
        float macro = btexFbm(fx*0.5f,fz*0.5f,5,2001); // large dune shape
        float grain = btexFbm(fx,fz,6,2003);             // fine grain
        float pebble= btexFbm(fx*2,fz*2,4,2099);         // medium pebble
        // Small shadow pockets (darkening in hollows)
        float pocket = (pebble < 0.38f) ? (0.38f-pebble)*0.6f : 0.0f;
        float v = 0.68f + macro*0.18f + grain*0.10f - pocket;
        // Warm sandy colour with slight reddish tint at lower spots
        float rTint = 1.0f + macro*0.08f;
        buf[z][x][0]=(unsigned char)clampf(v*rTint*195,0,255);
        buf[z][x][1]=(unsigned char)clampf(v*168,0,255);
        buf[z][x][2]=(unsigned char)clampf(v*112,0,255);
    }
    UPLOAD_TEX(1)

    // 2 — Grass: multi-layer with micro-blades, dry patches, and soil showing through
    for(int z=0;z<BTEX_SIZE;z++) for(int x=0;x<BTEX_SIZE;x++){
        float fx=x/(float)BTEX_SIZE*14.0f, fz=z/(float)BTEX_SIZE*14.0f;
        float macro = btexFbm(fx*0.4f,fz*0.4f,4,3001);   // large patches
        float mid   = btexFbm(fx,fz,5,3007);              // medium texture
        // Fine grass blade streaks — anisotropic (taller in z)
        float blade = btexSmooth(fx*10,fz*2.5f,3101)*btexSmooth(fx*10,fz*2.8f,3203)*0.22f;
        // Bare soil shows through in sparse patches
        float sparse= btexFbm(fx*1.5f,fz*1.5f,4,3301);
        float soilAmt= (sparse < 0.32f) ? (0.32f-sparse)*1.2f : 0.0f;
        float v = 0.62f + macro*0.20f + mid*0.12f + blade;
        // Lerp between green and brown (soil)
        float green_r = v*72,  green_g = v*145, green_b = v*50;
        float soil_r  = v*110, soil_g  = v*88,  soil_b  = v*55;
        buf[z][x][0]=(unsigned char)clampf(green_r*(1-soilAmt)+soil_r*soilAmt,0,255);
        buf[z][x][1]=(unsigned char)clampf(green_g*(1-soilAmt)+soil_g*soilAmt,0,255);
        buf[z][x][2]=(unsigned char)clampf(green_b*(1-soilAmt)+soil_b*soilAmt,0,255);
    }
    UPLOAD_TEX(2)

    // 3 — Rock: weathered granite with cracks, lichen stains, and facets
    for(int z=0;z<BTEX_SIZE;z++) for(int x=0;x<BTEX_SIZE;x++){
        float fx=x/(float)BTEX_SIZE*12.0f, fz=z/(float)BTEX_SIZE*12.0f;
        float base  = btexFbm(fx,fz,7,4013);
        // Ridge lines — invert fBm for ridge-like crags
        float ridge = 1.0f - fabsf(btexFbm(fx*1.5f,fz*1.5f,5,4099)*2-1);
        ridge = ridge*ridge; // sharpen
        // Crack network
        float crack1 = fabsf(btexFbm(fx*3,fz*0.5f,4,4201)*2-1);
        float crack2 = fabsf(btexFbm(fx*0.5f,fz*3,4,4301)*2-1);
        float crack  = 1.0f - clampf((crack1<crack2?crack1:crack2)*4.0f,0,1);
        // Lichen: greenish patches at lower spots
        float lichen = btexFbm(fx*0.7f,fz*0.7f,4,4401);
        float lichenAmt = (lichen > 0.62f) ? (lichen-0.62f)*1.5f : 0.0f;
        float v = 0.42f + base*0.32f + ridge*0.14f - crack*0.15f;
        float lr = v*188, lg = v*178, lb = v*165; // grey stone
        float lichenR=v*80, lichenG=v*140, lichenB=v*70;
        buf[z][x][0]=(unsigned char)clampf(lr*(1-lichenAmt)+lichenR*lichenAmt,0,255);
        buf[z][x][1]=(unsigned char)clampf(lg*(1-lichenAmt)+lichenG*lichenAmt,0,255);
        buf[z][x][2]=(unsigned char)clampf(lb*(1-lichenAmt)+lichenB*lichenAmt,0,255);
    }
    UPLOAD_TEX(3)

    // 4 — Snow: compressed ice with subsurface blue tint, sparkle, and wind-blown ridges
    for(int z=0;z<BTEX_SIZE;z++) for(int x=0;x<BTEX_SIZE;x++){
        float fx=x/(float)BTEX_SIZE*8.0f, fz=z/(float)BTEX_SIZE*8.0f;
        float base  = btexFbm(fx,fz,5,5003);
        // Wind-blown ridges (anisotropic in x)
        float windR = btexSmooth(fx*0.8f,fz*4.0f,5101)*0.15f;
        // Fine sparkle
        float sp = btexSmooth(fx*22,fz*22,5201);
        float sparkle = (sp>0.88f)?(sp-0.88f)*7.0f:0.0f;
        // Shadow between ridges
        float shadow = btexFbm(fx*2,fz*2,4,5301)*0.10f;
        float v = 0.80f + base*0.12f + windR - shadow;
        float vS = v + sparkle*0.22f;
        // Blue subsurface tint in deep areas
        float depthAmt = clampf(1.0f-base*2.0f, 0.0f, 0.3f);
        buf[z][x][0]=(unsigned char)clampf(vS*235*(1-depthAmt)+vS*180*depthAmt,0,255);
        buf[z][x][1]=(unsigned char)clampf(vS*240*(1-depthAmt)+vS*210*depthAmt,0,255);
        buf[z][x][2]=(unsigned char)clampf(vS*255,0,255);
    }
    UPLOAD_TEX(4)

    // 5 — Tundra: pale grey-green mossy permafrost with frost crack patterns
    for(int z=0;z<BTEX_SIZE;z++) for(int x=0;x<BTEX_SIZE;x++){
        float fx=x/(float)BTEX_SIZE*12.0f, fz=z/(float)BTEX_SIZE*12.0f;
        float base  = btexFbm(fx,fz,5,6001);
        float frost  = btexFbm(fx*3,fz*3,4,6101);
        float crackV = fabsf(btexFbm(fx*2,fz*0.5f,4,6201)*2-1);
        float crackH = fabsf(btexFbm(fx*0.5f,fz*2,4,6301)*2-1);
        float crack = 1.0f - clampf((crackV<crackH?crackV:crackH)*5.0f,0,1);
        float v = 0.52f + base*0.22f + frost*0.08f - crack*0.12f;
        buf[z][x][0]=(unsigned char)clampf(v*145,0,255);
        buf[z][x][1]=(unsigned char)clampf(v*158,0,255);
        buf[z][x][2]=(unsigned char)clampf(v*138,0,255);
    }
    UPLOAD_TEX(5)

    // 6 — Wetland/Marsh: dark muddy green with standing water patches
    for(int z=0;z<BTEX_SIZE;z++) for(int x=0;x<BTEX_SIZE;x++){
        float fx=x/(float)BTEX_SIZE*10.0f, fz=z/(float)BTEX_SIZE*10.0f;
        float mud   = btexFbm(fx,fz,5,7001);
        float water = btexFbm(fx*0.6f,fz*0.6f,4,7101);
        float algae = btexFbm(fx*2,fz*2,4,7201);
        float wetAmt= clampf((1.0f-water)*1.5f,0,1);
        float v = 0.38f + mud*0.18f + algae*0.12f;
        // Dark muddy green, puddles are darker blue-grey
        float dr=v*58, dg=v*82, db=v*44;
        float wr=v*42, wg=v*55, wb=v*68;
        buf[z][x][0]=(unsigned char)clampf(dr*(1-wetAmt)+wr*wetAmt,0,255);
        buf[z][x][1]=(unsigned char)clampf(dg*(1-wetAmt)+wg*wetAmt,0,255);
        buf[z][x][2]=(unsigned char)clampf(db*(1-wetAmt)+wb*wetAmt,0,255);
    }
    UPLOAD_TEX(6)

    // 7 — Canyon/Red-rock: deep red sandstone with layered strata lines
    for(int z=0;z<BTEX_SIZE;z++) for(int x=0;x<BTEX_SIZE;x++){
        float fx=x/(float)BTEX_SIZE*16.0f, fz=z/(float)BTEX_SIZE*16.0f;
        float base  = btexFbm(fx,fz,5,8001);
        // Horizontal strata banding
        float strata = 0.5f+0.5f*sinf(fz*8.0f + base*3.0f);
        float grain  = btexFbm(fx*4,fz*4,4,8101);
        float v = 0.55f + base*0.20f + strata*0.15f + grain*0.08f;
        // Red-orange sandstone palette
        buf[z][x][0]=(unsigned char)clampf(v*210,0,255);
        buf[z][x][1]=(unsigned char)clampf(v*110 + strata*20,0,255);
        buf[z][x][2]=(unsigned char)clampf(v*55,0,255);
    }
    UPLOAD_TEX(7)

#undef UPLOAD_TEX
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
        char buffer[512]; struct sockaddr_in from; socklen_t fromlen=sizeof(from); int n;
        while((n=recvfrom(sockfd,buffer,sizeof(buffer),0,(struct sockaddr*)&from,&fromlen))>0){
            if(n < (int)sizeof(int)) continue;
            int recvType; memcpy(&recvType, buffer, sizeof(int));
            if(recvType==MSG_CONNECT){
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
            } else if(recvType==MSG_TERRAIN_SEED && n==(int)(sizeof(int)+sizeof(unsigned int))){
                if(!isServer && !seedReceived){
                    unsigned int seed;
                    memcpy(&seed, buffer+sizeof(int), sizeof(unsigned int));
                    printf("[NET] Received terrain seed %u from server\n", seed);
                    generateAllTerrain(seed);
                    seedReceived = true;
                    pthread_mutex_lock(&playerMutex);
                    player.x=0; player.y=airports[0].groundY+1.0f; player.z=0;
                    player.pitch=0; player.yaw=0; player.roll=0;
                    pthread_mutex_unlock(&playerMutex);
                }
            } else if(recvType==MSG_TORNADO && n==(int)(sizeof(int)+sizeof(int)+MAX_TORNADOS*(int)(sizeof(float)*4))){
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
            } else if(recvType==MSG_PLAYER_DATA && n==(int)(sizeof(int)+sizeof(float)*14)){
                float rdata[14];
                memcpy(rdata, buffer+sizeof(int), sizeof(rdata));
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
            } else if(recvType==MSG_EXPLOSION && n==(int)(sizeof(int)+sizeof(float)*4)){
                float ed[4]; memcpy(ed, buffer+sizeof(int), sizeof(ed));
                receiveExplosion(ed[0],ed[1],ed[2],ed[3]);
                if(isServer && clientConnected)
                    sendto(sockfd,buffer,n,0,(struct sockaddr*)&otherAddr,sizeof(otherAddr));
            } else if(recvType==MSG_VOICE && n>(int)VOICE_HDR){
                voiceReceive(buffer, n);
                if(isServer && clientConnected)
                    sendto(sockfd,buffer,n,0,(struct sockaddr*)&otherAddr,sizeof(otherAddr));
            } else if(recvType==MSG_CHAT && n>=(int)(sizeof(int)+1)){
                char msgbuf[CHAT_MSG_LEN+16];
                int rawlen = n-sizeof(int);
                if(rawlen >= CHAT_MSG_LEN) rawlen = CHAT_MSG_LEN-1;
                char raw[CHAT_MSG_LEN]; memcpy(raw, buffer+sizeof(int), rawlen); raw[rawlen]='\0';
                snprintf(msgbuf, sizeof(msgbuf), "[remote] %s", raw);
                chatAddMsg(msgbuf);
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
            *fog=0.0003f; *rain=0.0f;
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
            *fog=0.0003f; *rain=0.0f; spd=0; break;
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
        weather.fogDensity    += (tFog  - weather.fogDensity)*sp*60;
        weather.rainIntensity += (tRain - weather.rainIntensity)*sp*60;
        weather.windX += (tWX - weather.windX)*sp*60;
        weather.windZ += (tWZ - weather.windZ)*sp*60;
        weather.transitionTimer = 20.0f + ((float)rand()/RAND_MAX)*80.0f;
    }
    // Smooth per-frame lerp toward current targets (sky colour driven by updateSunLight)
    float sp = dt * 0.15f;
    float tR,tG,tB,tFog,tRain,tWX,tWZ;
    weatherPreset(weather.type,&tR,&tG,&tB,&tFog,&tRain,&tWX,&tWZ);
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

        // Pre-compute t-dependent values shared across all particles of this explosion
        float one_minus_t = 1.0f - t;
        float rise_factor0 = (1.0f - one_minus_t*one_minus_t);
        float rise_factor1 = rise_factor0;

        glBegin(GL_QUADS);
        for(int p=0;p<EXPLOSION_PARTICLES;p++){
            float spd  = explosions[i].ps[p];
            int   type = explosions[i].pt[p];
            float dirX = explosions[i].px[p];
            float dirY = explosions[i].py[p];
            float dirZ = explosions[i].pz[p];

            float px, py, pz, sz;
            float r, g, b, a;

            if(type == 0){
                // ── Stem: rises quickly, smaller spread ──
                float rise = spd * rise_factor0 * 6.0f;
                px = cx + dirX * rise;
                py = cy + dirY * rise;
                pz = cz + dirZ * rise;
                float szScale = 0.5f + spd * 0.05f + t * 1.2f;
                sz = (0.4f + t*0.7f) * (1.0f - t*0.25f) * szScale;
                if(t < 0.2f){
                    float ft=t/0.2f; r=1.0f; g=0.9f-ft*0.5f; b=0.0f; a=1.0f;
                } else if(t < 0.5f){
                    float ft=(t-0.2f)/0.3f; r=0.9f-ft*0.6f; g=0.4f-ft*0.3f; b=0.0f; a=0.9f-ft*0.15f;
                } else {
                    float ft=(t-0.5f)/0.5f; r=0.22f-ft*0.18f; g=r*0.8f; b=r*0.7f; a=0.65f-ft*0.65f;
                }

            } else if(type == 1){
                // ── Cap: arc up then curl outward, tighter radius ──
                float rise   = spd * rise_factor1 * 5.0f;
                float tAdj   = (t > 0.2f) ? (t-0.2f)*(1.0f/0.8f) : 0.0f;
                float spread = (t > 0.2f) ? spd * tAdj * tAdj * 7.0f : 0.0f;
                px = cx + dirX * (rise + spread);
                py = cy + dirY * rise;
                pz = cz + dirZ * (rise + spread);
                float szScale = 0.7f + spd * 0.09f + t * 1.6f;
                sz = (0.5f + t*0.9f) * (1.0f - t*0.35f) * szScale;
                if(t < 0.25f){
                    float ft=t/0.25f; r=1.0f; g=0.7f-ft*0.3f; b=0.0f; a=1.0f;
                } else if(t < 0.55f){
                    float ft=(t-0.25f)/0.3f; r=1.0f-ft*0.7f; g=0.4f-ft*0.35f; b=0.0f; a=0.95f-ft*0.15f;
                } else {
                    float ft=(t-0.55f)/0.45f; r=0.22f-ft*0.18f; g=r*0.85f; b=r*0.75f; a=0.72f-ft*0.72f;
                }

            } else if(type == 2){
                // ── Base ring: low radial blast, tighter ──
                float ring = spd * rise_factor0 * 6.0f;
                px = cx + dirX * ring;
                py = cy + dirY * ring * 0.3f;
                pz = cz + dirZ * ring;
                sz = (0.6f + t*0.4f) * (1.0f - t*0.8f) * (0.5f + spd*0.06f);
                if(sz < 0.01f) sz = 0.01f;
                if(t < 0.1f){
                    float ft=t/0.1f; r=1.0f; g=1.0f; b=0.6f-ft*0.6f; a=1.0f;
                } else if(t < 0.3f){
                    float ft=(t-0.1f)/0.2f; r=1.0f-ft*0.4f; g=0.6f-ft*0.5f; b=0.0f; a=1.0f-ft*0.4f;
                } else {
                    float ft=(t-0.3f)/0.7f; r=0.25f-ft*0.25f; g=0.0f; b=0.0f; a=0.4f-ft*0.4f;
                }

            } else {
                // ── Dust: slow expanding ground cloud, persists full lifetime ──
                // Ease-in-out: accelerates then slows gently
                float eased = t*t*(3.0f-2.0f*t);
                float ring = spd * eased * 14.0f;
                px = cx + dirX * ring;
                py = cy + dirY * ring + 0.3f; // hugs ground
                pz = cz + dirZ * ring;
                // Large soft billow, grows over time
                sz = (1.5f + t * 4.0f) * (0.4f + spd * 0.3f);
                // Tan/brown dust colour fading out slowly
                float fade = 1.0f - t*t;
                r = 0.52f; g = 0.40f; b = 0.26f;
                a = 0.28f * fade;
            }

            glColor4f(r,g,b,a);
            glVertex3f(px-sz, py-sz, pz);
            glVertex3f(px+sz, py-sz, pz);
            glVertex3f(px+sz, py+sz, pz);
            glVertex3f(px-sz, py+sz, pz);
        }
        glEnd();
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
    // Pre-compute per-point angle steps to avoid repeated division inside the loop
    float invPts = 1.0f / pts;
    float twoPi  = 2.0f * (float)M_PI;
    for(int r=0; r<rings; r++){
        float frac  = (float)r/(rings-1);
        float frac1 = (r+1 < rings) ? (float)(r+1)/(rings-1) : 1.0f;
        float y     = gndY + frac  * maxH;
        float y1    = gndY + frac1 * maxH;
        float radius= 1.0f + frac *frac  * 9.0f;
        float r1    = 1.0f + frac1*frac1 * 9.0f;
        float phase = t->angle + frac  * 6.0f;
        float ph1   = t->angle + frac1 * 6.0f;
        float alpha = 0.12f + frac*0.30f;
        float rv = 0.20f + frac*0.35f;
        float gv = 0.18f + frac*0.28f;
        float bv = 0.18f + frac*0.28f;
        glColor4f(rv,gv,bv,alpha);
        glBegin(GL_TRIANGLE_STRIP);
        for(int p=0; p<=pts; p++){
            float step = (float)p * invPts * twoPi;
            float a    = phase + step;
            float a1   = ph1   + step;
            glVertex3f(t->x + cosf(a )*radius, y,  t->z + sinf(a )*radius);
            glVertex3f(t->x + cosf(a1)*r1,     y1, t->z + sinf(a1)*r1);
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

// -------- Retail / Fast Food Buildings --------
typedef enum { STORE_ALBERTSONS=0, STORE_ARBYS=1, STORE_LJS=2 } StoreType;
#define MAX_STORES 48
#define STORE_ENTER_DIST 8.0f

// Store interior is a rectangle in local space: X ±SW_HALF, Z -SD_BACK..SD_FRONT
#define SW_HALF  10.0f   // half-width
#define SD_BACK  14.0f   // depth behind counter
#define SD_FRONT  5.0f   // depth in front of counter (toward door)
#define SWALL_THICK 0.3f // collision margin from walls
#define SCEIL    4.0f    // ceiling height
#define SDOOR_Z  (SD_FRONT - 0.5f)   // door local Z
#define SCOUNTER_Z (-2.5f)            // front face of counter

typedef struct {
    float wx, wy, wz;
    float heading;
    StoreType type;
} Store;
static Store     stores[MAX_STORES];
static int       numStores    = 0;
static int       inStoreIdx   = -1;          // -1 = outside
static StoreType lastStoreType = STORE_ALBERTSONS; // type of last visited store
// Walker's position inside the store, in store-local coords
static float storeWalkerX = 0.0f;
static float storeWalkerZ = SDOOR_Z - 1.0f;
static float storeWalkerYaw = (float)M_PI;  // faces counter (toward -Z)

// Inventory: up to 5 slots per store type, count of each item held
#define INV_SLOTS 5
static int   inventory[INV_SLOTS] = {0,0,0,0,0};

// Active consumable effects
static float effectSpeed    = 0.0f;  // extra speed multiplier, decays
static float effectSpeedTimer = 0.0f;
static float effectHeal     = 0.0f;  // heal rate per second, decays
static float effectHealTimer  = 0.0f;
static float playerHealth   = 1.0f;  // 0..1

// Item effect types
typedef enum { EFF_NONE, EFF_HEAL, EFF_SPEED, EFF_HEALSPD } EffectType;
typedef struct {
    const char *label;        // menu display
    const char *buyMsg;       // cashier response on purchase
    const char *useMsg;       // message when consumed
    EffectType  effect;
    float       effectMag;    // strength (heal: hp/s, speed: multiplier)
    float       effectDur;    // seconds
} MenuItem;

static const MenuItem menuAlbertsons[INV_SLOTS] = {
    {"1) Deli Sandwich       $6.99",  "Here's your sandwich, freshly made!",
     "Ate the sandwich. Feeling better!",    EFF_HEAL,   0.05f, 8.0f},
    {"2) Rotisserie Chicken  $7.99",  "Hot rotisserie chicken — enjoy!",
     "Devoured the chicken. Full heal!",     EFF_HEAL,   0.12f, 12.0f},
    {"3) House Salad         $4.49",  "One salad coming right up!",
     "Ate the salad. Light but nourishing.", EFF_HEAL,   0.03f, 6.0f},
    {"4) Bakery Croissant    $2.29",  "Still warm from the oven!",
     "Croissant eaten. Sugar rush!",         EFF_SPEED,  0.30f, 5.0f},
    {"5) Store-Brand Cola    $1.49",  "Enjoy the fizz!",
     "Chugged the cola. Caffeine kick!",     EFF_SPEED,  0.50f, 8.0f},
};
static const MenuItem menuArbys[INV_SLOTS] = {
    {"1) Classic Beef n Cheddar  $5.79", "WE HAVE THE MEATS! Enjoy!",
     "Devoured the Beef n Cheddar!",         EFF_HEAL,   0.06f, 8.0f},
    {"2) Smokehouse Brisket      $7.29", "Smoked low and slow, just for you!",
     "Ate the brisket. Powerful recovery!",  EFF_HEAL,   0.15f, 15.0f},
    {"3) Curly Fries (Large)     $3.49", "Curly fries fresh out the fryer!",
     "Ate the curly fries. Carb rush!",      EFF_SPEED,  0.40f, 6.0f},
    {"4) Jamocha Shake           $3.99", "One Jamocha shake!",
     "Drank the shake. Brain freeze + speed!", EFF_SPEED, 0.60f, 10.0f},
    {"5) Market Fresh Wrap       $6.49", "Market Fresh — literally!",
     "Wrap consumed. Balanced boost!",       EFF_HEALSPD,0.04f, 10.0f},
};
static const MenuItem menuLJS[INV_SLOTS] = {
    {"1) Fish & Chips (3pc)   $8.99",  "Arrr, three golden fish pieces!",
     "Fish & Chips demolished. Sea power!", EFF_HEAL,   0.10f, 10.0f},
    {"2) Shrimp Basket        $7.49",  "Shrimp basket, fresh from the sea!",
     "Shrimp inhaled. Speed of a dolphin!", EFF_SPEED,  0.45f, 8.0f},
    {"3) Hush Puppies (6pc)   $2.79",  "Six hush puppies, aye aye cap'n!",
     "Hush puppies eaten. Mild recovery.",  EFF_HEAL,   0.03f, 5.0f},
    {"4) Coleslaw             $1.99",  "Classic coleslaw, nautical style!",
     "Coleslaw finished. Vitamins!",        EFF_HEAL,   0.02f, 4.0f},
    {"5) Root Beer Float      $2.49",  "Root beer float, ahoy!",
     "Float consumed. Maximum caffeine!",   EFF_SPEED,  0.70f, 12.0f},
};

static void initStores(){
    numStores = 0;
    float halfW = TERRAIN_SIZE * TERRAIN_SCALE * 0.5f;
    srand(terrainSeed ^ 0xF00DC0DE);
    for(int ai = 0; ai < numAirports && numStores < MAX_STORES - 3; ai++){
        float apx = airports[ai].wx, apz = airports[ai].wz;
        int nStores = 1 + rand() % 3;
        for(int s = 0; s < nStores && numStores < MAX_STORES; s++){
            float angle = ((float)rand()/RAND_MAX) * 6.28318f;
            float dist  = 80.0f + ((float)rand()/RAND_MAX) * 120.0f;
            float sx = apx + cosf(angle) * dist;
            float sz = apz + sinf(angle) * dist;
            sx = clampf(sx, -halfW + 20, halfW - 20);
            sz = clampf(sz, -halfW + 20, halfW - 20);
            float sy = terrainHeightAt(sx, sz);
            int gx2 = (int)((sx/TERRAIN_SCALE) + TERRAIN_SIZE*0.5f);
            int gz2 = (int)((sz/TERRAIN_SCALE) + TERRAIN_SIZE*0.5f);
            gx2 = gx2<0?0:(gx2>TERRAIN_SIZE?TERRAIN_SIZE:gx2);
            gz2 = gz2<0?0:(gz2>TERRAIN_SIZE?TERRAIN_SIZE:gz2);
            if(terrainBiome[gz2][gx2] == 0) continue;
            Store *st = &stores[numStores++];
            st->wx = sx; st->wy = sy; st->wz = sz;
            st->heading = ((float)rand()/RAND_MAX) * 6.28318f;
            st->type = (StoreType)(rand() % 3);
        }
    }
    // Flatten terrain under every store so buildings sit on level ground
    for(int i = 0; i < numStores; i++){
        Store *st = &stores[i];
        float elev = terrainHeightAt(st->wx, st->wz);
        st->wy = elev;
        flattenRect(st->wx, st->wz, st->heading,
                    (SD_FRONT + SD_BACK) * 0.5f + 4.0f,
                    SW_HALF + 4.0f,
                    elev, 6.0f);
    }
    bakeTerrainColors();
    srand((unsigned)time(NULL));
}

// Render a 3D sign panel with text above the store entrance.
// Called inside the store's local coordinate space (after push/translate/rotate).
// signZ: front face Z of building, signY: top of building, label: sign text.
// bgR/G/B: sign background, txR/G/B: letter colour.
static void drawStoreSign(float signZ, float signY, const char *label,
                          float bgR, float bgG, float bgB,
                          float txR, float txG, float txB){
    glDisable(GL_LIGHTING);
    // Sign backing panel (slightly in front of building face)
    float sw = 0; // compute pixel width so we can center
    for(const char *c=label;*c;c++) sw += glutBitmapWidth(GLUT_BITMAP_HELVETICA_18,*c);
    float worldW = sw * 0.018f; // approx world units per pixel at this scale

    // Background panel
    glColor3f(bgR, bgG, bgB);
    glNormal3f(0,0,1);
    glBegin(GL_QUADS);
        glVertex3f(-worldW*0.5f - 0.15f, signY + 0.05f, signZ + 0.08f);
        glVertex3f( worldW*0.5f + 0.15f, signY + 0.05f, signZ + 0.08f);
        glVertex3f( worldW*0.5f + 0.15f, signY + 0.65f, signZ + 0.08f);
        glVertex3f(-worldW*0.5f - 0.15f, signY + 0.65f, signZ + 0.08f);
    glEnd();

    // Text — project sign centre to screen coords, then draw in window space
    // so glRasterPos clipping can't silently swallow the text.
    {
        GLdouble mv[16], proj[16]; GLint vp[4];
        glGetDoublev(GL_MODELVIEW_MATRIX,  mv);
        glGetDoublev(GL_PROJECTION_MATRIX, proj);
        glGetIntegerv(GL_VIEWPORT, vp);
        GLdouble sx, sy, sz;
        if(gluProject(0.0, signY+0.36f, signZ+0.10f, mv, proj, vp, &sx, &sy, &sz) == GL_TRUE
           && sz > 0.0 && sz < 1.0){
            // compute pixel width of text
            float pw = 0;
            for(const char *c=label;*c;c++) pw += glutBitmapWidth(GLUT_BITMAP_HELVETICA_18,*c);
            // switch to window space
            glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
            glOrtho(0, vp[2], 0, vp[3], -1, 1);
            glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
            glDisable(GL_DEPTH_TEST);
            glColor3f(txR, txG, txB);
            glRasterPos2f((float)(sx - pw*0.5f), (float)sy);
            for(const char *c=label;*c;c++)
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
            glEnable(GL_DEPTH_TEST);
            glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix();
            glMatrixMode(GL_MODELVIEW);
        }
    }

    glEnable(GL_LIGHTING);
}

static void drawStoreFascia(float w, float h, float d,
                             float yr, float yg, float yb,
                             float fr, float fg, float fb){
    drawBox(0, 0, 0, w, h, d, yr, yg, yb);
    // Coloured fascia band
    glDisable(GL_LIGHTING);
    glColor3f(fr, fg, fb);
    glNormal3f(0,0,1);
    glBegin(GL_QUADS);
        glVertex3f(-w*0.85f, h*0.55f, d+0.02f);
        glVertex3f( w*0.85f, h*0.55f, d+0.02f);
        glVertex3f( w*0.85f, h*0.90f, d+0.02f);
        glVertex3f(-w*0.85f, h*0.90f, d+0.02f);
    glEnd();
    glEnable(GL_LIGHTING);
}

// Draw the interior of a store in store-local coordinates.
// Layout: door at Z=+SD_FRONT, counter at Z=SCOUNTER_Z, back wall at Z=-SD_BACK.
// Width: X = -SW_HALF .. +SW_HALF. Height: 0 .. SCEIL.
static void drawStoreInterior(StoreType type){
    float W = SW_HALF, D = SD_BACK, F = SD_FRONT, H = SCEIL;
    float CZ = SCOUNTER_Z; // counter front face

    // Floor — tiled appearance via colour alternation done with single quad
    glColor3f(0.74f, 0.72f, 0.68f);
    glNormal3f(0,1,0);
    glBegin(GL_QUADS);
        glVertex3f(-W,0,-D); glVertex3f(W,0,-D);
        glVertex3f(W,0, F);  glVertex3f(-W,0,F);
    glEnd();
    // Tile grid lines (darker strips)
    glColor3f(0.62f,0.60f,0.57f);
    for(float tz=-D; tz<F; tz+=2.0f){
        glBegin(GL_QUADS);
            glVertex3f(-W,0.005f,tz); glVertex3f(W,0.005f,tz);
            glVertex3f(W,0.005f,tz+0.06f); glVertex3f(-W,0.005f,tz+0.06f);
        glEnd();
    }
    for(float tx=-W; tx<W; tx+=2.0f){
        glBegin(GL_QUADS);
            glVertex3f(tx,0.005f,-D); glVertex3f(tx+0.06f,0.005f,-D);
            glVertex3f(tx+0.06f,0.005f,F); glVertex3f(tx,0.005f,F);
        glEnd();
    }

    // Ceiling with fluorescent light panels
    glColor3f(0.94f,0.94f,0.92f);
    glNormal3f(0,-1,0);
    glBegin(GL_QUADS);
        glVertex3f(-W,H,-D); glVertex3f(-W,H,F);
        glVertex3f(W,H,F);   glVertex3f(W,H,-D);
    glEnd();
    // Light panels (bright white insets)
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glColor4f(1.0f,0.98f,0.90f,0.5f);
    for(float lz=-D+2.0f; lz<F; lz+=4.0f){
        glBegin(GL_QUADS);
            glVertex3f(-W+1.0f,H-0.01f,lz);   glVertex3f(W-1.0f,H-0.01f,lz);
            glVertex3f(W-1.0f,H-0.01f,lz+1.2f); glVertex3f(-W+1.0f,H-0.01f,lz+1.2f);
        glEnd();
    }
    glDisable(GL_BLEND); glEnable(GL_LIGHTING);

    // Walls
    glColor3f(0.87f,0.86f,0.84f);
    // Back wall
    glNormal3f(0,0,1);
    glBegin(GL_QUADS);
        glVertex3f(-W,0,-D); glVertex3f(-W,H,-D);
        glVertex3f(W,H,-D);  glVertex3f(W,0,-D);
    glEnd();
    // Front wall (with door gap)
    glNormal3f(0,0,-1);
    glBegin(GL_QUADS); // left of door
        glVertex3f(-W,0,F); glVertex3f(-W,H,F);
        glVertex3f(-1.2f,H,F); glVertex3f(-1.2f,0,F);
    glEnd();
    glBegin(GL_QUADS); // right of door
        glVertex3f(1.2f,0,F); glVertex3f(1.2f,H,F);
        glVertex3f(W,H,F);    glVertex3f(W,0,F);
    glEnd();
    glBegin(GL_QUADS); // above door
        glVertex3f(-1.2f,2.3f,F); glVertex3f(-1.2f,H,F);
        glVertex3f(1.2f,H,F);     glVertex3f(1.2f,2.3f,F);
    glEnd();
    // Side walls
    for(int side=-1;side<=1;side+=2){
        float sx = side*W;
        glColor3f(0.85f,0.84f,0.82f);
        glNormal3f(-side,0,0);
        glBegin(GL_QUADS);
            glVertex3f(sx,0,-D); glVertex3f(sx,H,-D);
            glVertex3f(sx,H, F); glVertex3f(sx,0, F);
        glEnd();
    }

    // Brand colours for counter/header
    float ctr, ctg, ctb;
    if(type==STORE_ALBERTSONS){ ctr=0.12f; ctg=0.28f; ctb=0.70f; }
    else if(type==STORE_ARBYS){ ctr=0.80f; ctg=0.10f; ctb=0.10f; }
    else                       { ctr=0.22f; ctg=0.40f; ctb=0.72f; }

    // Service counter — spans full width, sits at CZ
    float ctW = W - 0.5f; // counter half-width
    drawBox(0, 0, CZ - 0.6f,  ctW, 1.1f, 0.6f,  ctr*0.55f, ctg*0.55f, ctb*0.55f);
    // Counter top (marble-ish white)
    glColor3f(0.90f,0.88f,0.84f);
    glNormal3f(0,1,0);
    glBegin(GL_QUADS);
        glVertex3f(-ctW,1.1f,CZ-1.2f); glVertex3f(ctW,1.1f,CZ-1.2f);
        glVertex3f(ctW,1.1f,CZ+0.05f); glVertex3f(-ctW,1.1f,CZ+0.05f);
    glEnd();
    // Cash registers
    drawBox(-ctW*0.5f, 1.1f, CZ-0.7f,  0.45f, 0.40f, 0.28f,  0.12f,0.12f,0.15f);
    drawBox( ctW*0.5f, 1.1f, CZ-0.7f,  0.45f, 0.40f, 0.28f,  0.12f,0.12f,0.15f);
    // Screens on registers
    glColor3f(0.0f,0.5f,1.0f);
    glNormal3f(0,0,1);
    glBegin(GL_QUADS);
        glVertex3f(-ctW*0.5f-0.3f,1.5f,CZ-0.43f); glVertex3f(-ctW*0.5f+0.3f,1.5f,CZ-0.43f);
        glVertex3f(-ctW*0.5f+0.3f,1.8f,CZ-0.43f); glVertex3f(-ctW*0.5f-0.3f,1.8f,CZ-0.43f);
        glVertex3f( ctW*0.5f-0.3f,1.5f,CZ-0.43f); glVertex3f( ctW*0.5f+0.3f,1.5f,CZ-0.43f);
        glVertex3f( ctW*0.5f+0.3f,1.8f,CZ-0.43f); glVertex3f( ctW*0.5f-0.3f,1.8f,CZ-0.43f);
    glEnd();

    // Menu board on back wall — big dark panel
    drawBox(0, 2.2f, -D+0.06f,  W*0.75f, 1.0f, 0.05f,  0.07f,0.07f,0.09f);
    // Header colour bar
    glDisable(GL_LIGHTING);
    glColor3f(ctr, ctg, ctb);
    glNormal3f(0,0,1);
    glBegin(GL_QUADS);
        glVertex3f(-W*0.75f,3.0f,-D+0.12f); glVertex3f(W*0.75f,3.0f,-D+0.12f);
        glVertex3f(W*0.75f,3.4f,-D+0.12f);  glVertex3f(-W*0.75f,3.4f,-D+0.12f);
    glEnd();
    glEnable(GL_LIGHTING);

    // Store-specific fill
    if(type==STORE_ALBERTSONS){
        // Three long aisle shelving rows between counter and door
        float aisle_zs[] = {-8.0f, -4.0f, 0.0f};
        for(int ai=0;ai<3;ai++){
            float az = aisle_zs[ai];
            // Shelf unit (left side, right side)
            for(int side=-1;side<=1;side+=2){
                float shx = side * 6.5f;
                for(int shelf=0;shelf<4;shelf++){
                    float sy2 = 0.3f + shelf*0.65f;
                    drawBox(shx, sy2, az,  1.0f, 0.04f, 4.5f,  0.72f,0.70f,0.68f);
                }
                // Upright supports
                drawBox(shx, 0, az-4.5f,  0.08f, 2.9f, 0.08f,  0.60f,0.58f,0.55f);
                drawBox(shx, 0, az+4.5f,  0.08f, 2.9f, 0.08f,  0.60f,0.58f,0.55f);
                // Products on each shelf
                for(int shelf=0;shelf<4;shelf++){
                    float sy2 = 0.35f + shelf*0.65f;
                    for(int p=0;p<10;p++){
                        float pz2 = az - 4.0f + p*0.88f;
                        float pr2=(p%4==0)?0.85f:(p%4==1)?0.15f:(p%4==2)?0.90f:0.50f;
                        float pg2=(p%4==0)?0.15f:(p%4==1)?0.65f:(p%4==2)?0.80f:0.30f;
                        float pb2=(p%4==0)?0.15f:(p%4==1)?0.20f:(p%4==2)?0.10f:0.85f;
                        drawBox(shx+(side)*0.0f, sy2, pz2,  0.1f, 0.22f, 0.10f,  pr2,pg2,pb2);
                    }
                }
            }
        }
        // Checkout lanes near entrance
        for(int lane=-1;lane<=1;lane+=2){
            drawBox(lane*3.5f, 0, F-2.5f,  0.8f, 0.9f, 1.2f,  0.55f,0.55f,0.60f);
        }
        // Produce section at back-left: green mounds
        for(int p=0;p<6;p++){
            float px2=-W+1.8f+(p%3)*1.5f, pz2=-D+1.5f+(p/3)*1.5f;
            drawBox(px2, 0, pz2,  0.6f, 0.35f, 0.6f,  0.20f+p*0.03f,0.55f,0.15f);
        }

    } else if(type==STORE_ARBYS){
        // Booth seating along side walls
        for(int side=-1;side<=1;side+=2){
            for(int b=0;b<4;b++){
                float bz = -D+2.0f + b*3.0f;
                float bx = side*(W-1.2f);
                // Seat
                drawBox(bx, 0, bz,  0.9f, 0.45f, 0.9f,  0.60f,0.15f,0.10f);
                // Table
                drawBox(bx, 0.45f, bz,  0.8f, 0.05f, 0.6f,  0.78f,0.68f,0.55f);
                // Table leg
                drawBox(bx, 0, bz,  0.06f, 0.45f, 0.06f,  0.45f,0.38f,0.30f);
                // Backrest
                drawBox(bx, 0.45f, bz+(side<0?-0.85f:0.85f),  0.9f, 0.55f, 0.06f,  0.55f,0.12f,0.10f);
            }
        }
        // Menu display boards across back wall
        for(int bd=0;bd<3;bd++){
            float bx2=-ctW*0.6f+bd*ctW*0.6f;
            drawBox(bx2, 1.6f, -D+0.08f,  ctW*0.18f, 0.5f, 0.04f,  0.07f,0.07f,0.09f);
        }
        // Drive-through pickup window on side wall
        glColor3f(0.65f,0.78f,0.90f);
        glNormal3f(-1,0,0);
        glBegin(GL_QUADS);
            glVertex3f(-W+0.02f,1.0f,-D+3.0f); glVertex3f(-W+0.02f,2.2f,-D+3.0f);
            glVertex3f(-W+0.02f,2.2f,-D+5.0f); glVertex3f(-W+0.02f,1.0f,-D+5.0f);
        glEnd();
        // Sauce / condiment station mid-floor
        drawBox(0, 0, -1.0f,  1.5f, 1.0f, 0.5f,  0.72f,0.60f,0.45f);
        for(int sc=0;sc<5;sc++)
            drawBox(-1.0f+sc*0.5f, 1.0f, -0.9f,  0.09f, 0.22f, 0.09f,  0.85f,0.12f,0.10f);

    } else { // LJS
        // Dining tables (round approximated as small squares)
        for(int t2=0;t2<6;t2++){
            float tx2=(t2%3-1)*5.5f, tz2=(t2/3)*4.0f-9.0f;
            drawBox(tx2, 0, tz2,  0.9f, 0.75f, 0.9f,  0.18f,0.35f,0.60f); // table
            drawBox(tx2, 0.75f, tz2,  0.8f, 0.04f, 0.8f,  0.70f,0.68f,0.65f); // top
            for(int ch=0;ch<4;ch++){ // chairs
                float ca=ch*1.5708f;
                drawBox(tx2+cosf(ca)*1.3f,0,tz2+sinf(ca)*1.3f, 0.35f,0.45f,0.35f, 0.22f,0.38f,0.62f);
            }
        }
        // Fish tank on back wall, full width glow
        drawBox(0, 0.6f, -D+0.4f,  W-1.0f, 0.8f, 0.3f,  0.03f,0.05f,0.18f);
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glColor4f(0.10f,0.40f,0.90f,0.30f);
        glBegin(GL_QUADS);
            glNormal3f(0,0,1);
            glVertex3f(-(W-1.0f),0.0f,-D+0.72f); glVertex3f(W-1.0f,0.0f,-D+0.72f);
            glVertex3f(W-1.0f,1.4f,-D+0.72f);    glVertex3f(-(W-1.0f),1.4f,-D+0.72f);
        glEnd();
        glDisable(GL_BLEND); glEnable(GL_LIGHTING);
        // Barrel / crate decor
        for(int cr=0;cr<3;cr++)
            drawBox(-W+1.5f+cr*1.2f, 0, F-1.5f,  0.45f,0.80f,0.45f, 0.50f,0.32f,0.15f);
        // Nautical porthole windows on side wall (dark circles approximated)
        for(int pw=0;pw<3;pw++){
            float pz2=-D+2.0f+pw*4.0f;
            drawBox(W-0.08f, 1.8f, pz2,  0.04f,0.55f,0.55f, 0.55f,0.55f,0.58f);
        }
    }
}

// Render the interior UI (menu board text + ordering instructions) as a 2D overlay.
static void renderStoreUI(StoreType type){
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    gluOrtho2D(0, resWidth, 0, resHeight);
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
    glDisable(GL_DEPTH_TEST); glDisable(GL_LIGHTING);

    const MenuItem *menu;
    const char *storeName;
    float hr, hg, hb;
    if(type==STORE_ALBERTSONS){ menu=menuAlbertsons; storeName="ALBERTSONS";         hr=0.12f;hg=0.28f;hb=0.90f; }
    else if(type==STORE_ARBYS){ menu=menuArbys;      storeName="ARBY'S";             hr=0.90f;hg=0.10f;hb=0.10f; }
    else                       { menu=menuLJS;        storeName="LONG JOHN SILVER'S"; hr=0.10f;hg=0.40f;hb=0.90f; }

    int panelW = 340, panelH = 280;
    int px = resWidth - panelW - 10, py = resHeight/2 - panelH/2;
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.04f,0.04f,0.07f,0.88f);
    glBegin(GL_QUADS);
        glVertex2i(px, py);           glVertex2i(px+panelW, py);
        glVertex2i(px+panelW, py+panelH); glVertex2i(px, py+panelH);
    glEnd();
    glColor4f(hr,hg,hb,0.92f);
    glBegin(GL_QUADS);
        glVertex2i(px, py+panelH-36); glVertex2i(px+panelW, py+panelH-36);
        glVertex2i(px+panelW, py+panelH); glVertex2i(px, py+panelH);
    glEnd();
    glDisable(GL_BLEND);

    glColor3f(1,1,1);
    glRasterPos2i(px+8, py+panelH-20);
    for(const char *c=storeName;*c;c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,*c);

    // Menu items with inventory count
    for(int i=0;i<INV_SLOTS;i++){
        // Highlight if item is in inventory
        if(inventory[i] > 0) glColor3f(0.40f,1.0f,0.40f);
        else                  glColor3f(0.92f,0.88f,0.80f);
        char line[96];
        snprintf(line, sizeof(line), "%s  [x%d]", menu[i].label, inventory[i]);
        glRasterPos2i(px+8, py+panelH-62 - i*36);
        for(const char *c=line;*c;c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12,*c);
        // Second press hint
        if(inventory[i] > 0){
            glColor3f(0.60f,0.90f,0.60f);
            glRasterPos2i(px+8, py+panelH-76 - i*36);
            const char *eat = "  ^ press again to consume";
            for(const char *c=eat;*c;c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12,*c);
        }
    }

    // Divider
    glColor3f(0.35f,0.35f,0.45f);
    glBegin(GL_LINES);
        glVertex2i(px+4, py+14); glVertex2i(px+panelW-4, py+14);
    glEnd();

    glColor3f(0.50f,0.80f,0.50f);
    glRasterPos2i(px+8, py+4);
    const char *hint = "WASD=walk  1-5=order/eat  F=exit (from door)";
    for(const char *c=hint;*c;c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12,*c);

    glEnable(GL_DEPTH_TEST); glEnable(GL_LIGHTING);
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

static void renderStores(){
    float eyeX = inPlane ? player.x : walkerX;
    float eyeZ = inPlane ? player.z : walkerZ;

    glEnable(GL_LIGHTING);
    for(int i = 0; i < numStores; i++){
        Store *s = &stores[i];
        float dx = s->wx - eyeX, dz = s->wz - eyeZ;
        if(dx*dx + dz*dz > 2000.0f*2000.0f) continue;

        glPushMatrix();
        glTranslatef(s->wx, s->wy, s->wz);
        glRotatef(s->heading * 57.2958f, 0, 1, 0);

        // Exterior shell — sized to match the interior (SW_HALF wide, SD_BACK+SD_FRONT deep)
        // Local Z: door at +SD_FRONT, back at -SD_BACK. Centre of building at Z=(SD_FRONT-SD_BACK)/2
        float bW = SW_HALF;                          // half-width = 10
        float bD = (SD_FRONT + SD_BACK) * 0.5f;     // half-depth = 9.5
        float bCZ = (SD_FRONT - SD_BACK) * 0.5f;    // centre offset = -4.5
        float bH = SCEIL;                            // height = 4

        switch(s->type){
        case STORE_ALBERTSONS:
            // Flat-roofed big-box grocery
            drawStoreFascia(bW, bH, bD + bCZ,         // front face at local Z=SD_FRONT
                            0.88f,0.88f,0.88f, 0.12f,0.28f,0.70f);
            drawStoreSign(bD + bCZ, bH, "Albertsons",
                          0.12f,0.28f,0.70f, 1.0f,1.0f,1.0f);
            // Side + back walls
            drawBox(0, 0, bCZ,  bW, bH, bD,  0.86f,0.86f,0.84f);
            // Canopy over entrance
            drawBox(0, bH, bD+bCZ+0.3f,  bW*0.6f, 0.22f, 1.5f,  0.12f,0.28f,0.70f);
            // Parking light poles
            for(int pl=-1;pl<=1;pl+=2){
                drawBox(pl*(bW+2.0f), 0, bCZ,  0.12f, 5.5f, 0.12f,  0.65f,0.65f,0.65f);
                glDisable(GL_LIGHTING);
                glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);
                glColor4f(1.0f,0.95f,0.7f,0.45f);
                glBegin(GL_TRIANGLE_FAN);
                    glVertex3f(pl*(bW+2.0f),5.6f,bCZ);
                    for(int k=0;k<=8;k++){ float a=k*6.28318f/8;
                        glVertex3f(pl*(bW+2.0f)+cosf(a)*1.5f,5.6f,bCZ+sinf(a)*1.5f); }
                glEnd();
                glDisable(GL_BLEND); glEnable(GL_LIGHTING);
            }
            // Door frame
            drawBox(0, 0, bD+bCZ+0.05f,  1.2f, 2.4f, 0.08f,  0.55f,0.62f,0.68f);
            break;

        case STORE_ARBYS:
            drawStoreFascia(bW, bH, bD+bCZ,
                            0.75f,0.55f,0.30f, 0.80f,0.10f,0.10f);
            drawStoreSign(bD+bCZ, bH, "Arby's",
                          0.80f,0.10f,0.10f, 1.0f,0.85f,0.05f);
            drawBox(0, 0, bCZ,  bW, bH, bD,  0.73f,0.53f,0.28f);
            // Signature peaked hat roof
            glColor3f(0.80f,0.10f,0.10f);
            glBegin(GL_TRIANGLE_FAN);
                glNormal3f(0,1,0.2f);
                glVertex3f(0, bH+1.8f, bCZ);
                glVertex3f(-bW, bH, -(bD-bCZ)); glVertex3f(bW, bH, -(bD-bCZ));
                glVertex3f(bW, bH, bD+bCZ);     glVertex3f(-bW, bH, bD+bCZ);
                glVertex3f(-bW, bH, -(bD-bCZ));
            glEnd();
            // Drive-through lane
            glColor3f(0.90f,0.80f,0.10f);
            glNormal3f(0,1,0);
            glBegin(GL_QUADS);
                glVertex3f(-1.2f,0.02f,bD+bCZ); glVertex3f(1.2f,0.02f,bD+bCZ);
                glVertex3f(1.2f,0.02f,bD+bCZ+10.0f); glVertex3f(-1.2f,0.02f,bD+bCZ+10.0f);
            glEnd();
            drawBox(0, 0, bD+bCZ+0.05f,  1.0f, 2.2f, 0.08f,  0.60f,0.55f,0.50f);
            break;

        case STORE_LJS:
            drawStoreFascia(bW, bH, bD+bCZ,
                            0.22f,0.40f,0.72f, 1.00f,0.85f,0.10f);
            drawStoreSign(bD+bCZ, bH, "Long John Silver's",
                          0.10f,0.20f,0.55f, 1.0f,0.85f,0.10f);
            drawBox(0, 0, bCZ,  bW, bH, bD,  0.20f,0.38f,0.70f);
            // White peaked roof
            glColor3f(0.92f,0.92f,0.92f);
            glBegin(GL_TRIANGLES);
                glNormal3f(0, bD, bW);
                glVertex3f(-bW,bH,-(bD-bCZ)); glVertex3f(bW,bH,-(bD-bCZ));
                glVertex3f(0, bH+2.2f, bCZ);
                glNormal3f(0, bD,-bW);
                glVertex3f(-bW,bH,bD+bCZ); glVertex3f(bW,bH,bD+bCZ);
                glVertex3f(0, bH+2.2f, bCZ);
                glNormal3f(-bW, bD, 0);
                glVertex3f(-bW,bH,-(bD-bCZ)); glVertex3f(-bW,bH,bD+bCZ);
                glVertex3f(0, bH+2.2f, bCZ);
                glNormal3f(bW, bD, 0);
                glVertex3f(bW,bH,-(bD-bCZ)); glVertex3f(bW,bH,bD+bCZ);
                glVertex3f(0, bH+2.2f, bCZ);
            glEnd();
            drawBox(0, 0, bD+bCZ+0.05f,  1.0f, 2.2f, 0.08f,  0.55f,0.62f,0.68f);
            break;
        }

        // If player is inside this store, render the interior too
        if(inStoreIdx == i){
            drawStoreInterior(s->type);
        }

        glPopMatrix();
    }
}

// Render the store interior UI overlay (called from renderScene after world is drawn)
static void renderStoreOverlay(){
    if(inStoreIdx < 0 || inStoreIdx >= numStores) return;
    renderStoreUI(stores[inStoreIdx].type);
}

// -------- Trees --------
static void initTrees(){
    numTrees = 0;
    float halfW = TERRAIN_SIZE * TERRAIN_SCALE * 0.5f;
    srand(terrainSeed ^ 0x7EEDBABE);
    int attempts = MAX_TREES * 6;
    while(numTrees < MAX_TREES && attempts-- > 0){
        float tx = ((float)rand()/RAND_MAX*2.0f-1.0f)*halfW;
        float tz = ((float)rand()/RAND_MAX*2.0f-1.0f)*halfW;
        float h  = terrainHeightAt(tx, tz);

        // Skip water, mountain tops, near airports
        int gx = (int)((tx/TERRAIN_SCALE) + TERRAIN_SIZE/2.0f);
        int gz = (int)((tz/TERRAIN_SCALE) + TERRAIN_SIZE/2.0f);
        gx = gx<0?0:(gx>TERRAIN_SIZE?TERRAIN_SIZE:gx);
        gz = gz<0?0:(gz>TERRAIN_SIZE?TERRAIN_SIZE:gz);
        int biome = terrainBiome[gz][gx];
        if(biome == 0 || biome == 4 || biome == 7) continue; // skip water, snow, canyon
        if(h > TERRAIN_HEIGHT * 0.65f) continue;

        // Skip near airports
        bool nearAirport = false;
        for(int ai=0;ai<numAirports;ai++){
            float adx=tx-airports[ai].wx, adz=tz-airports[ai].wz;
            if(adx*adx+adz*adz < 80.0f*80.0f){ nearAirport=true; break; }
        }
        if(nearAirport) continue;

        Tree *t = &trees[numTrees++];
        t->x = tx; t->z = tz; t->y = h;
        t->h = 8.0f + (float)rand()/RAND_MAX * 18.0f;  // jurassic scale trees
        t->r = t->h * 0.55f;
        // Variant by biome: grass/highland=oak(1), sand/desert=palm(2), rock/tundra=pine(0), wetland=oak
        if(biome == 1)                  t->variant = 2; // palm for sand/desert
        else if(biome == 3 || biome==5) t->variant = 0; // pine for rock/tundra
        else                            t->variant = 1; // oak for grass/wetland
    }
    srand((unsigned)time(NULL));
}

// -------- Suburbs --------
#define MAX_SUBURB_HOUSES 1200
typedef struct {
    float wx, wy, wz;
    float heading;
    float w, d, h;   // half-width, half-depth, wall height
    float r, g, b;   // wall color
    float rr, rg, rb; // roof color
} SuburbHouse;
static SuburbHouse suburbHouses[MAX_SUBURB_HOUSES];
static int numSuburbHouses = 0;

static void initSuburbs(){
    numSuburbHouses = 0;
    srand(terrainSeed ^ 0xCAFEBABE);
    float halfW = TERRAIN_SIZE * TERRAIN_SCALE * 0.5f;

    static const float wallPalette[][3] = {
        {0.93f,0.87f,0.78f},{0.80f,0.72f,0.60f},{0.75f,0.80f,0.70f},
        {0.90f,0.78f,0.70f},{0.72f,0.78f,0.85f},{0.85f,0.82f,0.75f},
    };
    static const float roofPalette[][3] = {
        {0.35f,0.20f,0.12f},{0.50f,0.25f,0.18f},{0.28f,0.28f,0.32f},
        {0.60f,0.42f,0.28f},{0.20f,0.30f,0.22f},{0.45f,0.35f,0.28f},
    };

    for(int ai = 0; ai < numAirports && numSuburbHouses < MAX_SUBURB_HOUSES - 60; ai++){
        float apx = airports[ai].wx, apz = airports[ai].wz;
        int nBlocks = 3 + rand() % 4;
        for(int bl = 0; bl < nBlocks; bl++){
            float angle = ((float)rand()/RAND_MAX) * 6.28318f;
            float dist  = 160.0f + ((float)rand()/RAND_MAX) * 220.0f;
            float bx = apx + cosf(angle)*dist;
            float bz = apz + sinf(angle)*dist;
            bx = clampf(bx, -halfW+30, halfW-30);
            bz = clampf(bz, -halfW+30, halfW-30);
            int gx = (int)((bx/TERRAIN_SCALE) + TERRAIN_SIZE*0.5f);
            int gz = (int)((bz/TERRAIN_SCALE) + TERRAIN_SIZE*0.5f);
            gx = gx<0?0:(gx>TERRAIN_SIZE?TERRAIN_SIZE:gx);
            gz = gz<0?0:(gz>TERRAIN_SIZE?TERRAIN_SIZE:gz);
            if(terrainBiome[gz][gx] == 0) continue; // skip water

            float blockHead = ((float)rand()/RAND_MAX) * 6.28318f;
            float ch = cosf(blockHead), sh = sinf(blockHead);
            int rows = 2 + rand() % 3, cols = 3 + rand() % 5;
            float lotW = 10.0f + ((float)rand()/RAND_MAX)*5.0f;
            float lotD = 12.0f + ((float)rand()/RAND_MAX)*6.0f;

            for(int row = 0; row < rows && numSuburbHouses < MAX_SUBURB_HOUSES; row++){
                for(int col = 0; col < cols && numSuburbHouses < MAX_SUBURB_HOUSES; col++){
                    float fx = (col - cols*0.5f) * lotW;
                    float fz = (row - rows*0.5f) * lotD;
                    float wx = bx + ch*fx + sh*fz;
                    float wz = bz - sh*fx + ch*fz;
                    wx = clampf(wx, -halfW+10, halfW-10);
                    wz = clampf(wz, -halfW+10, halfW-10);
                    int gx2 = (int)((wx/TERRAIN_SCALE)+TERRAIN_SIZE*0.5f);
                    int gz2 = (int)((wz/TERRAIN_SCALE)+TERRAIN_SIZE*0.5f);
                    gx2=gx2<0?0:(gx2>TERRAIN_SIZE?TERRAIN_SIZE:gx2);
                    gz2=gz2<0?0:(gz2>TERRAIN_SIZE?TERRAIN_SIZE:gz2);
                    if(terrainBiome[gz2][gx2] == 0) continue;
                    float ey = terrainHeightAt(wx, wz);
                    float hw = 2.5f + ((float)rand()/RAND_MAX)*1.5f;
                    float hd = 3.5f + ((float)rand()/RAND_MAX)*1.5f;
                    float hh = 2.2f + ((float)rand()/RAND_MAX)*0.8f;
                    int wi = rand() % 6, ri = rand() % 6;
                    float hHead = blockHead + (rand()%2)*3.14159f;
                    flattenRect(wx, wz, hHead, hd+3.0f, hw+3.0f, ey, 4.0f);
                    SuburbHouse *house = &suburbHouses[numSuburbHouses++];
                    house->wx = wx; house->wy = ey; house->wz = wz;
                    house->heading = hHead;
                    house->w = hw; house->d = hd; house->h = hh;
                    house->r  = wallPalette[wi][0]; house->g  = wallPalette[wi][1]; house->b  = wallPalette[wi][2];
                    house->rr = roofPalette[ri][0]; house->rg = roofPalette[ri][1]; house->rb = roofPalette[ri][2];
                }
            }
        }
    }
    bakeTerrainColors();
    srand((unsigned)time(NULL));
}

static void drawHouse(const SuburbHouse *h){
    glPushMatrix();
    glTranslatef(h->wx, h->wy, h->wz);
    glRotatef(h->heading * 57.2958f, 0, 1, 0);

    // Walls
    drawBox(0, 0, 0, h->w, h->h, h->d, h->r, h->g, h->b);

    // Pitched roof (two triangular end caps + two sloped faces)
    float rH = h->w * 0.65f; // peak height above walls
    glDisable(GL_LIGHTING);
    glColor3f(h->rr, h->rg, h->rb);
    glNormal3f(0, h->d, rH);
    glBegin(GL_QUADS);
        glVertex3f(-h->w, h->h,  h->d);
        glVertex3f( h->w, h->h,  h->d);
        glVertex3f( h->w, h->h, -h->d);
        glVertex3f(-h->w, h->h, -h->d);
    glEnd();
    // Front slope
    glNormal3f(0, rH, h->d);
    glBegin(GL_TRIANGLES);
        glVertex3f(-h->w, h->h,  h->d);
        glVertex3f( h->w, h->h,  h->d);
        glVertex3f(0,     h->h + rH, 0);
    glEnd();
    // Back slope
    glNormal3f(0, rH, -h->d);
    glBegin(GL_TRIANGLES);
        glVertex3f(-h->w, h->h, -h->d);
        glVertex3f( h->w, h->h, -h->d);
        glVertex3f(0,     h->h + rH, 0);
    glEnd();
    // Left end cap
    glNormal3f(-1, 0, 0);
    glBegin(GL_TRIANGLES);
        glVertex3f(-h->w, h->h, -h->d);
        glVertex3f(-h->w, h->h,  h->d);
        glVertex3f(-h->w, h->h + rH, 0);
    glEnd();
    // Right end cap
    glNormal3f(1, 0, 0);
    glBegin(GL_TRIANGLES);
        glVertex3f(h->w, h->h, -h->d);
        glVertex3f(h->w, h->h,  h->d);
        glVertex3f(h->w, h->h + rH, 0);
    glEnd();
    glEnable(GL_LIGHTING);

    // Driveway (flat dark strip in front)
    glDisable(GL_LIGHTING);
    glColor3f(0.30f, 0.30f, 0.30f);
    glNormal3f(0, 1, 0);
    glBegin(GL_QUADS);
        glVertex3f(-h->w*0.4f, 0.03f,  h->d);
        glVertex3f( h->w*0.4f, 0.03f,  h->d);
        glVertex3f( h->w*0.4f, 0.03f,  h->d + 5.0f);
        glVertex3f(-h->w*0.4f, 0.03f,  h->d + 5.0f);
    glEnd();
    glEnable(GL_LIGHTING);

    glPopMatrix();
}

static void renderSuburbs(){
    if(numSuburbHouses == 0) return;
    float eyeX = inPlane ? player.x : walkerX;
    float eyeZ = inPlane ? player.z : walkerZ;
    for(int i = 0; i < numSuburbHouses; i++){
        const SuburbHouse *h = &suburbHouses[i];
        float dx = h->wx - eyeX, dz = h->wz - eyeZ;
        if(dx*dx + dz*dz > 2000.0f*2000.0f) continue;
        drawHouse(h);
    }
}

static void renderTrees(){
    if(numTrees == 0) return;

    float eyeX = inPlane ? player.x : walkerX;
    float eyeZ = inPlane ? player.z : walkerZ;

    // Get camera right/up for billboarding
    float mv[16]; glGetFloatv(GL_MODELVIEW_MATRIX, mv);
    float rx = mv[0], rz = mv[8]; // right vector X and Z (ignore Y for upright trees)

    glDisable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    for(int i = 0; i < numTrees; i++){
        Tree *t = &trees[i];
        float dx = t->x - eyeX, dz = t->z - eyeZ;
        float d2 = dx*dx + dz*dz;
        if(d2 > 1400.0f*1400.0f) continue;

        float fade = 1.0f;
        if(d2 > 1200.0f*1200.0f) fade = 1.0f - (sqrtf(d2)-1200.0f)/200.0f;

        // Trunk
        glColor3f(0.38f, 0.25f, 0.14f);
        float tx2 = t->x, ty = t->y, tz2 = t->z;
        float trunkH = t->h * 0.40f;
        float trunkR = t->r * 0.12f;
        glBegin(GL_QUADS);
            glNormal3f(rx, 0, rz);
            glVertex3f(tx2 - rx*trunkR, ty,           tz2 - rz*trunkR);
            glVertex3f(tx2 + rx*trunkR, ty,           tz2 + rz*trunkR);
            glVertex3f(tx2 + rx*trunkR, ty + trunkH,  tz2 + rz*trunkR);
            glVertex3f(tx2 - rx*trunkR, ty + trunkH,  tz2 - rz*trunkR);
        glEnd();

        // Canopy — variant colours
        float cr, cg, cb;
        if(t->variant == 0){ cr=0.18f; cg=0.42f; cb=0.15f; } // pine: dark green
        else if(t->variant==1){ cr=0.22f; cg=0.52f; cb=0.18f; } // oak: medium green
        else { cr=0.38f; cg=0.58f; cb=0.12f; } // palm: yellow-green

        glColor4f(cr, cg, cb, fade);

        if(t->variant == 0){
            // Pine: 3 stacked triangular layers
            for(int layer = 0; layer < 3; layer++){
                float ly0 = ty + trunkH + layer * t->h * 0.20f;
                float ly1 = ly0 + t->h * 0.40f;
                float lr  = t->r * (1.0f - layer * 0.28f);
                // Two crossed quads
                for(int cross=0; cross<2; cross++){
                    float ax = (cross==0) ? rx : rz;
                    float az = (cross==0) ? rz : -rx;
                    glBegin(GL_TRIANGLES);
                        glNormal3f(0,0,1);
                        glVertex3f(tx2 - ax*lr, ly0, tz2 - az*lr);
                        glVertex3f(tx2 + ax*lr, ly0, tz2 + az*lr);
                        glVertex3f(tx2,          ly1, tz2);
                    glEnd();
                }
            }
        } else if(t->variant == 1){
            // Oak: round puff — crossed billboard quads
            float cy2 = ty + trunkH + t->r * 0.6f;
            float lr = t->r;
            for(int cross=0; cross<2; cross++){
                float ax = (cross==0) ? rx : rz;
                float az = (cross==0) ? rz : -rx;
                glBegin(GL_QUADS);
                    glNormal3f(0,0,1);
                    glVertex3f(tx2 - ax*lr, cy2 - lr*0.7f, tz2 - az*lr);
                    glVertex3f(tx2 + ax*lr, cy2 - lr*0.7f, tz2 + az*lr);
                    glVertex3f(tx2 + ax*lr, cy2 + lr*0.7f, tz2 + az*lr);
                    glVertex3f(tx2 - ax*lr, cy2 + lr*0.7f, tz2 - az*lr);
                glEnd();
            }
        } else {
            // Palm: fan of fronds at top
            float topY = ty + t->h;
            float trunkMidX = tx2, trunkMidZ = tz2;
            // Straight trunk (billboard)
            glColor4f(0.55f, 0.42f, 0.22f, fade);
            glBegin(GL_QUADS);
                glVertex3f(trunkMidX - rx*trunkR*1.5f, ty,      trunkMidZ - rz*trunkR*1.5f);
                glVertex3f(trunkMidX + rx*trunkR*1.5f, ty,      trunkMidZ + rz*trunkR*1.5f);
                glVertex3f(trunkMidX + rx*trunkR*0.8f, topY,    trunkMidZ + rz*trunkR*0.8f);
                glVertex3f(trunkMidX - rx*trunkR*0.8f, topY,    trunkMidZ - rz*trunkR*0.8f);
            glEnd();
            // Fronds (6 drooping blades)
            glColor4f(cr, cg, cb, fade);
            for(int f=0; f<6; f++){
                float fa = f * 3.14159f * 2.0f / 6.0f;
                float fx2 = cosf(fa)*t->r, fz2 = sinf(fa)*t->r;
                glBegin(GL_TRIANGLES);
                    glNormal3f(0,1,0);
                    glVertex3f(tx2,        topY,         tz2);
                    glVertex3f(tx2+fx2,    topY-t->r*0.4f, tz2+fz2);
                    glVertex3f(tx2+fx2*0.6f, topY+0.5f,  tz2+fz2*0.6f);
                glEnd();
            }
        }
    }

    glDisable(GL_BLEND);
}

// -------- Clouds --------
static void initClouds(){
    numClouds = 0;
    float halfW = TERRAIN_SIZE * TERRAIN_SCALE * 0.5f;
    srand(terrainSeed ^ 0xC1A3D5);
    for(int i = 0; i < MAX_CLOUDS; i++){
        Cloud *c = &clouds[numClouds++];
        c->x = ((float)rand()/RAND_MAX * 2.0f - 1.0f) * halfW;
        c->z = ((float)rand()/RAND_MAX * 2.0f - 1.0f) * halfW;
        // Layer clouds between altitudes based on weather
        c->y = 60.0f + (float)rand()/RAND_MAX * 80.0f;
        c->r = 18.0f + (float)rand()/RAND_MAX * 40.0f;
        c->brightness = 0.78f + (float)rand()/RAND_MAX * 0.22f;
    }
    srand((unsigned)time(NULL)); // restore proper random state
}

static void renderClouds(){
    if(numClouds == 0) return;

    float mv[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, mv);
    float rx = mv[0], ry = mv[4], rz = mv[8];
    float ux = mv[1], uy = mv[5], uz = mv[9];

    float camX = inPlane ? player.x : walkerX;
    float camY = inPlane ? player.y : walkerY;
    float camZ = inPlane ? player.z : walkerZ;

    // Build visible list with squared distances for back-to-front sort
    static int  order[MAX_CLOUDS];
    static float dist2[MAX_CLOUDS];
    int visible = 0;
    float cullR = 1200.0f;
    for(int i = 0; i < numClouds; i++){
        Cloud *c = &clouds[i];
        float dx=c->x-camX, dy=c->y-camY, dz=c->z-camZ;
        float d2 = dx*dx+dy*dy+dz*dz;
        if(d2 > cullR*cullR) continue;
        order[visible]  = i;
        dist2[visible]  = d2;
        visible++;
    }
    // Insertion sort (small N, fast enough)
    for(int i=1;i<visible;i++){
        int   ki=order[i]; float kd=dist2[i]; int j=i-1;
        while(j>=0 && dist2[j]<kd){ order[j+1]=order[j]; dist2[j+1]=dist2[j]; j--; }
        order[j+1]=ki; dist2[j+1]=kd;
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    // Depth test ON, depth write OFF — clouds occlude nothing but are occluded by solid geo
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    static const float offsets[5][3] = {
        { 0.0f,  0.0f, 0.0f},
        { 0.7f,  0.3f, 0.0f},
        {-0.6f,  0.4f, 0.0f},
        { 0.2f, -0.5f, 0.0f},
        {-0.3f, -0.4f, 0.0f},
    };

    for(int vi = 0; vi < visible; vi++){
        Cloud *c = &clouds[order[vi]];
        float dx=c->x-camX, dy=c->y-camY, dz=c->z-camZ;
        float d2=dist2[vi];

        float alpha = 0.58f * (1.0f - d2/(cullR*cullR));
        if(weather.type==WX_OVERCAST||weather.type==WX_STORM)
            alpha = fminf(alpha*1.6f, 0.85f);

        float b = c->brightness;
        if(weather.type==WX_STORM) b*=0.50f;
        else if(weather.type==WX_OVERCAST) b*=0.70f;

        // If camera is inside this cloud, full white fog-like fill
        float insideR = c->r * 0.6f;
        if(dx*dx+dy*dy+dz*dz < insideR*insideR){
            // Already inside — don't draw this cloud (fog covers it)
            continue;
        }

        glColor4f(b, b, b*0.97f, alpha);

        for(int p = 0; p < 5; p++){
            float px = c->x + offsets[p][0]*c->r*0.9f;
            float py = c->y + offsets[p][1]*c->r*0.5f;
            float pz = c->z + offsets[p][2]*c->r*0.9f;
            float pr = c->r * (0.7f + offsets[p][1]*0.25f);
            glBegin(GL_TRIANGLE_FAN);
            glVertex3f(px, py, pz);
            for(int j=0;j<=8;j++){
                float a=j*6.28318f/8.0f;
                float ca=cosf(a)*pr, sa=sinf(a)*pr;
                glVertex3f(px+rx*ca+ux*sa, py+ry*ca+uy*sa, pz+rz*ca+uz*sa);
            }
            glEnd();
        }
    }

    glDepthMask(GL_TRUE);
    glEnable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
}

// -------- Sky / Sun --------
// Returns sky RGB and sun direction for given gameTime [0,1)
static void skyColorsForTime(float t,
                              float *skyR, float *skyG, float *skyB,
                              float *horizR, float *horizG, float *horizB,
                              float *sunElevDeg, float *sunAzimDeg){
    // t: 0=midnight, 0.25=6am dawn, 0.5=noon, 0.75=6pm dusk, 1=midnight
    float ang = t * 2.0f * (float)M_PI;  // sun azimuth circles around
    *sunAzimDeg  = t * 360.0f;
    *sunElevDeg  = sinf(ang) * 90.0f;    // +90 at noon, -90 at midnight

    float elev01 = (*sunElevDeg + 90.0f) / 180.0f; // 0=nadir, 0.5=horizon, 1=zenith
    float dayF   = (*sunElevDeg > 0.0f) ? (*sunElevDeg / 90.0f) : 0.0f;
    float dawnF  = 1.0f - fabsf(*sunElevDeg / 18.0f);
    if(dawnF < 0.0f) dawnF = 0.0f;

    // Zenith colour
    *skyR = 0.08f + dayF * 0.42f;
    *skyG = 0.12f + dayF * 0.56f;
    *skyB = 0.22f + dayF * 0.68f;

    // Horizon colour (warm at dawn/dusk, pale blue at noon, dark at night)
    *horizR = 0.10f + dayF*0.60f + dawnF*0.50f;
    *horizG = 0.10f + dayF*0.55f + dawnF*0.15f;
    *horizB = 0.15f + dayF*0.50f - dawnF*0.10f;
    if(*horizB < 0.0f) *horizB = 0.0f;
}

static void updateSunLight(){
    float skyR, skyG, skyB, horizR, horizG, horizB, elevDeg, azimDeg;
    skyColorsForTime(gameTime, &skyR,&skyG,&skyB, &horizR,&horizG,&horizB, &elevDeg, &azimDeg);

    float elevRad = elevDeg * (float)M_PI / 180.0f;
    float azimRad = azimDeg * (float)M_PI / 180.0f;
    float lx = -sinf(azimRad)*cosf(elevRad);
    float ly =  sinf(elevRad);
    float lz = -cosf(azimRad)*cosf(elevRad);

    GLfloat lpos[4] = { lx, ly, lz, 0.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lpos);

    float dayF = (elevDeg > 0.0f) ? (elevDeg / 90.0f) : 0.0f;
    float dawnF = 1.0f - fabsf(elevDeg / 18.0f); if(dawnF<0)dawnF=0;

    GLfloat amb[4]  = { 0.05f+dayF*0.20f,  0.05f+dayF*0.20f,  0.08f+dayF*0.22f,  1.0f };
    GLfloat diff[4] = { 0.50f+dayF*0.50f+dawnF*0.30f,
                        0.45f+dayF*0.45f,
                        0.35f+dayF*0.45f, 1.0f };
    GLfloat spec[4] = { diff[0]*0.7f, diff[1]*0.7f, diff[2]*0.7f, 1.0f };
    glLightfv(GL_LIGHT0, GL_AMBIENT,  amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  diff);
    glLightfv(GL_LIGHT0, GL_SPECULAR, spec);

    // Blend time-of-day horizon colour with weather (weather darkens/greys the sky)
    // weatherFactor: 0=clear, 1=storm (from fog density proxy)
    float wf = clampf(weather.fogDensity / 0.007f, 0.0f, 1.0f);
    float baseR = horizR*(1.0f-wf) + (horizR*0.4f+0.1f)*wf;
    float baseG = horizG*(1.0f-wf) + (horizG*0.4f+0.1f)*wf;
    float baseB = horizB*(1.0f-wf) + (horizB*0.4f+0.15f)*wf;
    weather.skyR = baseR;
    weather.skyG = baseG;
    weather.skyB = baseB;
    glClearColor(weather.skyR, weather.skyG, weather.skyB, 1.0f);
    // Update fog colour too
    GLfloat fogCol[4] = {weather.skyR, weather.skyG, weather.skyB, 1.0f};
    glFogfv(GL_FOG_COLOR, fogCol);
}

static void renderSkyDome(float camX, float camY, float camZ){
    float skyR, skyG, skyB, horizR, horizG, horizB, elevDeg, azimDeg;
    skyColorsForTime(gameTime, &skyR,&skyG,&skyB, &horizR,&horizG,&horizB, &elevDeg, &azimDeg);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_FOG);

    glPushMatrix();
    glTranslatef(camX, camY, camZ);

    // Draw sky as a large hemisphere of quads (zenith=skyRGB, horizon=horizRGB)
    const int stacks=8, slices=16;
    const float R = 900.0f;
    for(int s=0; s<stacks; s++){
        float a0 = (float)s     / stacks * (float)M_PI * 0.5f;  // 0..pi/2
        float a1 = (float)(s+1) / stacks * (float)M_PI * 0.5f;
        float t0 = (float)s     / stacks;  // 0=horizon, 1=zenith
        float t1 = (float)(s+1) / stacks;
        // Lerp colours
        float r0=horizR+(skyR-horizR)*t0, g0=horizG+(skyG-horizG)*t0, b0=horizB+(skyB-horizB)*t0;
        float r1=horizR+(skyR-horizR)*t1, g1=horizG+(skyG-horizG)*t1, b1=horizB+(skyB-horizB)*t1;
        glBegin(GL_TRIANGLE_STRIP);
        for(int sl=0; sl<=slices; sl++){
            float phi = (float)sl / slices * 2.0f * (float)M_PI;
            float cp = cosf(phi), sp2 = sinf(phi);
            glColor3f(r1,g1,b1);
            glVertex3f(cp*cosf(a1)*R, sinf(a1)*R, sp2*cosf(a1)*R);
            glColor3f(r0,g0,b0);
            glVertex3f(cp*cosf(a0)*R, sinf(a0)*R, sp2*cosf(a0)*R);
        }
        glEnd();
    }
    // Below horizon: fill with horizon colour
    glColor3f(horizR, horizG, horizB);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0,-R,0);
    for(int sl=0;sl<=slices;sl++){
        float phi = (float)sl/slices*2.0f*(float)M_PI;
        glVertex3f(cosf(phi)*R, 0, sinf(phi)*R);
    }
    glEnd();

    // Sun disc (only when above horizon)
    if(elevDeg > -5.0f){
        float elevRad = elevDeg * (float)M_PI / 180.0f;
        float azimRad = azimDeg * (float)M_PI / 180.0f;
        float sx = -sinf(azimRad)*cosf(elevRad)*0.98f*R;
        float sy =  sinf(elevRad)*0.98f*R;
        float sz = -cosf(azimRad)*cosf(elevRad)*0.98f*R;
        float dayF = (elevDeg>0)?(elevDeg/90.0f):0.0f;
        float dawnF2 = 1.0f-fabsf(elevDeg/15.0f); if(dawnF2<0)dawnF2=0;
        glBegin(GL_TRIANGLE_FAN);
        glColor3f(1.0f, 0.95f+dawnF2*0.05f-dayF*0.05f, 0.60f+dayF*0.35f);
        glVertex3f(sx, sy, sz);
        glColor3f(horizR*1.4f, horizG*1.1f, horizB*0.7f);
        float sunR = 40.0f;
        // two tangent vectors perpendicular to sun direction
        float len = sqrtf(sx*sx+sy*sy+sz*sz);
        float ux,uy,uz;
        if(fabsf(sy/len) < 0.9f){ ux=0;uy=1;uz=0; }
        else { ux=1;uy=0;uz=0; }
        float tx = uy*(sz/len)-uz*(sy/len), ty = uz*(sx/len)-ux*(sz/len), tz = ux*(sy/len)-uy*(sx/len);
        float bx = (sy/len)*tz-(sz/len)*ty, by=(sz/len)*tx-(sx/len)*tz, bz=(sx/len)*ty-(sy/len)*tx;
        for(int k=0;k<=16;k++){
            float a = (float)k/16*2.0f*(float)M_PI;
            glVertex3f(sx+(tx*cosf(a)+bx*sinf(a))*sunR,
                       sy+(ty*cosf(a)+by*sinf(a))*sunR,
                       sz+(tz*cosf(a)+bz*sinf(a))*sunR);
        }
        glEnd();
    }

    glPopMatrix();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_FOG);
}

// -------- Cockpit View --------
// Geometry units: 1 unit ≈ cockpit width. Dashboard sits at z ≈ -1.4 so
// there's plenty of forward FOV. Panel top edge is at y ≈ -0.38 (below horizon).
static void renderCockpit(){
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_FOG);
    glEnable(GL_DEPTH_TEST);  // use depth so panel occludes stick etc.
    glClear(GL_DEPTH_BUFFER_BIT);  // fresh depth for cockpit layer

    // sun-side ambient tint to simulate light coming through canopy
    float dayF2 = clampf(sinf(gameTime*2.0f*(float)M_PI), 0.0f, 1.0f);
    float ambR = 0.10f + dayF2*0.18f;
    float ambG = 0.10f + dayF2*0.14f;
    float ambB = 0.12f + dayF2*0.10f;

    // helper: quad with simple vertical shading to fake AO
#define CPANEL_QUAD(x0,y0,z0, x1,y1,z1, x2,y2,z2, x3,y3,z3, r,g,b) \
    glBegin(GL_QUADS); \
    glColor3f((r)*0.75f,(g)*0.75f,(b)*0.75f); glVertex3f(x0,y0,z0); \
    glColor3f((r)*0.75f,(g)*0.75f,(b)*0.75f); glVertex3f(x1,y1,z1); \
    glColor3f((r),(g),(b));                    glVertex3f(x2,y2,z2); \
    glColor3f((r),(g),(b));                    glVertex3f(x3,y3,z3); \
    glEnd();

    if(player.type == PLANE_FIGHTER){
        // ---- F/A-18 Hornet cockpit ----
        // Dashboard — far back, low on screen
        float pz  = -1.40f;   // panel face Z
        float pzt = -1.20f;   // panel top (tilted toward pilot)
        float pt  = -0.38f;   // top edge Y
        float pb  = -0.85f;   // bottom edge Y
        float pw  =  0.62f;   // half-width
        CPANEL_QUAD(-pw,pb,pz, pw,pb,pz, pw,pt,pzt, -pw,pt,pzt, 0.09f,0.09f,0.11f)

        // Glareshield (thin shelf above panel, deep black, cuts horizon slightly)
        float gz = -1.15f;
        CPANEL_QUAD(-pw,pt,pzt, pw,pt,pzt, pw,pt+0.04f,gz, -pw,pt+0.04f,gz, 0.05f,0.05f,0.06f)

        // Canopy frame — thin side bows, far enough not to block view
        float cbz0 = -0.60f, cbz1 = -2.50f;
        float cby0 = -0.10f, cby1 =  0.55f;
        float cbx  =  0.68f, cbt  =  0.04f; // x position and thickness
        // Left bow
        CPANEL_QUAD(-cbx,cby0,cbz0, -cbx+cbt,cby0,cbz0, -cbx+cbt,cby1,cbz1, -cbx,cby1,cbz1, 0.14f,0.14f,0.16f)
        // Right bow
        CPANEL_QUAD( cbx-cbt,cby0,cbz0,  cbx,cby0,cbz0,  cbx,cby1,cbz1,  cbx-cbt,cby1,cbz1, 0.14f,0.14f,0.16f)
        // Top arch bar (very thin)
        CPANEL_QUAD(-cbx,cby1-0.025f,cbz1, cbx,cby1-0.025f,cbz1, cbx,cby1,cbz1, -cbx,cby1,cbz1, 0.12f,0.12f,0.14f)

        // Cockpit side walls (dark grey inside fuselage)
        CPANEL_QUAD(-cbx,pb,cbz0, -cbx,pt,cbz1, -cbx,cby1,cbz1, -cbx,pb,cbz1, 0.08f+ambR*0.3f,0.08f+ambG*0.3f,0.10f+ambB*0.3f)
        CPANEL_QUAD( cbx,pt,cbz1,  cbx,pb,cbz0,  cbx,pb,cbz1,  cbx,cby1,cbz1, 0.08f+ambR*0.3f,0.08f+ambG*0.3f,0.10f+ambB*0.3f)

        // HUD combiner glass (transparent, tilted)
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.12f+dayF2*0.05f, 0.95f, 0.22f, 0.10f);
        glBegin(GL_QUADS);
          glVertex3f(-0.30f, pt+0.05f, pzt-0.02f);
          glVertex3f( 0.30f, pt+0.05f, pzt-0.02f);
          glVertex3f( 0.30f, pt+0.30f, -0.90f);
          glVertex3f(-0.30f, pt+0.30f, -0.90f);
        glEnd();
        // HUD green reticle lines
        glColor4f(0.0f, 1.0f, 0.1f, 0.55f);
        glLineWidth(1.2f);
        glBegin(GL_LINES);
          // Horizon line
          glVertex3f(-0.10f, pt+0.18f, -0.91f); glVertex3f( 0.10f, pt+0.18f, -0.91f);
          // Pitch ladder ticks
          glVertex3f(-0.04f, pt+0.22f, -0.91f); glVertex3f( 0.04f, pt+0.22f, -0.91f);
          glVertex3f(-0.04f, pt+0.14f, -0.91f); glVertex3f( 0.04f, pt+0.14f, -0.91f);
          // Velocity vector circle (small)
          float vvx = player.roll*0.04f, vvy = player.pitch*0.04f;
          for(int k=0;k<8;k++){
              float a0=k*6.28318f/8, a1=(k+1)*6.28318f/8;
              glVertex3f(vvx+cosf(a0)*0.018f, pt+0.18f+vvy+sinf(a0)*0.018f, -0.91f);
              glVertex3f(vvx+cosf(a1)*0.018f, pt+0.18f+vvy+sinf(a1)*0.018f, -0.91f);
          }
        glEnd();
        glLineWidth(1.0f);
        glDisable(GL_BLEND);

        // Throttle quadrant (left console)
        float tqx = -pw+0.06f, tqw = 0.10f;
        float tqz = pz+0.10f, tqBot = pb, tqTop = pb+0.28f;
        CPANEL_QUAD(tqx,tqBot,tqz, tqx+tqw,tqBot,tqz, tqx+tqw,tqTop,tqz, tqx,tqTop,tqz, 0.18f,0.18f,0.20f)
        // Throttle lever
        float tlevel = pb + 0.05f + player.throttle*0.20f;
        CPANEL_QUAD(tqx+0.01f,tlevel,tqz-0.005f, tqx+tqw-0.01f,tlevel,tqz-0.005f,
                    tqx+tqw-0.01f,tlevel+0.04f,tqz-0.010f, tqx+0.01f,tlevel+0.04f,tqz-0.010f, 0.30f,0.30f,0.35f)

        // Stick
        float sx = player.roll*0.06f, sy = -player.pitch*0.06f;
        float sbase = -0.65f, stop = pz+0.12f;
        glColor3f(0.20f,0.20f,0.22f);
        glBegin(GL_QUADS);
          glVertex3f(    -0.025f,      sbase, -0.55f);
          glVertex3f(     0.025f,      sbase, -0.55f);
          glVertex3f(sx + 0.025f, stop+sy,   pz+0.02f);
          glVertex3f(sx - 0.025f, stop+sy,   pz+0.02f);
        glEnd();
        // Stick top grip
        glColor3f(0.25f,0.25f,0.28f);
        glBegin(GL_QUADS);
          glVertex3f(sx-0.030f, stop+sy,        pz+0.005f);
          glVertex3f(sx+0.030f, stop+sy,        pz+0.005f);
          glVertex3f(sx+0.030f, stop+sy+0.055f, pz+0.005f);
          glVertex3f(sx-0.030f, stop+sy+0.055f, pz+0.005f);
        glEnd();

    } else if(player.type == PLANE_PROP){
        // ---- Cessna 172 cockpit ----
        float pz  = -1.35f;
        float pzt = -1.10f;
        float pt  = -0.36f;
        float pb  = -0.82f;
        float pw  =  0.72f;
        // Main panel (warm beige/grey)
        CPANEL_QUAD(-pw,pb,pz, pw,pb,pz, pw,pt,pzt, -pw,pt,pzt, 0.28f,0.25f,0.20f)

        // Glare shield (dark foam)
        CPANEL_QUAD(-pw,pt,pzt, pw,pt,pzt, pw,pt+0.05f,-1.08f, -pw,pt+0.05f,-1.08f, 0.08f,0.07f,0.06f)

        // Window pillars (A-pillar each side)
        float apz0 = -0.55f, apz1 = -2.2f;
        float apy0 = -0.12f, apy1 =  0.50f;
        float apx  =  0.72f, apt  =  0.055f;
        CPANEL_QUAD(-apx,apy0,apz0, -apx+apt,apy0,apz0, -apx+apt,apy1,apz1, -apx,apy1,apz1, 0.55f,0.52f,0.46f)
        CPANEL_QUAD( apx-apt,apy0,apz0,  apx,apy0,apz0,  apx,apy1,apz1,  apx-apt,apy1,apz1, 0.55f,0.52f,0.46f)

        // Side walls
        CPANEL_QUAD(-apx,pb,apz0, -apx,pt,apz1, -apx,apy1,apz1, -apx,pb,apz1, 0.22f+ambR*0.4f,0.20f+ambG*0.4f,0.17f+ambB*0.4f)
        CPANEL_QUAD( apx,pt,apz1,  apx,pb,apz0,  apx,pb,apz1,  apx,apy1,apz1, 0.22f+ambR*0.4f,0.20f+ambG*0.4f,0.17f+ambB*0.4f)

        // Instrument sub-panel (darker centre section)
        CPANEL_QUAD(-0.38f,pb+0.06f,pz+0.002f, 0.38f,pb+0.06f,pz+0.002f,
                     0.38f,pt-0.02f,pzt+0.002f, -0.38f,pt-0.02f,pzt+0.002f, 0.14f,0.13f,0.12f)

        // Yoke column (centre, coming up from below screen)
        float ycol = player.roll*0.05f, ypush = -player.pitch*0.05f;
        glColor3f(0.18f,0.18f,0.20f);
        glBegin(GL_QUADS);
          glVertex3f(-0.028f, -0.80f, -0.70f);
          glVertex3f( 0.028f, -0.80f, -0.70f);
          glVertex3f(ycol+0.028f, pt+0.02f+ypush, pzt+0.04f);
          glVertex3f(ycol-0.028f, pt+0.02f+ypush, pzt+0.04f);
        glEnd();
        // Yoke U-bar
        float yuy = pt+0.04f+ypush, yuz = pzt+0.035f;
        glBegin(GL_QUADS);
          glVertex3f(ycol-0.22f, yuy-0.018f, yuz);
          glVertex3f(ycol+0.22f, yuy-0.018f, yuz);
          glVertex3f(ycol+0.22f, yuy+0.018f, yuz);
          glVertex3f(ycol-0.22f, yuy+0.018f, yuz);
        glEnd();
        // Yoke left grip
        glBegin(GL_QUADS);
          glVertex3f(ycol-0.22f, yuy-0.018f, yuz);
          glVertex3f(ycol-0.22f+0.025f, yuy-0.018f, yuz);
          glVertex3f(ycol-0.22f+0.025f, yuy-0.075f, yuz);
          glVertex3f(ycol-0.22f, yuy-0.075f, yuz);
        glEnd();
        // Yoke right grip
        glBegin(GL_QUADS);
          glVertex3f(ycol+0.22f-0.025f, yuy-0.018f, yuz);
          glVertex3f(ycol+0.22f,        yuy-0.018f, yuz);
          glVertex3f(ycol+0.22f,        yuy-0.075f, yuz);
          glVertex3f(ycol+0.22f-0.025f, yuy-0.075f, yuz);
        glEnd();

        // Cowling edge visible over nose
        glColor3f(0.50f+ambR*0.3f, 0.48f+ambG*0.3f, 0.42f+ambB*0.3f);
        glBegin(GL_QUADS);
          glVertex3f(-0.40f, pt+0.06f, -1.05f);
          glVertex3f( 0.40f, pt+0.06f, -1.05f);
          glVertex3f( 0.40f, pt+0.13f, -1.80f);
          glVertex3f(-0.40f, pt+0.13f, -1.80f);
        glEnd();

    } else {
        // ---- Boeing 737 cockpit ----
        float pz  = -1.45f;
        float pzt = -1.18f;
        float pt  = -0.34f;
        float pb  = -0.80f;
        float pw  =  0.88f;
        // Main panel
        CPANEL_QUAD(-pw,pb,pz, pw,pb,pz, pw,pt,pzt, -pw,pt,pzt, 0.10f,0.10f,0.11f)

        // Glareshield
        CPANEL_QUAD(-pw,pt,pzt, pw,pt,pzt, pw,pt+0.06f,-1.10f, -pw,pt+0.06f,-1.10f, 0.06f,0.06f,0.07f)

        // A-pillars
        float apz0=-0.55f, apz1=-2.2f, apy0=-0.10f, apy1=0.48f, apx=0.88f, apt2=0.06f;
        CPANEL_QUAD(-apx,apy0,apz0, -apx+apt2,apy0,apz0, -apx+apt2,apy1,apz1, -apx,apy1,apz1, 0.12f,0.12f,0.13f)
        CPANEL_QUAD( apx-apt2,apy0,apz0,  apx,apy0,apz0,  apx,apy1,apz1,  apx-apt2,apy1,apz1, 0.12f,0.12f,0.13f)

        // Side consoles
        CPANEL_QUAD(-apx,pb,apz0, -apx,pt,apz1, -apx,apy1,apz1, -apx,pb,apz1, 0.09f+ambR*0.3f,0.09f+ambG*0.3f,0.10f+ambB*0.3f)
        CPANEL_QUAD( apx,pt,apz1,  apx,pb,apz0,  apx,pb,apz1,  apx,apy1,apz1, 0.09f+ambR*0.3f,0.09f+ambG*0.3f,0.10f+ambB*0.3f)

        // Left EFIS/MFD (blue navigation display)
        float mfdzF = pzt+0.003f;
        glColor3f(0.04f,0.18f,0.28f);
        glBegin(GL_QUADS);
          glVertex3f(-0.70f,pb+0.08f,pz+0.003f); glVertex3f(-0.25f,pb+0.08f,pz+0.003f);
          glVertex3f(-0.25f,pt-0.03f,mfdzF);      glVertex3f(-0.70f,pt-0.03f,mfdzF);
        glEnd();
        // Left EFIS content — magenta heading arc
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.9f,0.2f,0.9f,0.8f);
        float mfcx=-0.475f, mfcy=pb+0.14f, mfcz=pz+0.005f;
        glBegin(GL_LINE_STRIP);
        for(int k=-6;k<=6;k++){ float a=k*0.18f; glVertex3f(mfcx+sinf(a)*0.12f, mfcy+cosf(a)*0.12f, mfcz); }
        glEnd();
        glDisable(GL_BLEND);

        // Right EFIS (terrain/weather green)
        glColor3f(0.04f,0.22f,0.10f);
        glBegin(GL_QUADS);
          glVertex3f( 0.25f,pb+0.08f,pz+0.003f); glVertex3f( 0.70f,pb+0.08f,pz+0.003f);
          glVertex3f( 0.70f,pt-0.03f,mfdzF);      glVertex3f( 0.25f,pt-0.03f,mfdzF);
        glEnd();

        // Centre ECAM (amber/white)
        glColor3f(0.10f,0.08f,0.03f);
        glBegin(GL_QUADS);
          glVertex3f(-0.18f,pb+0.08f,pz+0.003f); glVertex3f( 0.18f,pb+0.08f,pz+0.003f);
          glVertex3f( 0.18f,pt-0.06f,mfdzF);      glVertex3f(-0.18f,pt-0.06f,mfdzF);
        glEnd();

        // Dual yokes
        float ycol=player.roll*0.06f, ypush=-player.pitch*0.05f;
        float yuy=pt+0.03f+ypush, yuz=pzt+0.04f;
        // Left yoke stem
        glColor3f(0.16f,0.16f,0.18f);
        glBegin(GL_QUADS);
          glVertex3f(-0.50f-0.02f,-0.78f,-0.80f); glVertex3f(-0.50f+0.02f,-0.78f,-0.80f);
          glVertex3f(ycol-0.50f+0.02f,yuy,yuz);    glVertex3f(ycol-0.50f-0.02f,yuy,yuz);
        glEnd();
        // Left yoke bar
        glBegin(GL_QUADS);
          glVertex3f(ycol-0.72f,yuy-0.015f,yuz); glVertex3f(ycol-0.30f,yuy-0.015f,yuz);
          glVertex3f(ycol-0.30f,yuy+0.015f,yuz); glVertex3f(ycol-0.72f,yuy+0.015f,yuz);
        glEnd();
        // Right yoke stem
        glBegin(GL_QUADS);
          glVertex3f( 0.50f-0.02f,-0.78f,-0.80f); glVertex3f( 0.50f+0.02f,-0.78f,-0.80f);
          glVertex3f(ycol+0.50f+0.02f,yuy,yuz);    glVertex3f(ycol+0.50f-0.02f,yuy,yuz);
        glEnd();
        // Right yoke bar
        glBegin(GL_QUADS);
          glVertex3f(ycol+0.30f,yuy-0.015f,yuz); glVertex3f(ycol+0.72f,yuy-0.015f,yuz);
          glVertex3f(ycol+0.72f,yuy+0.015f,yuz); glVertex3f(ycol+0.30f,yuy+0.015f,yuz);
        glEnd();
    }
#undef CPANEL_QUAD

    // ---- Instrument gauges (all plane types) ----
    // 6 gauges in a row on the instrument sub-panel
    // Positions in eye space
    float gRowZ;  // Z of gauge face
    float gRowY;  // Y centre of gauge row
    float gPW;    // panel half-width for gauge positioning
    if(player.type==PLANE_FIGHTER){ gRowZ=-1.38f; gRowY=-0.58f; gPW=0.50f; }
    else if(player.type==PLANE_PROP){ gRowZ=-1.33f; gRowY=-0.60f; gPW=0.34f; }
    else { gRowZ=-1.43f; gRowY=-0.58f; gPW=0.60f; }

    struct { float val; float needleScale; float r,g,b; } gauges[6] = {
        { player.y,                          0.008f, 0.80f,0.80f,0.95f }, // ALT
        { airspeed*6.28318f,                 1.000f, 0.95f,0.85f,0.65f }, // SPD (raw radians → sweep)
        { player.pitch*(180.0f/(float)M_PI), 0.035f, 0.30f,1.00f,0.35f }, // ATT (artificial horizon proxy)
        { player.vy*2.5f,                    0.300f, 0.65f,0.95f,0.75f }, // VSI
        { player.yaw,                        1.000f, 0.95f,0.95f,0.45f }, // HDG (raw radians)
        { player.roll*(180.0f/(float)M_PI),  0.035f, 0.95f,0.65f,0.65f }, // BANK
    };
    int numG=6;
    float gSpacing = (2.0f*gPW) / (numG-1);
    float gr=0.060f; // gauge radius

    for(int gi=0;gi<numG;gi++){
        float cx = -gPW + gi*gSpacing;
        float cy = gRowY;
        float cz = gRowZ;
        // Bezel ring
        glColor3f(0.30f,0.30f,0.33f);
        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(cx,cy,cz);
        for(int k=0;k<=18;k++){
            float a=k*2.0f*(float)M_PI/18;
            glVertex3f(cx+cosf(a)*gr, cy+sinf(a)*gr, cz);
        }
        glEnd();
        // Face
        glColor3f(0.04f,0.04f,0.07f);
        float fr=gr*0.87f;
        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(cx,cy,cz-0.0015f);
        for(int k=0;k<=18;k++){
            float a=k*2.0f*(float)M_PI/18;
            glVertex3f(cx+cosf(a)*fr, cy+sinf(a)*fr, cz-0.0015f);
        }
        glEnd();
        // Tick marks (12)
        glColor3f(0.55f,0.55f,0.60f);
        glBegin(GL_LINES);
        for(int k=0;k<12;k++){
            float a=k*2.0f*(float)M_PI/12;
            float ir = fr*(k%3==0?0.65f:0.75f);
            glVertex3f(cx+cosf(a)*ir,    cy+sinf(a)*ir,    cz-0.002f);
            glVertex3f(cx+cosf(a)*fr*0.95f, cy+sinf(a)*fr*0.95f, cz-0.002f);
        }
        glEnd();
        // Needle
        float needleAng = gauges[gi].val * gauges[gi].needleScale;
        glColor3f(gauges[gi].r, gauges[gi].g, gauges[gi].b);
        glLineWidth(1.5f);
        glBegin(GL_LINES);
        glVertex3f(cx, cy, cz-0.003f);
        glVertex3f(cx+sinf(needleAng)*fr*0.80f, cy+cosf(needleAng)*fr*0.80f, cz-0.003f);
        glEnd();
        glLineWidth(1.0f);
        // Centre dot
        glColor3f(0.80f,0.80f,0.85f);
        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(cx,cy,cz-0.004f);
        for(int k=0;k<=8;k++){
            float a=k*2.0f*(float)M_PI/8;
            glVertex3f(cx+cosf(a)*fr*0.08f, cy+sinf(a)*fr*0.08f, cz-0.004f);
        }
        glEnd();
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_FOG);
    glEnable(GL_TEXTURE_2D);
    glPopMatrix();
}

// -------- Render Scene --------
void renderScene(){
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // Update sun / time-of-day
    updateSunLight();

    // Camera
    float camX, camY, camZ, lookX, lookY, lookZ;
    float upX=0, upY=1, upZ=0;
    if(inStoreIdx >= 0 && inStoreIdx < numStores && !inPlane){
        // Interior camera: follow storeWalkerX/Z in store local space
        Store *cs = &stores[inStoreIdx];
        float ch2 = cosf(cs->heading), sh2 = sinf(cs->heading);
        // Transform local (lx,lz) to world: wx + sh2*lx + ch2*lz, wz + ch2*lx - sh2*lz
        // Wait — standard rotation: world_x = wx + cos(h)*lx + sin(h)*lz ... let me use apLocalToWorld convention
        // apLocalToWorld: wx + cos(h)*fwd + sin(h)*side, wz - sin(h)*fwd + cos(h)*side
        // Here fwd=storeWalkerX maps to X-axis, side=storeWalkerZ maps to Z-axis — use directly:
        float lx = storeWalkerX, lz = storeWalkerZ;
        float sws = sinf(storeWalkerYaw), cws = cosf(storeWalkerYaw);
        camX  = cs->wx + ch2*lx + sh2*lz;
        camY  = cs->wy + WALKER_EYE_H;
        camZ  = cs->wz - sh2*lx + ch2*lz;
        // Look direction is storeWalkerYaw rotated by store heading
        float lwx = sws, lwz = cws; // local look forward
        lookX = camX + (ch2*lwx + sh2*lwz);
        lookY = camY;
        lookZ = camZ + (-sh2*lwx + ch2*lwz);
        cockpitView = false;
    } else if(!inPlane){
        cockpitView = false;
        float sw = sinf(walkerYaw), cw = cosf(walkerYaw);
        camX = walkerX; camY = walkerY + WALKER_EYE_H; camZ = walkerZ;
        lookX = walkerX + sw; lookY = walkerY + WALKER_EYE_H; lookZ = walkerZ + cw;
    } else if(isPassenger){
        cockpitView = false;
        float sYaw = sinf(player.yaw), cYaw = cosf(player.yaw);
        float sc2 = PLANE_DEFS[player.type].scale;
        camX = player.x + cYaw*sc2*0.5f;
        camY = player.y + sc2*0.2f;
        camZ = player.z + (-sYaw)*sc2*0.5f;
        lookX = camX + sYaw*20.0f; lookY = camY; lookZ = camZ + cYaw*20.0f;
    } else if(cockpitView && inPlane){
        // First-person cockpit: camera at pilot's eye, looking out nose
        float sYaw   = sinf(player.yaw),   cYaw   = cosf(player.yaw);
        float sPitch = sinf(player.pitch),  cPitch = cosf(player.pitch);
        float sRoll  = sinf(player.roll),   cRoll  = cosf(player.roll);
        camX = player.x + sYaw*0.5f; camY = player.y + 1.2f; camZ = player.z + cYaw*0.5f;
        // Forward vector along plane heading + pitch
        float fwdX = sYaw*cPitch, fwdY = sPitch, fwdZ = cYaw*cPitch;
        lookX = camX + fwdX*10; lookY = camY + fwdY*10; lookZ = camZ + fwdZ*10;
        // Up vector rolled with plane
        upX = -sYaw*sPitch*cRoll + cYaw*sRoll;
        upY =  cPitch*cRoll;
        upZ = -cYaw*sPitch*cRoll - sYaw*sRoll;
    } else {
        float sYaw = sinf(player.yaw), cYaw = cosf(player.yaw);
        camX = player.x - sYaw*10; camY = player.y + 3; if(camY<1.5f) camY=1.5f;
        camZ = player.z - cYaw*10;
        lookX = player.x + sYaw*30; lookY = player.y; lookZ = player.z + cYaw*30;
    }
    gluLookAt(camX,camY,camZ, lookX,lookY,lookZ, upX,upY,upZ);

    // Sky dome (drawn first, behind everything)
    renderSkyDome(camX, camY, camZ);

    pthread_mutex_lock(&terrainMutex);
    renderTerrain();
    renderAllAirports();
    pthread_mutex_unlock(&terrainMutex);
    renderAllTornados();

    // Parked planes at gates — cull beyond 400 units
    float eyeX = inPlane ? player.x : walkerX;
    float eyeZ = inPlane ? player.z : walkerZ;
    for(int i=0;i<numParked;i++){
        if(!parkedPlanes[i].active) continue;
        float ddx=parkedPlanes[i].wx-eyeX, ddz=parkedPlanes[i].wz-eyeZ;
        if(ddx*ddx+ddz*ddz > 800.0f*800.0f) continue;
        drawPlaneModel(parkedPlanes[i].wx, parkedPlanes[i].wy, parkedPlanes[i].wz,
                       0, parkedPlanes[i].heading, 0, 1.0f, parkedPlanes[i].type);
    }

    renderNpcs();
    renderMissiles();

    // Player plane — skip in cockpit view (you're inside it)
    if(!isPassenger && !cockpitView)
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

    renderTrees();
    renderSuburbs();
    renderStores();
    renderExplosions();
    renderClouds();
    renderStoreOverlay();

    // Cockpit overlay (drawn after world, before HUD)
    if(cockpitView && inPlane && !isPassenger)
        renderCockpit();

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

    // Health bar (bottom-left, always visible)
    {
        int hbx=10, hby=8, hbw=120, hbh=10;
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.15f,0.15f,0.15f,0.7f);
        glBegin(GL_QUADS);
            glVertex2i(hbx,hby); glVertex2i(hbx+hbw,hby);
            glVertex2i(hbx+hbw,hby+hbh); glVertex2i(hbx,hby+hbh);
        glEnd();
        float hf = playerHealth;
        float hr2 = hf<0.5f ? 1.0f : 2.0f*(1-hf);
        float hg2 = hf>0.5f ? 1.0f : 2.0f*hf;
        glColor4f(hr2, hg2, 0.0f, 0.9f);
        glBegin(GL_QUADS);
            glVertex2i(hbx,hby); glVertex2i(hbx+(int)(hbw*hf),hby);
            glVertex2i(hbx+(int)(hbw*hf),hby+hbh); glVertex2i(hbx,hby+hbh);
        glEnd();
        glDisable(GL_BLEND); glEnable(GL_LIGHTING);
        glColor3f(0.8f,0.8f,0.8f);
        HUD12(hbx, hby+hbh+2, "HP");
        // Active effect indicators
        int ex = hbx + hbw + 8;
        if(effectHealTimer > 0){
            glColor3f(0.3f,1.0f,0.3f);
            snprintf(buf,sizeof(buf),"HEAL %.0fs", effectHealTimer);
            HUD12(ex, hby+hbh+2, buf); ex += 70;
        }
        if(effectSpeedTimer > 0){
            glColor3f(1.0f,0.85f,0.1f);
            snprintf(buf,sizeof(buf),"SPD %.0fs", effectSpeedTimer);
            HUD12(ex, hby+hbh+2, buf);
        }
        // Inventory summary
        bool hasItems = false;
        for(int iv=0;iv<INV_SLOTS;iv++) if(inventory[iv]>0){ hasItems=true; break; }
        if(hasItems){
            glColor3f(0.7f,0.9f,0.7f);
            char ibuf[64]="BAG:";
            for(int iv=0;iv<INV_SLOTS;iv++)
                if(inventory[iv]>0){ char tmp[8]; snprintf(tmp,sizeof(tmp)," %d×%d",iv+1,inventory[iv]); strcat(ibuf,tmp); }
            HUD12(hbx, hby-14, ibuf);
        }
    }

    // Gear warning: airborne, gear up, low altitude
    if(inPlane && !onGround && !gearDeployed && aglHud < 40.0f){
        glColor3f(1,0.1f,0.1f);
        HUD(resWidth/2-80, resHeight/2-60, "GEAR NOT DOWN");
    }

    // Server hosting info — top right corner
    if(isServer && !clientConnected){
        glColor3f(0.4f,0.85f,1.0f);
        sprintf(buf, "Hosting on %s:5000  |  Share this IP", localIP);
        int sw=0; for(const char *c=buf;*c;c++) sw+=glutBitmapWidth(GLUT_BITMAP_HELVETICA_12,*c);
        HUD12(resWidth-sw-8, resHeight-18, buf);
    } else if(isServer && clientConnected){
        glColor3f(0.3f,1.0f,0.4f);
        HUD12(resWidth-120, resHeight-18, "Player connected");
    }

    // On-foot / enter-exit prompts
    if(!inPlane){
        float dx = walkerX-player.x, dz = walkerZ-player.z;
        float distToPlane = sqrtf(dx*dx+dz*dz);
        glColor3f(0.6f,1.0f,0.6f);
        // Check proximity to stores
        bool nearStore = false;
        for(int si=0;si<numStores;si++){
            float sdx=walkerX-stores[si].wx, sdz=walkerZ-stores[si].wz;
            if(sqrtf(sdx*sdx+sdz*sdz) <= STORE_ENTER_DIST+4.0f){
                const char *snames[]={"Albertsons","Arby's","Long John Silver's"};
                snprintf(buf, sizeof(buf), "[F] Enter %s", snames[stores[si].type]);
                HUD(resWidth/2-90, resHeight/2-40, buf);
                nearStore = true; break;
            }
        }
        if(!nearStore){
            if(distToPlane <= ENTER_DIST)
                HUD(resWidth/2-80, resHeight/2-40, "[F] Enter aircraft");
            else {
                sprintf(buf, "On foot  |  plane %.0f m away", distToPlane);
                HUD(10, resHeight-78, buf);
            }
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

    // ---- Chat overlay — own 2D ortho block ----
    // Decay timers
    for(int i=0;i<chatCount;){
        chatLog[i].timer -= DT;
        if(chatLog[i].timer <= 0.0f && !chatOpen){
            memmove(&chatLog[i], &chatLog[i+1], sizeof(ChatMsg)*(chatCount-i-1));
            chatCount--;
        } else i++;
    }
    {
        // Enter 2D
        glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
        gluOrtho2D(0, resWidth, 0, resHeight);
        glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        float cx = 12.0f;
        // Input box at bottom, messages stacked above it
        float inputY  = 14.0f;   // bottom of input box
        float inputH  = 22.0f;
        float msgBase = chatOpen ? (inputY + inputH + 4.0f) : 14.0f;
        int   show    = chatCount > 8 ? 8 : chatCount;

        // Draw messages
        for(int i = 0; i < show; i++){
            ChatMsg *m = &chatLog[chatCount - show + i];
            float alpha = chatOpen ? 0.92f : fminf(m->timer / 1.5f, 1.0f) * 0.88f;
            if(alpha <= 0.01f) continue;
            float my = msgBase + i * 18.0f;
            // Text shadow
            glColor4f(0,0,0,alpha*0.6f);
            glRasterPos2f(cx+1, my-1);
            for(const char *c = m->text; *c; c++)
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
            // Text
            glColor4f(0.92f, 0.96f, 1.0f, alpha);
            glRasterPos2f(cx, my);
            for(const char *c = m->text; *c; c++)
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
        }

        // Input box
        if(chatOpen){
            // Background
            menuRect(cx-2, inputY, 340, inputH, 0.04f,0.07f,0.14f, 0.88f);
            menuRectOutline(cx-2, inputY, 340, inputH, 0.3f,0.6f,1.0f);
            // Prompt + typed text + cursor
            char inputDisplay[CHAT_MSG_LEN+6];
            snprintf(inputDisplay, sizeof(inputDisplay), "> %s|", chatInput);
            glColor4f(1,1,1,1);
            glRasterPos2f(cx+4, inputY+7);
            for(const char *c = inputDisplay; *c; c++)
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
            // Hint
            glColor4f(0.5f,0.7f,1.0f,0.6f);
            glRasterPos2f(cx+4, inputY+inputH+2);
            const char *hint = "Enter=send  Esc=cancel";
            for(const char *c = hint; *c; c++)
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
        }

        // Exit 2D
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
        glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
    }
    // Swap is handled by the caller so menus can overlay before presenting
}

// -------- Missiles --------
static void updateMissiles(float dt){
    if(missileCooldown > 0.0f) missileCooldown -= dt;

    for(int i=0;i<MAX_MISSILES;i++){
        Missile *m = &missiles[i];
        if(!m->active) continue;

        float step = MISSILE_SPEED * dt;
        m->x += m->vx * dt;
        m->y += m->vy * dt;
        m->z += m->vz * dt;
        m->dist += step;

        // Self-destruct if out of range or hit the ground
        if(m->dist > MISSILE_RANGE || m->y < terrainHeightAt(m->x, m->z)){
            triggerExplosion(m->x, m->y, m->z);
            m->active = false;
            continue;
        }

        // Check NPC planes
        for(int n=0;n<numNpcs;n++){
            Npc *np = &npcs[n];
            if(!np->alive) continue;
            if(np->state != NPC_FLY && np->state != NPC_APPROACH &&
               np->state != NPC_TAKEOFF && np->state != NPC_LAND) continue;
            float dx=m->x-np->px, dy=m->y-np->py, dz=m->z-np->pz;
            if(dx*dx+dy*dy+dz*dz < MISSILE_HIT_R*MISSILE_HIT_R){
                triggerExplosionEx(np->px, np->py, np->pz, 2.5f);
                np->alive = false;
                m->active = false;
                break;
            }
        }
    }
}

static void fireMissile(){
    if(missileCooldown > 0.0f) return;
    if(player.type != PLANE_FIGHTER) return;
    // Find free slot
    for(int i=0;i<MAX_MISSILES;i++){
        if(missiles[i].active) continue;
        Missile *m = &missiles[i];
        float sp = sinf(player.pitch), cp = cosf(player.pitch);
        float sy = sinf(player.yaw),   cy = cosf(player.yaw);
        // Direction: forward along plane heading + pitch
        m->vx = sy * cp * MISSILE_SPEED;
        m->vy = sp      * MISSILE_SPEED;
        m->vz = cy * cp * MISSILE_SPEED;
        // Spawn slightly ahead of nose
        m->x = player.x + sy*3.0f;
        m->y = player.y + sp*3.0f;
        m->z = player.z + cy*3.0f;
        m->dist = 0.0f;
        m->active = true;
        missileCooldown = MISSILE_COOLDOWN;
        sndPlay(SND_MISSILE);
        return;
    }
}

static void renderMissiles(){
    for(int i=0;i<MAX_MISSILES;i++){
        Missile *m = &missiles[i];
        if(!m->active) continue;

        glPushMatrix();
        glTranslatef(m->x, m->y, m->z);

        // Align missile along its velocity direction
        float len = sqrtf(m->vx*m->vx + m->vy*m->vy + m->vz*m->vz);
        if(len > 0.001f){
            float dx=m->vx/len, dy=m->vy/len, dz=m->vz/len;
            float pitch = asinf(dy);
            float yaw   = atan2f(dx, dz);
            glRotatef(yaw   * 180.0f / (float)M_PI, 0,1,0);
            glRotatef(-pitch * 180.0f / (float)M_PI, 1,0,0);
        }

        // Body — white/grey cylinder
        glColor3f(0.85f,0.85f,0.88f);
        drawOctSegment(0.0f,0.0f, 0.8f, 0.06f,0.06f, 0.6f);
        drawOctSegment(0.06f,0.06f, 0.6f, 0.06f,0.06f,-0.4f);
        drawOctSegment(0.06f,0.06f,-0.4f, 0.0f,0.0f,-0.6f);
        drawOctCap(0.0f,0.0f, 0.8f, 0,0,1);
        // Nose cone — dark
        glColor3f(0.20f,0.20f,0.22f);
        drawOctSegment(0.0f,0.0f, 1.0f, 0.06f,0.06f, 0.8f);
        drawOctCap(0.0f,0.0f, 1.0f, 0,0,1);
        // 4 tail fins
        glColor3f(0.70f,0.70f,0.72f);
        for(int f=0;f<4;f++){
            glPushMatrix();
            glRotatef(f*90.0f, 0,0,1);
            glBegin(GL_TRIANGLES);
                glNormal3f(1,0,0);
                glVertex3f( 0.005f,0.06f,-0.20f);
                glVertex3f( 0.005f,0.06f,-0.55f);
                glVertex3f( 0.005f,0.22f,-0.50f);
                glNormal3f(-1,0,0);
                glVertex3f(-0.005f,0.06f,-0.20f);
                glVertex3f(-0.005f,0.22f,-0.50f);
                glVertex3f(-0.005f,0.06f,-0.55f);
            glEnd();
            glPopMatrix();
        }
        // Rocket exhaust glow (additive orange)
        glEnable(GL_BLEND); glBlendFunc(GL_ONE, GL_ONE);
        glDisable(GL_LIGHTING);
        float flicker = 0.8f + 0.2f*(float)(rand()%100)/100.0f;
        glColor3f(1.0f*flicker, 0.45f*flicker, 0.0f);
        glBegin(GL_TRIANGLE_FAN);
            glVertex3f(0.0f, 0.0f, -0.8f);
            for(int k=0;k<=8;k++){
                float a=k*6.28318f/8;
                glVertex3f(cosf(a)*0.05f, sinf(a)*0.05f, -0.6f);
            }
        glEnd();
        glEnable(GL_LIGHTING);
        glDisable(GL_BLEND);

        glPopMatrix();
    }
}

// -------- NPCs --------
static void updateNpcs(float dt){
    for(int i=0;i<numNpcs;i++){
        Npc *n = &npcs[i];
        if(!n->alive) continue;
        Airport *ap = &airports[n->airportIdx];

        switch(n->state){

        case NPC_WALK: {
            // Wander around the apron
            n->wanderTimer -= dt;
            if(n->wanderTimer <= 0.0f){
                n->wanderTimer = 1.5f + ((float)rand()/RAND_MAX)*3.0f;
                n->wanderYaw   = n->yaw + ((float)rand()/RAND_MAX - 0.5f)*2.5f;
            }
            // Steer toward wander heading
            float dyaw = n->wanderYaw - n->yaw;
            while(dyaw >  (float)M_PI) dyaw -= 2.0f*(float)M_PI;
            while(dyaw < -(float)M_PI) dyaw += 2.0f*(float)M_PI;
            n->yaw += dyaw * 3.0f * dt;

            // Keep on apron — bias back toward home if too far
            float hx = n->x - n->homeX, hz = n->z - n->homeZ;
            float hdist = sqrtf(hx*hx+hz*hz);
            float speed = 2.5f;
            if(hdist > 18.0f){
                float toHomeYaw = atan2f(n->homeX-n->x, n->homeZ-n->z);
                float d2 = toHomeYaw - n->yaw;
                while(d2 >  (float)M_PI) d2 -= 2.0f*(float)M_PI;
                while(d2 < -(float)M_PI) d2 += 2.0f*(float)M_PI;
                n->yaw += d2 * 4.0f * dt;
                speed = 4.0f;
            }
            n->x += sinf(n->yaw)*speed*dt;
            n->z += cosf(n->yaw)*speed*dt;
            n->y  = ap->groundY;

            // After walk timer expires, try to find a parked plane to board
            n->stateTimer -= dt;
            if(n->stateTimer <= 0.0f){
                // Find nearest active parked plane at this airport
                int best = -1;
                float bestD = 1e9f;
                for(int p=0;p<numParked;p++){
                    if(!parkedPlanes[p].active) continue;
                    float dx=parkedPlanes[p].wx-n->homeX, dz=parkedPlanes[p].wz-n->homeZ;
                    if(sqrtf(dx*dx+dz*dz)>220.0f) continue; // not our airport
                    dx=parkedPlanes[p].wx-n->x; dz=parkedPlanes[p].wz-n->z;
                    float d=sqrtf(dx*dx+dz*dz);
                    if(d<bestD){ bestD=d; best=p; }
                }
                if(best>=0){
                    n->wpIdx   = best; // reuse wpIdx as target parked plane
                    n->state   = NPC_BOARD;
                    n->stateTimer = 15.0f; // timeout
                } else {
                    n->stateTimer = 5.0f + ((float)rand()/RAND_MAX)*10.0f;
                }
            }
            break;
        }

        case NPC_BOARD: {
            // Walk toward target parked plane
            int p = n->wpIdx;
            if(p<0||p>=numParked||!parkedPlanes[p].active){
                n->state=NPC_WALK; n->stateTimer=5.0f; break;
            }
            float tx=parkedPlanes[p].wx, tz=parkedPlanes[p].wz;
            float dx=tx-n->x, dz=tz-n->z;
            float d=sqrtf(dx*dx+dz*dz);
            if(d>0.3f){
                n->yaw = atan2f(dx,dz);
                float spd = d>2.0f ? 4.5f : 1.5f;
                n->x += sinf(n->yaw)*spd*dt;
                n->z += cosf(n->yaw)*spd*dt;
                n->y  = ap->groundY;
            }
            n->stateTimer -= dt;
            if(d < 3.0f || n->stateTimer<=0.0f){
                if(d < 3.0f){
                    // Board the plane — start respawn countdown
                    parkedPlanes[p].active = false;
                    parkedPlanes[p].respawnTimer = PLANE_RESPAWN_TIME;
                    n->ptype    = parkedPlanes[p].type;
                    n->px       = parkedPlanes[p].wx;
                    n->py       = parkedPlanes[p].wy;
                    n->pz       = parkedPlanes[p].wz;
                    n->pyaw     = parkedPlanes[p].heading;
                    n->ppitch   = 0.0f; n->proll = 0.0f;
                    n->pspeed   = 0.0f;
                    n->pthrottle= 0.0f;
                    n->state    = NPC_TAKEOFF;
                    n->stateTimer = 30.0f;
                } else {
                    n->state=NPC_WALK; n->stateTimer=5.0f;
                }
            }
            break;
        }

        case NPC_TAKEOFF: {
            float groundY = ap->groundY;
            float maxSpd  = PLANE_DEFS[n->ptype].maxSpeed;
            float tkSpd   = PLANE_DEFS[n->ptype].takeoffSpeed;
            float accel   = PLANE_DEFS[n->ptype].accel;
            n->pthrottle  = 1.0f;
            n->pspeed    += accel * dt;
            if(n->pspeed > maxSpd) n->pspeed = maxSpd;
            // Roll along heading
            n->px += sinf(n->pyaw)*cosf(n->ppitch)*n->pspeed*dt;
            n->pz += cosf(n->pyaw)*cosf(n->ppitch)*n->pspeed*dt;
            if(n->pspeed >= tkSpd){
                n->ppitch += 0.8f*dt;
                if(n->ppitch > 0.25f) n->ppitch = 0.25f;
            }
            n->py += sinf(n->ppitch)*n->pspeed*dt;
            // Ground floor
            float tkGnd = terrainHeightAt(n->px, n->pz);
            if(n->py < tkGnd + 1.0f) n->py = tkGnd + 1.0f;
            if(n->py > groundY + 60.0f || n->stateTimer<=0.0f){
                n->ppitch = 0.05f;
                n->state  = NPC_FLY;
                n->wpIdx  = 0;
                n->stateTimer = 60.0f + ((float)rand()/RAND_MAX)*60.0f;
            }
            n->stateTimer -= dt;
            break;
        }

        case NPC_FLY: {
            float maxSpd = PLANE_DEFS[n->ptype].maxSpeed;
            float accel  = PLANE_DEFS[n->ptype].accel;
            // Circuit of 4 waypoints around the home airport
            float cx=ap->wx, cz=ap->wz;
            float alt = ap->groundY + 120.0f;
            float offsets[4][2] = {{400,400},{-400,400},{-400,-400},{400,-400}};
            int wi = n->wpIdx % 4;
            n->wpX = cx + offsets[wi][0];
            n->wpZ = cz + offsets[wi][1];
            n->wpAlt = alt;
            float dx=n->wpX-n->px, dz=n->wpZ-n->pz;
            float hdist=sqrtf(dx*dx+dz*dz);
            if(hdist > 5.0f){
                float tgtYaw = atan2f(dx,dz);
                float dyaw = tgtYaw - n->pyaw;
                while(dyaw >  (float)M_PI) dyaw -= 2.0f*(float)M_PI;
                while(dyaw < -(float)M_PI) dyaw += 2.0f*(float)M_PI;
                n->pyaw += dyaw * 1.5f * dt;
                n->proll = clampf(-dyaw*1.2f, -0.5f, 0.5f);
            } else {
                n->wpIdx++;
            }
            // Altitude hold
            float dalt = n->wpAlt - n->py;
            n->ppitch = clampf(dalt * 0.02f, -0.3f, 0.25f);
            n->pspeed += accel * 0.5f * dt;
            if(n->pspeed > maxSpd*0.85f) n->pspeed = maxSpd*0.85f;
            n->px += sinf(n->pyaw)*cosf(n->ppitch)*n->pspeed*dt;
            n->py += sinf(n->ppitch)*n->pspeed*dt;
            n->pz += cosf(n->pyaw)*cosf(n->ppitch)*n->pspeed*dt;
            // Ground collision
            { float flyGnd = terrainHeightAt(n->px, n->pz) + 1.5f;
              if(n->py < flyGnd){ n->py = flyGnd; if(n->ppitch < 0.0f) n->ppitch = 0.1f; } }
            n->stateTimer -= dt;
            if(n->stateTimer <= 0.0f){
                n->state = NPC_APPROACH;
                n->stateTimer = 30.0f;
            }
            break;
        }

        case NPC_APPROACH: {
            // Fly toward airport at descending glide
            float tgtX=ap->wx, tgtZ=ap->wz;
            float tgtY=ap->groundY + 2.0f;
            float dx=tgtX-n->px, dy=tgtY-n->py, dz=tgtZ-n->pz;
            float dist=sqrtf(dx*dx+dy*dy+dz*dz);
            if(dist>1.0f){
                float tgtYaw=atan2f(dx,dz);
                float dyaw=tgtYaw-n->pyaw;
                while(dyaw >  (float)M_PI) dyaw -= 2.0f*(float)M_PI;
                while(dyaw < -(float)M_PI) dyaw += 2.0f*(float)M_PI;
                n->pyaw += dyaw*2.0f*dt;
                n->proll = clampf(-dyaw*1.5f,-0.5f,0.5f);
                n->ppitch = clampf(dy/dist*0.6f,-0.3f,0.2f);
                float maxSpd=PLANE_DEFS[n->ptype].maxSpeed;
                float tkSpd=PLANE_DEFS[n->ptype].takeoffSpeed;
                float tgtSpd = tkSpd*1.3f;
                if(n->pspeed > tgtSpd) n->pspeed -= 3.0f*dt;
                if(n->pspeed < tgtSpd*0.7f) n->pspeed = tgtSpd*0.7f;
                n->px += sinf(n->pyaw)*cosf(n->ppitch)*n->pspeed*dt;
                n->py += sinf(n->ppitch)*n->pspeed*dt;
                n->pz += cosf(n->pyaw)*cosf(n->ppitch)*n->pspeed*dt;
                float gnd=terrainHeightAt(n->px,n->pz);
                if(n->py < gnd+1.5f){ n->py=gnd+1.0f; n->state=NPC_LAND; n->stateTimer=8.0f; }
            } else {
                n->state=NPC_LAND; n->stateTimer=8.0f;
            }
            n->stateTimer-=dt;
            if(n->stateTimer<=0.0f){ n->state=NPC_WALK; n->stateTimer=5.0f; }
            break;
        }

        case NPC_LAND: {
            // Roll to stop
            n->pspeed -= PLANE_DEFS[n->ptype].brakeDecel * dt;
            if(n->pspeed < 0.0f) n->pspeed = 0.0f;
            float gnd = terrainHeightAt(n->px,n->pz);
            n->py = gnd + 1.0f;
            n->ppitch *= 0.9f; n->proll *= 0.9f;
            n->px += sinf(n->pyaw)*n->pspeed*dt;
            n->pz += cosf(n->pyaw)*n->pspeed*dt;
            n->stateTimer -= dt;
            if(n->pspeed<=0.0f || n->stateTimer<=0.0f){
                // Repark: reuse an inactive slot if available, else append
                int slot=-1;
                for(int p=0;p<numParked;p++) if(!parkedPlanes[p].active){ slot=p; break; }
                if(slot<0 && numParked<MAX_PARKED) slot=numParked++;
                if(slot>=0){
                    ParkedPlane *pp=&parkedPlanes[slot];
                    pp->wx=n->px; pp->wy=n->py; pp->wz=n->pz;
                    pp->heading=n->pyaw; pp->active=true; pp->type=n->ptype;
                    pp->respawnTimer=0.0f;
                }
                // Deplane: walker pops out next to plane
                n->x = n->px + cosf(n->pyaw)*2.0f;
                n->z = n->pz - sinf(n->pyaw)*2.0f;
                n->y = terrainHeightAt(n->x,n->z);
                n->yaw = n->pyaw;
                n->state = NPC_DEPLANE;
                n->stateTimer = 2.0f + ((float)rand()/RAND_MAX)*3.0f;
            }
            break;
        }

        case NPC_DEPLANE: {
            // Walk away briefly then look for a different plane to swap to
            n->x += sinf(n->yaw)*2.5f*dt;
            n->z += cosf(n->yaw)*2.5f*dt;
            n->y  = terrainHeightAt(n->x,n->z);
            n->stateTimer -= dt;
            if(n->stateTimer<=0.0f){
                // Look for a nearby parked plane of a DIFFERENT type to swap to
                int swapTarget = -1;
                float bestSwapDist = 1e9f;
                for(int p=0;p<numParked;p++){
                    if(!parkedPlanes[p].active) continue;
                    if(parkedPlanes[p].type == n->ptype) continue; // skip same type
                    float pdx=parkedPlanes[p].wx-n->x, pdz=parkedPlanes[p].wz-n->z;
                    float pd=sqrtf(pdx*pdx+pdz*pdz);
                    if(pd < 220.0f && pd < bestSwapDist){ bestSwapDist=pd; swapTarget=p; }
                }
                if(swapTarget >= 0 && (rand()%3 != 0)){ // 67% chance to swap
                    n->wpIdx  = swapTarget;
                    n->state  = NPC_BOARD;
                    n->stateTimer = 20.0f;
                } else {
                    n->homeX = n->x; n->homeZ = n->z;
                    n->state = NPC_WALK;
                    n->stateTimer = 10.0f + ((float)rand()/RAND_MAX)*15.0f;
                    n->wanderTimer = 1.0f;
                    n->wanderYaw   = n->yaw;
                }
            }
            break;
        }

        } // switch
    } // for npcs

    // Plane respawn: inactive slots count down and re-activate with a fresh plane
    for(int p=0;p<numParked;p++){
        if(parkedPlanes[p].active) continue;
        parkedPlanes[p].respawnTimer -= dt;
        if(parkedPlanes[p].respawnTimer <= 0.0f){
            parkedPlanes[p].active = true;
            // Randomise the plane type on respawn
            parkedPlanes[p].type = (PlaneType)(rand()%3);
            parkedPlanes[p].respawnTimer = PLANE_RESPAWN_TIME;
        }
    }
}

static void renderNpcs(){
    float eyeX = inPlane ? player.x : walkerX;
    float eyeZ = inPlane ? player.z : walkerZ;
    for(int i=0;i<numNpcs;i++){
        Npc *n = &npcs[i];
        if(!n->alive) continue;
        switch(n->state){
        case NPC_WALK:
        case NPC_BOARD:
        case NPC_DEPLANE: {
            float dx=n->x-eyeX, dz=n->z-eyeZ;
            if(dx*dx+dz*dz > 600.0f*600.0f) break;
            drawWalkerModel(n->x, n->y, n->z, n->yaw);
            break;
        }
        case NPC_TAKEOFF:
        case NPC_FLY:
        case NPC_APPROACH:
        case NPC_LAND: {
            float dx=n->px-eyeX, dz=n->pz-eyeZ;
            if(dx*dx+dz*dz > 1200.0f*1200.0f) break;
            drawPlaneModel(n->px, n->py, n->pz, n->ppitch, n->pyaw, n->proll,
                           (n->state==NPC_LAND||n->state==NPC_TAKEOFF)?1.0f:0.5f, n->ptype);
            break;
        }
        }
    }
}

// -------- Physics --------
void updatePhysics(float dt){
    pthread_mutex_lock(&playerMutex);

    if(!inPlane){
        // ---- On-foot walker ----

        // If inside a store, walk around inside with collision
        if(inStoreIdx >= 0){
            float sws = sinf(storeWalkerYaw), cws = cosf(storeWalkerYaw);
            float mx = 0, mz = 0;
            float spd = WALKER_SPEED * (1.0f + effectSpeed);
            // W/S = forward/back, A/D = strafe left/right (mouse turns)
            if(keys[XK_w]){ mx += sws*spd*dt; mz += cws*spd*dt; }
            if(keys[XK_s]){ mx -= sws*spd*dt; mz -= cws*spd*dt; }
            if(keys[XK_a]){ mx -= cws*spd*dt; mz += sws*spd*dt; }
            if(keys[XK_d]){ mx += cws*spd*dt; mz -= sws*spd*dt; }
            float nx = storeWalkerX + mx;
            float nz = storeWalkerZ + mz;
            // Wall collision (store local bounds)
            float xLimit = SW_HALF - SWALL_THICK;
            float zFront = SDOOR_Z - SWALL_THICK;
            float zBack  = -(SD_BACK - SWALL_THICK);
            nx = clampf(nx, -xLimit, xLimit);
            nz = clampf(nz, zBack,   zFront);
            storeWalkerX = nx; storeWalkerZ = nz;

            // F to exit
            if(keys[XK_f]){
                if(!enterKeyHeld){
                    enterKeyHeld = true;
                    // Only allow exit when near the door
                    if(storeWalkerZ > SDOOR_Z - 2.5f){
                        inStoreIdx = -1;
                        chatAddMsg("[Store] Thanks for visiting! (press 1-5 anywhere to use items)");
                    } else {
                        chatAddMsg("[Store] Walk back to the entrance to leave.");
                    }
                }
            } else enterKeyHeld = false;

            // Decay effects inside store too
            if(effectHealTimer > 0){ effectHealTimer -= dt;
                playerHealth = clampf(playerHealth + effectHeal*dt, 0, 1); }
            if(effectSpeedTimer > 0){ effectSpeedTimer -= dt; }
            if(effectSpeedTimer <= 0) effectSpeed = 0;
            if(effectHealTimer  <= 0) effectHeal  = 0;

            pthread_mutex_unlock(&playerMutex);
            return;
        }

        float moveX = 0, moveZ = 0;
        float swY = sinf(walkerYaw), cwY = cosf(walkerYaw);
        // W/S = forward/back, A/D = strafe left/right
        if(keys[XK_w]){ moveX += swY*WALKER_SPEED*dt; moveZ += cwY*WALKER_SPEED*dt; }
        if(keys[XK_s]){ moveX -= swY*WALKER_SPEED*dt; moveZ -= cwY*WALKER_SPEED*dt; }
        if(keys[XK_a]){ moveX -= cwY*WALKER_SPEED*dt; moveZ += swY*WALKER_SPEED*dt; }
        if(keys[XK_d]){ moveX += cwY*WALKER_SPEED*dt; moveZ -= swY*WALKER_SPEED*dt; }
        walkerX += moveX;
        walkerZ += moveZ;
        walkerY  = terrainHeightAt(walkerX, walkerZ) + 1.0f;

        // F key: enter store, own plane, parked plane, or remote airliner
        if(keys[XK_f]){
            if(!enterKeyHeld){
                enterKeyHeld = true;
                bool acted = false;

                // Check stores first
                for(int si=0;si<numStores && !acted;si++){
                    float sdx = walkerX - stores[si].wx;
                    float sdz = walkerZ - stores[si].wz;
                    if(sqrtf(sdx*sdx+sdz*sdz) <= STORE_ENTER_DIST){
                        inStoreIdx       = si;
                        lastStoreType    = stores[si].type;
                        storeWalkerX     = 0.0f;
                        storeWalkerZ     = -(SD_BACK * 0.5f); // spawn in back half of store
                        storeWalkerYaw   = 0.0f; // face toward door (+Z direction)
                        // enterKeyHeld stays true — player must release F before exit logic fires
                        // Clear inventory on store change so items don't mix between stores
                        for(int iv=0;iv<INV_SLOTS;iv++) inventory[iv]=0;
                        const char *names[] = {"Albertsons","Arby's","Long John Silver's"};
                        char msg[96];
                        snprintf(msg, sizeof(msg), "[Store] Welcome to %s! Walk around, press 1-5 to order.",
                                 names[stores[si].type]);
                        chatAddMsg(msg);
                        acted = true;
                    }
                }

                if(!acted){
                    float dx = walkerX - player.x, dz = walkerZ - player.z;
                    if(sqrtf(dx*dx+dz*dz) <= ENTER_DIST){
                        isPassenger = false;
                        inPlane = true;
                        acted = true;
                    }
                }
                if(!acted){
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
                                parkedPlanes[i].respawnTimer = PLANE_RESPAWN_TIME;
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
    if(keys[XK_g]){ if(!gearKeyHeld && !onGround){ gearDeployed=!gearDeployed; gearKeyHeld=true; sndPlay(SND_GEAR); } }
    else gearKeyHeld=false;
    // C key: toggle cockpit view (only when flying as pilot)
    if(keys[XK_c]){ if(!cockpitKeyHeld && inPlane && !isPassenger){ cockpitView=!cockpitView; cockpitKeyHeld=true; } }
    else cockpitKeyHeld=false;
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

    // Space key: fire missile (fighter only, airborne only)
    if(keys[XK_space]){
        if(!missileKeyHeld && !onGround && player.type == PLANE_FIGHTER){
            fireMissile();
            missileKeyHeld = true;
        }
    } else missileKeyHeld = false;

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

        float sy = sinf(player.yaw), cy2 = cosf(player.yaw);
        player.vx = sy * airspeed;
        player.vy = 0.0f;
        player.vz = cy2 * airspeed;

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
        // ---- In flight — arcade physics ----
        // Control inputs
        float pitchRate = 1.0f, rollRate = 0.9f;
        if(keys[XK_w]) player.pitch += pitchRate*dt;
        if(keys[XK_s]) player.pitch -= pitchRate*dt;
        if(keys[XK_a]) player.roll  -= rollRate*dt;
        if(keys[XK_d]) player.roll  += rollRate*dt;
        player.pitch = clampf(player.pitch, -1.3f, 1.3f);
        player.roll  = clampf(player.roll,  -1.5f, 1.5f);

        // Coordinated turn: bank drives yaw
        player.yaw -= sinf(player.roll) * dt;

        // Turbulence
        turbulenceTimer -= dt;
        if(turbulenceTimer <= 0.0f){
            turbulenceTarget = ((float)rand()/RAND_MAX)*((float)rand()/RAND_MAX);
            turbulenceTimer  = 4.0f + ((float)rand()/RAND_MAX)*8.0f;
        }
        turbulenceLevel += (turbulenceTarget - turbulenceLevel)*dt*0.4f;
        float turb = turbulenceLevel * TURBULENCE_MAX;
        player.pitch += ((float)rand()/RAND_MAX-0.5f)*turb;
        player.roll  += ((float)rand()/RAND_MAX-0.5f)*turb;
        player.yaw   += ((float)rand()/RAND_MAX-0.5f)*turb*0.5f;

        // Throttle drives airspeed convergence
        float thrustSpeed = player.throttle * MAX_SPEED;
        float accel = (thrustSpeed > airspeed) ? PLANE_DEFS[player.type].accel : -2.0f;
        airspeed += accel * dt;
        if(airspeed < 0.0f) airspeed = 0.0f;
        // No upper clamp — dive speed is unlimited

        // Stall sink
        float stallSink = 0.0f;
        if(airspeed < STALL_SPEED)
            stallSink = (STALL_SPEED - airspeed) * 2.0f;

        float syaw  = sinf(player.yaw),  cyaw  = cosf(player.yaw);
        float spitch = sinf(player.pitch), cpitch = cosf(player.pitch);

        player.vx = syaw  * cpitch * airspeed;
        player.vy = spitch * airspeed - GRAVITY*dt - stallSink;
        player.vz = cyaw  * cpitch * airspeed;

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
                menuState = MENU_DEAD;
            } else if(gearExtension > 0.9f){
                // Normal landing
                onGround  = true;
                airspeed  = sqrtf(player.vx*player.vx + player.vz*player.vz);
                player.pitch = 0.0f;
                player.roll  = 0.0f;
                sndPlay(SND_LAND);
            } else {
                // Gear-up, slow speed (e.g. stall onto runway) — no explosion, just stop
                airspeed  = 0.0f;
                player.vx = player.vy = player.vz = 0.0f;
                onGround  = true;
                gearDeployed = true;
            }
        }
    }

    // Consumable effect decay (runs regardless of in-plane/on-foot/in-store)
    if(effectHealTimer > 0){
        effectHealTimer -= dt;
        playerHealth = clampf(playerHealth + effectHeal * dt, 0.0f, 1.0f);
        if(effectHealTimer <= 0){ effectHeal = 0; effectHealTimer = 0; }
    }
    if(effectSpeedTimer > 0){
        effectSpeedTimer -= dt;
        if(effectSpeedTimer <= 0){ effectSpeed = 0; effectSpeedTimer = 0; }
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
        float cy = resHeight/2.0f + 80;

        glColor3f(0.6f,0.85f,1.0f);
        int htw=0; const char *ht="Join Server";
        for(const char *c=ht;*c;c++) htw+=glutBitmapWidth(GLUT_BITMAP_TIMES_ROMAN_24,*c);
        menuText((resWidth-htw)/2.0f, cy+80, ht, GLUT_BITMAP_TIMES_ROMAN_24);
        menuDivider(cy+66);

        // IP label + input box (wider for internet IPs)
        glColor3f(0.85f,0.85f,0.85f);
        menuText(LBL_X, cy+26, "Server IP:", GLUT_BITMAP_HELVETICA_18);

        float ibx=CTL_X, iby=cy+12;
        float ibw=220, ibh=28;
        menuRect(ibx,iby,ibw,ibh,
            joinIPFocus?0.12f:0.08f, joinIPFocus?0.22f:0.12f, joinIPFocus?0.38f:0.22f, 0.96f);
        menuRectOutline(ibx,iby,ibw,ibh,
            joinIPFocus?0.5f:0.32f, joinIPFocus?0.78f:0.55f, joinIPFocus?1.0f:0.75f);
        glColor3f(1,1,1);
        menuText(ibx+6, iby+7, joinIP, GLUT_BITMAP_HELVETICA_18);
        if(joinIPFocus){
            int cw=0; for(const char *c=joinIP;*c;c++) cw+=glutBitmapWidth(GLUT_BITMAP_HELVETICA_18,*c);
            glColor3f(0.7f,0.9f,1.0f);
            glBegin(GL_LINES);
            glVertex2f(ibx+6+cw, iby+4); glVertex2f(ibx+6+cw, iby+24);
            glEnd();
        }

        // Instructions
        glColor3f(0.55f,0.75f,1.0f);
        menuText(LBL_X, cy-14, "Ask the host for their IP address.", GLUT_BITMAP_HELVETICA_12);
        menuText(LBL_X, cy-30, "Works over internet and LAN.", GLUT_BITMAP_HELVETICA_12);
        menuText(LBL_X, cy-46, "Port 5000 must be open on the host.", GLUT_BITMAP_HELVETICA_12);

        // LAN servers (quick-fill convenience)
        pthread_mutex_lock(&lanMutex);
        if(numLanServers > 0){
            glColor3f(0.55f,0.78f,1.0f);
            menuText(LBL_X, cy-66, "LAN servers found:", GLUT_BITMAP_HELVETICA_12);
            for(int li = 0; li < numLanServers; li++){
                float rowY = cy - 86 - li*22;
                bool sel = (li == selectedServer);
                menuRect(CTL_X-2, rowY-2, 220, 20,
                    sel?0.15f:0.08f, sel?0.30f:0.12f, sel?0.55f:0.18f, 0.90f);
                if(sel) menuRectOutline(CTL_X-2, rowY-2, 220, 20, 0.4f,0.7f,1.0f);
                glColor3f(sel?1.0f:0.85f, sel?1.0f:0.85f, 1.0f);
                menuText(CTL_X+4, rowY+3, lanServers[li].ip, GLUT_BITMAP_HELVETICA_12);
            }
        }
        pthread_mutex_unlock(&lanMutex);

        if(joinFailed){
            glColor3f(1.0f,0.25f,0.25f);
            menuText(LBL_X, cy-130, "Could not connect. Check IP and firewall.", GLUT_BITMAP_HELVETICA_12);
        }

        menuButton(BTN_X, cy-160, "Connect", mx,my);
        menuButton(BTN_X, cy-210, "Back",    mx,my);

    } else if(menuState==MENU_DEAD){
        // Semi-transparent dark red overlay
        menuRect(0,0,(float)resWidth,(float)resHeight, 0.18f,0.02f,0.02f,0.72f);

        // "YOU CRASHED" in big red text
        const char *msg = "YOU CRASHED";
        int mw=0; for(const char *c=msg;*c;c++) mw+=glutBitmapWidth(GLUT_BITMAP_TIMES_ROMAN_24,*c);
        glColor3f(1.0f,0.15f,0.10f);
        menuText((resWidth-mw)/2.0f, resHeight/2.0f+40, msg, GLUT_BITMAP_TIMES_ROMAN_24);

        menuButton(BTN_X, resHeight/2.0f-20, "Return to Menu", mx,my);
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
            glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluPerspective(70,(double)resWidth/resHeight,0.5,6000.0);
            glMatrixMode(GL_MODELVIEW);
        }
        if(inRect(mx,my, arrowRightX,row, SML_W,SML_H)){
            if(resIndex<NUM_RES_OPTIONS-1) resIndex++;
            resWidth=resOptions[resIndex][0]; resHeight=resOptions[resIndex][1];
            XResizeWindow(dpy,win,resWidth,resHeight);
            glViewport(0,0,resWidth,resHeight);
            glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluPerspective(70,(double)resWidth/resHeight,0.5,6000.0);
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
        float cy  = resHeight/2.0f + 80;
        float ibx = CTL_X, iby = cy+12;
        joinIPFocus = inRect(mx,my, ibx,iby, 220,28);

        // Click on a LAN server row — fill IP box and select
        pthread_mutex_lock(&lanMutex);
        for(int li = 0; li < numLanServers; li++){
            float rowY = cy - 86 - li*22;
            if(inRect(mx,my, (int)(CTL_X-2),(int)(rowY-2), 220,20)){
                selectedServer = li;
                strncpy(joinIP, lanServers[li].ip, sizeof(joinIP)-1);
                joinIPLen = (int)strlen(joinIP);
                joinIPFocus = false;
            }
        }
        pthread_mutex_unlock(&lanMutex);

        if(inRect(mx,my, BTN_X,(int)(cy-160), BTN_W,BTN_H)){
            isServer=false; joinFailed=false; joinTimer=0.0f;
            menuState=MENU_PLAYING;
        }
        if(inRect(mx,my, BTN_X,(int)(cy-210), BTN_W,BTN_H)) menuState=MENU_MAIN;

    } else if(menuState==MENU_DEAD){
        if(inRect(mx,my, BTN_X,(int)(resHeight/2.0f-20), BTN_W,BTN_H)){
            menuState   = MENU_MAIN;
            gameStarted = false;
        }
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
            gluPerspective(70,(double)resWidth/resHeight,0.5,6000.0);
            glMatrixMode(GL_MODELVIEW);
        }
    }
    // Mouse motion
    if(e->type==MotionNotify){
        int mx = e->xmotion.x, my = e->xmotion.y;
        if(mouseCaptured && menuState==MENU_PLAYING){
            int dx = mx - mouseWarpX;
            int dy = my - mouseWarpY;
            if(dx != 0 || dy != 0){
                if(!inPlane){
                    // Walker / store: yaw left-right, pitch not used
                    if(inStoreIdx >= 0)
                        storeWalkerYaw -= dx * mouseSensX;
                    else
                        walkerYaw      -= dx * mouseSensX;
                }
                XWarpPointer(dpy, None, win, 0,0,0,0, mouseWarpX, mouseWarpY);
            }
        } else {
            menuMouseX = mx;
            menuMouseY = resHeight - my;
        }
    }
    if(e->type==ButtonPress && e->xbutton.button==Button1){
        if(menuState!=MENU_PLAYING){
            handleMenuClick(e->xbutton.x, e->xbutton.y);
        }
    }
    if(e->type==KeyPress){
        KeySym k=XLookupKeysym(&e->xkey,0);

        // Chat input intercepts all keys when open (in MENU_PLAYING)
        if(chatOpen && menuState==MENU_PLAYING){
            if(k==XK_Escape || k==XK_t || k==XK_T){
                chatOpen = false;
                chatInput[0]='\0'; chatInputLen=0;
            } else if(k==XK_Return){
                if(chatInputLen>0){ chatSend(chatInput); }
                chatInput[0]='\0'; chatInputLen=0;
                chatOpen = false;
            } else if(k==XK_BackSpace){
                if(chatInputLen>0){ chatInputLen--; chatInput[chatInputLen]='\0'; }
            } else {
                char buf[8]; int n=XLookupString(&e->xkey,buf,sizeof(buf),NULL,NULL);
                if(n==1 && chatInputLen<CHAT_MSG_LEN-2){
                    chatInput[chatInputLen++]=buf[0];
                    chatInput[chatInputLen]='\0';
                }
            }
            return;
        }

        // Store ordering/consuming: digit 1-5 while inside a store
        if(inStoreIdx >= 0 && inStoreIdx < numStores && menuState==MENU_PLAYING){
            int orderIdx = -1;
            if(k==XK_1) orderIdx=0; else if(k==XK_2) orderIdx=1;
            else if(k==XK_3) orderIdx=2; else if(k==XK_4) orderIdx=3;
            else if(k==XK_5) orderIdx=4;
            if(orderIdx >= 0){
                StoreType st2 = stores[inStoreIdx].type;
                const MenuItem *menu2 = (st2==STORE_ALBERTSONS) ? menuAlbertsons
                                       :(st2==STORE_ARBYS)       ? menuArbys
                                                                  : menuLJS;
                const MenuItem *item = &menu2[orderIdx];
                if(inventory[orderIdx] > 0){
                    // Consume item from inventory
                    inventory[orderIdx]--;
                    chatAddMsg(item->useMsg);
                    // Apply effect
                    if(item->effect == EFF_HEAL || item->effect == EFF_HEALSPD){
                        effectHeal      = item->effectMag;
                        effectHealTimer = item->effectDur;
                    }
                    if(item->effect == EFF_SPEED || item->effect == EFF_HEALSPD){
                        effectSpeed      = item->effectMag;
                        effectSpeedTimer = item->effectDur;
                    }
                } else {
                    // Buy item — add to inventory
                    inventory[orderIdx]++;
                    char msg[128];
                    snprintf(msg, sizeof(msg), "[Cashier] %s  (press %d again to eat/drink)",
                             item->buyMsg, orderIdx+1);
                    chatAddMsg(msg);
                }
            }
            return;
        }

        // Consume inventory items outside stores too (press 1-5)
        if(menuState==MENU_PLAYING && !chatOpen && inStoreIdx<0){
            int orderIdx = -1;
            if(k==XK_1) orderIdx=0; else if(k==XK_2) orderIdx=1;
            else if(k==XK_3) orderIdx=2; else if(k==XK_4) orderIdx=3;
            else if(k==XK_5) orderIdx=4;
            if(orderIdx >= 0 && inventory[orderIdx] > 0){
                const MenuItem *menu2 = (lastStoreType==STORE_ALBERTSONS) ? menuAlbertsons
                                        :(lastStoreType==STORE_ARBYS)      ? menuArbys
                                                                            : menuLJS;
                const MenuItem *item = &menu2[orderIdx];
                inventory[orderIdx]--;
                chatAddMsg(item->useMsg);
                if(item->effect == EFF_HEAL || item->effect == EFF_HEALSPD){
                    effectHeal      = item->effectMag;
                    effectHealTimer = item->effectDur;
                }
                if(item->effect == EFF_SPEED || item->effect == EFF_HEALSPD){
                    effectSpeed      = item->effectMag;
                    effectSpeedTimer = item->effectDur;
                }
            }
        }

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
            if(menuState==MENU_PLAYING){
                menuState=MENU_MAIN;
                if(mouseCaptured){ XUngrabPointer(dpy, CurrentTime); mouseCaptured=false; }
            } else if(menuState==MENU_DEAD){
                menuState=MENU_MAIN; gameStarted=false;
                if(mouseCaptured){ XUngrabPointer(dpy, CurrentTime); mouseCaptured=false; }
            } else running=false;
        }
        if(k==XK_t || k==XK_T){
            if(menuState==MENU_PLAYING && !chatOpen){
                chatOpen=true; chatInput[0]='\0'; chatInputLen=0;
            }
        }
        if(k==XK_m || k==XK_M){ if(!mapKeyHeld){ mapOpen=!mapOpen; mapKeyHeld=true; } }
        if(k==XK_v || k==XK_V) voiceTx = true;
    }
    if(e->type==KeyRelease){
        KeySym k=XLookupKeysym(&e->xkey,0);
        if(k<65536) keys[k]=false;
        if(k==XK_m || k==XK_M) mapKeyHeld=false;
        if(k==XK_v || k==XK_V) voiceTx = false;
    }
}

// forward decl
void startGame();

// -------- Game Loop --------
void gameLoop(){
    XEvent e;
    while(running){
        while(XPending(dpy)){ XNextEvent(dpy,&e); handleInput(&e); }

        // Always tick LAN discovery + host broadcasts
        updateLanServers(DT);
        // Flush pending chat send (deferred because chatSend is defined before network globals)
        if(chatHasPending){
            chatHasPending = false;
            int pkt[1 + CHAT_MSG_LEN/4 + 1];
            pkt[0] = MSG_CHAT;
            memset(pkt+1, 0, CHAT_MSG_LEN);
            strncpy((char*)(pkt+1), chatPendingMsg, CHAT_MSG_LEN-1);
            if((isServer&&clientConnected)||(!isServer&&seedReceived))
                sendto(sockfd,(char*)pkt,sizeof(int)+CHAT_MSG_LEN,0,
                       (struct sockaddr*)&otherAddr,sizeof(otherAddr));
        }
        if(gameStarted && isServer){
            lanAnnounceTimer -= DT;
            if(lanAnnounceTimer <= 0.0f){
                lanBroadcastPresence();
                lanAnnounceTimer = LAN_ANNOUNCE_INTERVAL;
            }
        }

        if(menuState != MENU_PLAYING){
            if(gameStarted){
                // Dead state: keep animating explosions/tornados while frozen
                if(menuState == MENU_DEAD){
                    updateExplosions(DT);
                    updateTornados(DT);
                }
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
            // Capture mouse and hide cursor
            mouseWarpX = resWidth  / 2;
            mouseWarpY = resHeight / 2;
            XWarpPointer(dpy, None, win, 0,0,0,0, mouseWarpX, mouseWarpY);
            // Create invisible cursor
            static char cdata[1] = {0};
            Pixmap cpix = XCreateBitmapFromData(dpy, win, cdata, 1, 1);
            XColor ccol; ccol.pixel=0; ccol.red=0; ccol.green=0; ccol.blue=0; ccol.flags=0;
            Cursor invisCursor = XCreatePixmapCursor(dpy, cpix, cpix, &ccol, &ccol, 0, 0);
            XFreePixmap(dpy, cpix);
            XGrabPointer(dpy, win, True,
                         PointerMotionMask|ButtonPressMask|ButtonReleaseMask,
                         GrabModeAsync, GrabModeAsync, win, invisCursor, CurrentTime);
            mouseCaptured = true;
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

        gameTime += gameTimeSpeed * DT;
        if(gameTime >= 1.0f) gameTime -= 1.0f;
        updateWeather(DT);
        updateTornados(DT);
        updateExplosions(DT);
        updateMissiles(DT);
        updateNpcs(DT);
        updatePhysics(DT);
        // Engine sound: on when in-plane and not dead
        sndUpdateEngine(player.throttle, inPlane && menuState==MENU_PLAYING);
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

    // Sun — position/colour updated dynamically each frame by updateSunLight()
    // Set initial values so first frame isn't black
    GLfloat lightPos[]  = { -0.3f, 0.8f, 0.5f, 0.0f };
    GLfloat ambient[]   = { 0.15f, 0.18f, 0.22f, 1.0f };
    GLfloat diffuse[]   = { 0.90f, 0.85f, 0.70f, 1.0f };
    GLfloat specular[]  = { 0.70f, 0.65f, 0.55f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);

    // Sky fill light (GL_LIGHT1): dim, from below — softens shadow side
    glEnable(GL_LIGHT1);
    GLfloat skyPos[]  = { 0.0f, -1.0f, 0.0f, 0.0f }; // from below (sky bounce)
    GLfloat skyAmb[]  = { 0.0f,  0.0f, 0.0f, 1.0f };
    GLfloat skyDiff[] = { 0.08f, 0.12f, 0.18f, 1.0f }; // cool blue fill
    GLfloat skySpec[] = { 0.0f,  0.0f,  0.0f, 1.0f };
    glLightfv(GL_LIGHT1, GL_POSITION, skyPos);
    glLightfv(GL_LIGHT1, GL_AMBIENT,  skyAmb);
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  skyDiff);
    glLightfv(GL_LIGHT1, GL_SPECULAR, skySpec);

    // Material specular response — shinier metal
    GLfloat matSpec[] = { 0.7f, 0.7f, 0.7f, 1.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  matSpec);
    glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 72.0f);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    glClearColor(0.5f,0.8f,1.0f,1.0f);
    generateCheckerboardTexture();
    generateBiomeTextures();
}

// -------- Network Init --------
// serverPort: port the server listens on
// clientPort: port the client binds to locally (must differ from serverPort on same machine)
// -------- LAN Server Discovery --------
// Host: broadcast "FLIGHTSIM_HOST" every 2s on LAN_DISCOVERY_PORT
// Clients: listen on LAN_DISCOVERY_PORT, record source IPs, display in Join UI

static void lanAddServer(const char *ip){
    pthread_mutex_lock(&lanMutex);
    // Update existing entry if already known
    for(int i=0;i<numLanServers;i++){
        if(strcmp(lanServers[i].ip, ip)==0){
            lanServers[i].seenTimer = LAN_SERVER_TIMEOUT;
            pthread_mutex_unlock(&lanMutex);
            return;
        }
    }
    if(numLanServers < MAX_LAN_SERVERS){
        strncpy(lanServers[numLanServers].ip, ip, 63);
        lanServers[numLanServers].ip[63] = '\0';
        lanServers[numLanServers].seenTimer = LAN_SERVER_TIMEOUT;
        numLanServers++;
    }
    pthread_mutex_unlock(&lanMutex);
}

static void* lanListenThreadFunc(void *arg){
    (void)arg;
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if(s < 0) return NULL;
    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(LAN_DISCOVERY_PORT);
    if(bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0){ close(s); return NULL; }
    fcntl(s, F_SETFL, O_NONBLOCK);
    lanSock = s;
    char buf[64];
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);
    while(lanThreadRunning){
        int n = recvfrom(s, buf, sizeof(buf)-1, 0, (struct sockaddr*)&from, &fromlen);
        if(n > 0){
            buf[n] = '\0';
            if(strncmp(buf, "FLIGHTSIM_HOST", 14)==0){
                char fromIP[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &from.sin_addr, fromIP, sizeof(fromIP));
                lanAddServer(fromIP);
            }
        }
        usleep(50000); // 50ms poll
    }
    close(s);
    lanSock = -1;
    return NULL;
}

// Start the LAN listener (call once from main after GL context)
static void startLanListener(){
    lanThreadRunning = true;
    pthread_create(&lanThread, NULL, lanListenThreadFunc, NULL);
}

// Called from game loop to broadcast presence when hosting
static void lanBroadcastPresence(){
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if(s < 0) return;
    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
    struct sockaddr_in dest = {0};
    dest.sin_family      = AF_INET;
    dest.sin_addr.s_addr = INADDR_BROADCAST;
    dest.sin_port        = htons(LAN_DISCOVERY_PORT);
    const char *msg = "FLIGHTSIM_HOST";
    sendto(s, msg, strlen(msg), 0, (struct sockaddr*)&dest, sizeof(dest));
    close(s);
}

// Tick LAN timers — call each frame (DT seconds)
static void updateLanServers(float dt){
    pthread_mutex_lock(&lanMutex);
    for(int i=0;i<numLanServers;){
        lanServers[i].seenTimer -= dt;
        if(lanServers[i].seenTimer <= 0.0f){
            // Remove by swapping with last
            lanServers[i] = lanServers[--numLanServers];
            if(selectedServer >= numLanServers) selectedServer = numLanServers-1;
        } else {
            i++;
        }
    }
    pthread_mutex_unlock(&lanMutex);
}

void initNetwork(const char* serverIp, int serverPort){
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd<0){ perror("socket"); exit(1); }

    // Both server and client bind the same port so NAT traversal works:
    // client sends first (opens NAT hole), server replies to source addr it sees.
    // SO_REUSEADDR lets restarts reuse the port immediately.
    int reuse = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bindAddr = {0};
    bindAddr.sin_family      = AF_INET;
    bindAddr.sin_addr.s_addr = INADDR_ANY;
    bindAddr.sin_port        = htons(serverPort);
    if(bind(sockfd,(struct sockaddr*)&bindAddr,sizeof(bindAddr))<0){ perror("bind"); exit(1); }
    printf("[NET] %s bound to port %d | target server: %s:%d\n",
        isServer ? "Server" : "Client", serverPort, serverIp, serverPort);

    // Client pre-fills server address; server waits to learn client address on first MSG_CONNECT.
    if(!isServer){
        otherAddr.sin_family = AF_INET;
        otherAddr.sin_port   = htons(serverPort);
        inet_pton(AF_INET, serverIp, &otherAddr.sin_addr);
    }

    fcntl(sockfd, F_SETFL, O_NONBLOCK);

    // Detect local IP via UDP connect trick (no packet sent)
    {
        int tmp = socket(AF_INET, SOCK_DGRAM, 0);
        if(tmp >= 0){
            struct sockaddr_in probe = {0};
            probe.sin_family = AF_INET;
            probe.sin_port   = htons(80);
            inet_pton(AF_INET, "8.8.8.8", &probe.sin_addr);
            if(connect(tmp,(struct sockaddr*)&probe,sizeof(probe))==0){
                struct sockaddr_in me = {0}; socklen_t ml=sizeof(me);
                if(getsockname(tmp,(struct sockaddr*)&me,&ml)==0)
                    inet_ntop(AF_INET,&me.sin_addr,localIP,sizeof(localIP));
            }
            close(tmp);
        }
        printf("[NET] Local IP: %s\n", localIP);
    }
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

    // Spawn at runway threshold, facing down the runway
    {
        Airport *a0 = &airports[0];
        // apLocalToWorld: world_x = wx + cos(h)*fwd, world_z = wz - sin(h)*fwd
        // Threshold at local fwd = -runwayLen
        float wx, wz;
        apLocalToWorld(a0, -a0->runwayLen, 0.0f, &wx, &wz);
        player.x   = wx;
        player.z   = wz;
        player.y   = a0->groundY + 1.0f;
        // World velocity direction of local +fwd: vx=cos(h), vz=-sin(h)
        // Yaw convention: vx=sin(yaw), vz=cos(yaw)
        // => sin(yaw)=cos(h), cos(yaw)=-sin(h) => yaw = pi/2 - h
        player.yaw = (float)M_PI*0.5f - a0->heading;
    }
    player.pitch=0; player.roll=0; player.throttle=0.0f; player.type=PLANE_FIGHTER;
    onGround=true; airspeed=0.0f;
    sndInit();
    initWeather();
    initClouds();
    initTrees();
    initStores();
    initSuburbs();
    voiceInit();
    voiceStart();
    initNetwork(joinIP, 5000);

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
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluPerspective(70,(double)resWidth/resHeight,0.5,6000.0);
    glMatrixMode(GL_MODELVIEW);

    startLanListener(); // background thread listens for LAN server announcements

    gameLoop();

    running=false;
    pthread_join(netThread,NULL);
    lanThreadRunning=false;
    pthread_join(lanThread,NULL);

    glXMakeCurrent(dpy,0,0); glXDestroyContext(dpy,glc); XDestroyWindow(dpy,win); XCloseDisplay(dpy);
    return 0;
}