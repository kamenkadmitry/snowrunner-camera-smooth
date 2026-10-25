// CameraSmooth.asi — SnowRunner free-camera rotation smoother.
//
// Loaded by the game's ASI loader (dinput8.dll). Hooks the per-frame camera
// update function (located by signature, not a hardcoded offset) so smoothing
// runs on the game thread right before the function copies the camera block
// cam -> shadow. Our write is therefore the last one before the render-side
// copy: deterministic, no racing, no flicker.
//
// Camera struct layout (resolved at runtime):
//   cam+0x00  view matrix   (row-major, translation in row 4)  <- we write
//   cam+0x40  projection    (per-frame jitter, never touched)
//   cam+0x80  world matrix  (camera basis + eye in row 4)      <- we write
//   cam+0xC0  jittered VP   (recomputed by the game each frame)

#include <windows.h>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <cstdio>

namespace {

// call <accessor>; mov r15,rax; mov rcx,[rax]; mov rdx,[rcx+8]; mov r8,[rcx+10h]
const uint8_t kPattern[] = {
    0xE8, 0x00, 0x00, 0x00, 0x00,
    0x4C, 0x8B, 0xF8,
    0x48, 0x8B, 0x08,
    0x48, 0x8B, 0x51, 0x08,
    0x4C, 0x8B, 0x41, 0x10,
};
const bool kPatternMask[] = {
    true, false, false, false, false,
    true, true, true,
    true, true, true,
    true, true, true, true,
    true, true, true, true,
};

// epilogue of the per-frame camera writer function (0x884a2f):
//   mov rbx,[rsp+0x200]; mov r14,[rsp+0x210]; mov rdi,[rsp+0x218];
//   add rsp,0x1f0; pop rbp; ret
const uint8_t kEpilogueSig[] = {
    0x48, 0x8B, 0x9C, 0x24, 0x00, 0x02, 0x00, 0x00,
    0x4C, 0x8B, 0xB4, 0x24, 0x10, 0x02, 0x00, 0x00,
    0x48, 0x8B, 0xBC, 0x24, 0x18, 0x02, 0x00, 0x00,
    0x48, 0x81, 0xC4, 0xF0, 0x01, 0x00, 0x00,
    0x5D, 0xC3,
};
const size_t kHookLen = 16;               // patch region: first two movs
const uintptr_t kFlyFlagRva = 0x2A5834C;  // freecam flag (.data)

// freecam enable: same two dev-check patches SnowFlyer2 applies —
//   cmp byte [rax+0xC8], imm   (imm 0 -> 1, two sites with different suffixes)
const uint8_t kDevCheckSig[] = {0x80, 0xB8, 0xC8, 0x00, 0x00, 0x00};

const float kTau = 0.12f;                 // smoothing time constant, seconds
const float kNewInputThreshold = 0.002f;  // rad; bigger delta => new game target
const float kSnapAngle = 1.2f;            // rad; snap instead of smoothing huge jumps

struct Quat { float x, y, z, w; };
struct Mat { float m[4][4]; };

uintptr_t g_base = 0;
uintptr_t g_slot = 0;      // -> owner P; cam = [P+8]
uintptr_t g_flagAddr = 0;

// step logger — last line before a crash tells us the culprit
void Trace(const char* msg) {
    static HANDLE h = nullptr;
    if (!h) {
        wchar_t p[MAX_PATH];
        GetModuleFileNameW(nullptr, p, MAX_PATH);
        *wcsrchr(p, L'\\') = 0;
        wcscat_s(p, MAX_PATH, L"\\CameraSmooth-trace.log");
        h = CreateFileW(p, GENERIC_WRITE, FILE_SHARE_READ, nullptr,
                        OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (h == INVALID_HANDLE_VALUE) { h = nullptr; return; }
        SetFilePointer(h, 0, nullptr, FILE_END);
    }
    DWORD w;
    WriteFile(h, msg, static_cast<DWORD>(strlen(msg)), &w, nullptr);
    WriteFile(h, "\r\n", 2, &w, nullptr);
    FlushFileBuffers(h);
}
void TraceU(const char* msg, uintptr_t v) {
    char buf[96];
    snprintf(buf, sizeof(buf), "%s%llx", msg, (unsigned long long)v);
    Trace(buf);
}

Quat g_smoothed{0.f, 0.f, 0.f, 1.f};
Quat g_target{0.f, 0.f, 0.f, 1.f};
bool g_hasState = false;
bool g_enabled = true;
bool g_f8down = false;
bool g_f9down = false;
LARGE_INTEGER g_prev{};
LARGE_INTEGER g_freq{};

Mat MatMul(const Mat& a, const Mat& b) {
    Mat r = {};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}

uintptr_t FindPattern(uintptr_t begin, size_t size) {
    const size_t plen = sizeof(kPattern);
    for (size_t i = 0; i + plen <= size; ++i) {
        bool ok = true;
        for (size_t j = 0; j < plen; ++j) {
            if (kPatternMask[j] && reinterpret_cast<uint8_t*>(begin)[i + j] != kPattern[j]) {
                ok = false;
                break;
            }
        }
        if (ok) return begin + i;
    }
    return 0;
}

Quat QuatFromMat(const Mat& m) {
    float t = m.m[0][0] + m.m[1][1] + m.m[2][2];
    Quat q;
    if (t > 0.f) {
        float s = sqrtf(t + 1.f) * 2.f;
        q.w = 0.25f * s;
        q.x = (m.m[2][1] - m.m[1][2]) / s;
        q.y = (m.m[0][2] - m.m[2][0]) / s;
        q.z = (m.m[1][0] - m.m[0][1]) / s;
    } else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
        float s = sqrtf(1.f + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.f;
        q.w = (m.m[2][1] - m.m[1][2]) / s;
        q.x = 0.25f * s;
        q.y = (m.m[0][1] + m.m[1][0]) / s;
        q.z = (m.m[0][2] + m.m[2][0]) / s;
    } else if (m.m[1][1] > m.m[2][2]) {
        float s = sqrtf(1.f + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.f;
        q.w = (m.m[0][2] - m.m[2][0]) / s;
        q.x = (m.m[0][1] + m.m[1][0]) / s;
        q.y = 0.25f * s;
        q.z = (m.m[1][2] + m.m[2][1]) / s;
    } else {
        float s = sqrtf(1.f + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.f;
        q.w = (m.m[1][0] - m.m[0][1]) / s;
        q.x = (m.m[0][2] + m.m[2][0]) / s;
        q.y = (m.m[1][2] + m.m[2][1]) / s;
        q.z = 0.25f * s;
    }
    return q;
}

Quat QuatNormalize(Quat q) {
    float l = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (l < 1e-8f) return Quat{0.f, 0.f, 0.f, 1.f};
    q.x /= l; q.y /= l; q.z /= l; q.w /= l;
    return q;
}

Quat QuatSlerp(Quat a, Quat b, float t) {
    float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0.f) { b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w; d = -d; }
    if (d > 0.9995f) {
        return QuatNormalize(Quat{
            a.x + t * (b.x - a.x),
            a.y + t * (b.y - a.y),
            a.z + t * (b.z - a.z),
            a.w + t * (b.w - a.w)});
    }
    float th = acosf(d);
    float s = sinf(th);
    float wa = sinf((1.f - t) * th) / s;
    float wb = sinf(t * th) / s;
    return Quat{
        a.x * wa + b.x * wb,
        a.y * wa + b.y * wb,
        a.z * wa + b.z * wb,
        a.w * wa + b.w * wb};
}

float QuatAngle(Quat a, Quat b) {
    float d = fabsf(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    if (d > 1.f) d = 1.f;
    return 2.f * acosf(d);
}

Mat MatFromQuat(Quat q, float tx, float ty, float tz) {
    float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    Mat m = {};
    m.m[0][0] = 1.f - 2.f * (yy + zz);
    m.m[0][1] = 2.f * (xy - wz);
    m.m[0][2] = 2.f * (xz + wy);
    m.m[1][0] = 2.f * (xy + wz);
    m.m[1][1] = 1.f - 2.f * (xx + zz);
    m.m[1][2] = 2.f * (yz - wx);
    m.m[2][0] = 2.f * (xz - wy);
    m.m[2][1] = 2.f * (yz + wx);
    m.m[2][2] = 1.f - 2.f * (xx + yy);
    m.m[3][0] = tx;
    m.m[3][1] = ty;
    m.m[3][2] = tz;
    m.m[3][3] = 1.f;
    return m;
}

// view = inverse(world); basis is orthonormal so inverse = transpose + negated dot
Mat MatInverseRigid(const Mat& w) {
    Mat v = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            v.m[i][j] = w.m[j][i];
    v.m[3][0] = -(w.m[0][0] * w.m[3][0] + w.m[1][0] * w.m[3][1] + w.m[2][0] * w.m[3][2]);
    v.m[3][1] = -(w.m[0][1] * w.m[3][0] + w.m[1][1] * w.m[3][1] + w.m[2][1] * w.m[3][2]);
    v.m[3][2] = -(w.m[0][2] * w.m[3][0] + w.m[1][2] * w.m[3][1] + w.m[2][2] * w.m[3][2]);
    v.m[3][3] = 1.f;
    return v;
}

bool WorldLooksValid(const Mat& w) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (!std::isfinite(w.m[i][j])) return false;
    for (int i = 0; i < 3; ++i) {
        float l = sqrtf(w.m[i][0] * w.m[i][0] + w.m[i][1] * w.m[i][1] + w.m[i][2] * w.m[i][2]);
        if (fabsf(l - 1.f) > 0.005f) return false;
    }
    return w.m[0][3] == 0.f && w.m[1][3] == 0.f && w.m[2][3] == 0.f && w.m[3][3] == 1.f;
}

uintptr_t TextSection(uintptr_t moduleBase, size_t& size); // fwd

// Guard every pointer before dereference — during early frames the camera
// objects exist but can point at half-constructed or uncommitted memory,
// and an access violation inside a detour kills the process silently.
bool MemoryOk(const void* p, size_t len, bool needWrite = false) {
    if (!p) return false;
    const uintptr_t end = reinterpret_cast<uintptr_t>(p) + len - 1;
    for (uintptr_t a = reinterpret_cast<uintptr_t>(p); a <= end; a = (a & ~0xFFFull) + 0x1000) {
        MEMORY_BASIC_INFORMATION mbi;
        if (!VirtualQuery(reinterpret_cast<void*>(a), &mbi, sizeof(mbi))) return false;
        if (mbi.State != MEM_COMMIT) return false;
        const DWORD r = PAGE_READONLY | PAGE_READWRITE | PAGE_WRITECOPY |
                        PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE;
        const DWORD w = PAGE_READWRITE | PAGE_WRITECOPY | PAGE_EXECUTE_READWRITE;
        if ((mbi.Protect & (needWrite ? w : r)) == 0) return false;
    }
    return true;
}

bool ToggleFreecam(uintptr_t moduleBase, bool enable) {
    size_t size = 0;
    uintptr_t text = TextSection(moduleBase, size);
    if (!text) return false;
    int patched = 0;
    for (size_t i = 0; i + 9 <= size; ++i) {
        uint8_t* p = reinterpret_cast<uint8_t*>(text + i);
        if (memcmp(p, kDevCheckSig, sizeof(kDevCheckSig)) != 0) continue;
        if (p[7] != 0x75 && p[7] != 0x0F) continue;
        uint8_t want = enable ? 0x01 : 0x00;
        if (p[6] == want) { ++patched; continue; }
        DWORD old;
        if (!VirtualProtect(p + 6, 1, PAGE_EXECUTE_READWRITE, &old)) continue;
        p[6] = want;
        VirtualProtect(p + 6, 1, old, &old);
        ++patched;
    }
    if (patched == 0) return false;
    if (g_flagAddr)
        *reinterpret_cast<int32_t*>(g_flagAddr) = enable ? 1 : 0;
    return true;
}

void SmoothCamObject(uintptr_t cam) {
    // write only view + world (0x00-0xBF) — exactly the writes proven stable
    // live; +0xC0 viewProj is left to the game's own dirty-flag rebuild and the
    // object's true extent isn't guaranteed.
    if (!MemoryOk(reinterpret_cast<void*>(cam), 0xC0, true)) return;
    Mat world;
    memcpy(&world, reinterpret_cast<const void*>(cam + 0x80), sizeof(Mat));
    if (!WorldLooksValid(world)) return;
    Mat w = MatFromQuat(g_smoothed, world.m[3][0], world.m[3][1], world.m[3][2]);
    Mat v = MatInverseRigid(w);
    memcpy(reinterpret_cast<void*>(cam), &v, sizeof(Mat));
    memcpy(reinterpret_cast<void*>(cam + 0x80), &w, sizeof(Mat));
}

bool AdvanceSmoothed(uintptr_t srcCam) {
    if (!MemoryOk(reinterpret_cast<void*>(srcCam + 0x80), sizeof(Mat))) return false;
    Mat world;
    memcpy(&world, reinterpret_cast<const void*>(srcCam + 0x80), sizeof(Mat));
    if (!WorldLooksValid(world)) return false;
    Quat read = QuatNormalize(QuatFromMat(world));

    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    float dt = static_cast<float>(now.QuadPart - g_prev.QuadPart) / g_freq.QuadPart;
    g_prev = now;
    if (dt <= 0.f || dt > 0.1f) dt = 1.f / 60.f;

    if (!g_hasState) {
        g_smoothed = g_target = read;
        g_hasState = true;
        return false;
    }
    if (QuatAngle(g_smoothed, read) > kNewInputThreshold) g_target = read;
    if (QuatAngle(g_smoothed, g_target) > kSnapAngle) g_smoothed = g_target;
    else {
        float alpha = 1.f - expf(-dt / kTau);
        g_smoothed = QuatNormalize(QuatSlerp(g_smoothed, g_target, alpha));
    }
    return true;
}

void PollHotkeys() {
    if (GetAsyncKeyState(VK_F8) & 0x8000) {
        if (!g_f8down) {
            g_enabled = !g_enabled;
            g_f8down = true;
            Trace(g_enabled ? "smooth ON" : "smooth OFF");
        }
    } else {
        g_f8down = false;
    }
    if (GetAsyncKeyState(VK_F9) & 0x8000) {
        if (!g_f9down) {
            g_f9down = true;
            bool on = g_flagAddr && *reinterpret_cast<int32_t*>(g_flagAddr) == 0;
            if (ToggleFreecam(g_base, on)) Trace(on ? "freecam ON" : "freecam OFF");
        }
    } else {
        g_f9down = false;
    }
}

void SmoothAll(const uintptr_t* objs, int n) {
    uintptr_t srcCam = 0;
    for (int o = 0; o < n; ++o) {
        if (!MemoryOk(reinterpret_cast<void*>(objs[o] + 0x80), sizeof(Mat))) continue;
        Mat w;
        memcpy(&w, reinterpret_cast<const void*>(objs[o] + 0x80), sizeof(Mat));
        if (WorldLooksValid(w)) { srcCam = objs[o]; break; }
    }
    if (!srcCam) { g_hasState = false; return; }
    if (!AdvanceSmoothed(srcCam)) return;
    for (int o = 0; o < n; ++o)
        if (objs[o]) SmoothCamObject(objs[o]);
}

// Hook 1: epilogue of the per-frame camera writer (0x884a2f). Runs on the game
// thread right after that function stored fresh matrices (flag=0 path).
extern "C" void CameraSmoothTick() {
    Trace("t1 enter");
    if (!g_enabled || !g_slot) { g_hasState = false; Trace("t1 off"); return; }
    uintptr_t owner = *reinterpret_cast<uintptr_t*>(g_slot);
    if (!MemoryOk(reinterpret_cast<void*>(owner), 0x18)) { g_hasState = false; return; }
    uintptr_t objs[1] = {
        *reinterpret_cast<uintptr_t*>(owner + 0x08),
    };
    TraceU("t1 objs ", objs[0]);
    SmoothAll(objs, 1);
    Trace("t1 done");
}

// Hook 2: merge point (0x856299) of the big frame function — runs every frame
// in BOTH camera paths. Hotkeys live here; r15 is the frame-context pointer and
// [r15+0x18]+8 is the dev-freecam camera struct (same 0x140 layout).
extern "C" void CameraSmoothTick2(uintptr_t r15) {
    TraceU("t2 enter r15=", r15);
    PollHotkeys();
    Trace("t2 hotkeys ok");
    if (!g_enabled || !g_slot) { g_hasState = false; Trace("t2 off"); return; }
    uintptr_t owner = *reinterpret_cast<uintptr_t*>(g_slot);
    if (!MemoryOk(reinterpret_cast<void*>(owner), 0x18)) { g_hasState = false; return; }
    uintptr_t objs[2] = {
        *reinterpret_cast<uintptr_t*>(owner + 0x08),
        0,
    };
    if (r15 && MemoryOk(reinterpret_cast<void*>(r15 + 0x18), 8)) {
        uintptr_t fc = *reinterpret_cast<uintptr_t*>(r15 + 0x18);
        if (fc && MemoryOk(reinterpret_cast<void*>(fc + 0x08), 8))
            objs[1] = *reinterpret_cast<uintptr_t*>(fc + 0x08);
    }
    TraceU("t2 objs ", objs[0]);
    TraceU("t2 obj1 ", objs[1]);
    SmoothAll(objs, 2);
    Trace("t2 done");
}

uintptr_t TextSection(uintptr_t moduleBase, size_t& size) {
    auto dos = reinterpret_cast<IMAGE_DOS_HEADER*>(moduleBase);
    auto nt = reinterpret_cast<IMAGE_NT_HEADERS64*>(moduleBase + dos->e_lfanew);
    auto sec = IMAGE_FIRST_SECTION(nt);
    for (int i = 0; i < nt->FileHeader.NumberOfSections; ++i) {
        if (memcmp(sec[i].Name, ".text", 5) == 0) {
            size = sec[i].Misc.VirtualSize;
            return moduleBase + sec[i].VirtualAddress;
        }
    }
    return 0;
}

uintptr_t FindCallSite(uintptr_t moduleBase) {
    size_t size = 0;
    uintptr_t text = TextSection(moduleBase, size);
    return text ? FindPattern(text, size) : 0;
}

uintptr_t ResolveSlotFromCallSite(uintptr_t callSite) {
    int32_t rel = *reinterpret_cast<int32_t*>(callSite + 1);
    uintptr_t accessor = callSite + 5 + rel;
    const uint8_t* a = reinterpret_cast<const uint8_t*>(accessor);
    if (!(a[0] == 0x48 && a[1] == 0x8D && a[2] == 0x05 && a[7] == 0xC3)) return 0;
    int32_t disp = *reinterpret_cast<int32_t*>(accessor + 3);
    return accessor + 7 + disp;
}

uintptr_t FindEpilogue(uintptr_t moduleBase) {
    size_t size = 0;
    uintptr_t text = TextSection(moduleBase, size);
    if (!text) return 0;
    const size_t siglen = sizeof(kEpilogueSig);
    for (size_t i = 0; i + siglen <= size; ++i) {
        if (memcmp(reinterpret_cast<void*>(text + i), kEpilogueSig, siglen) == 0)
            return text + i;
    }
    return 0;
}

bool InstallHook(uintptr_t hookAddr) {
    // expected bytes at hook site: mov rbx,[rsp+0x200]; mov r14,[rsp+0x210]
    const uint8_t expected[kHookLen] = {
        0x48, 0x8B, 0x9C, 0x24, 0x00, 0x02, 0x00, 0x00,
        0x4C, 0x8B, 0xB4, 0x24, 0x10, 0x02, 0x00, 0x00,
    };
    if (memcmp(reinterpret_cast<void*>(hookAddr), expected, sizeof(expected)) != 0)
        return false;

    uint8_t* tramp = reinterpret_cast<uint8_t*>(
        VirtualAlloc(nullptr, 0x200, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE));
    if (!tramp) return false;

    uintptr_t retAddr = hookAddr + kHookLen;
    uintptr_t tick = reinterpret_cast<uintptr_t>(&CameraSmoothTick);
    uintptr_t t = reinterpret_cast<uintptr_t>(tramp);
    uint8_t* p = tramp;
    auto emit = [&](std::initializer_list<uint8_t> b) {
        for (uint8_t x : b) *p++ = x;
    };
    auto emit64 = [&](uintptr_t v) { memcpy(p, &v, 8); p += 8; };

    // save volatile GPRs + xmm0-5; rsp is restored before original code so
    // rsp-relative frame reads in the copied instructions stay correct
    emit({0x50, 0x51, 0x52});                               // push rax,rcx,rdx
    emit({0x41, 0x50, 0x41, 0x51, 0x41, 0x52, 0x41, 0x53}); // push r8-r11
    emit({0x48, 0x81, 0xEC, 0x80, 0x00, 0x00, 0x00});       // sub rsp,0x80
    for (int i = 0; i < 6; ++i)                              // movdqu [rsp+0x20+i*16],xmmN
        emit({0xF3, 0x0F, 0x7F, static_cast<uint8_t>(0x44 | (i << 3)), 0x24,
              static_cast<uint8_t>(0x20 + i * 0x10)});
    emit({0x48, 0xB8}); emit64(tick);                        // mov rax, tick
    emit({0xFF, 0xD0});                                      // call rax
    for (int i = 0; i < 6; ++i)                              // movdqu xmmN,[rsp+0x20+i*16]
        emit({0xF3, 0x0F, 0x6F, static_cast<uint8_t>(0x44 | (i << 3)), 0x24,
              static_cast<uint8_t>(0x20 + i * 0x10)});
    emit({0x48, 0x81, 0xC4, 0x80, 0x00, 0x00, 0x00});       // add rsp,0x80
    emit({0x41, 0x5B, 0x41, 0x5A, 0x41, 0x59, 0x41, 0x58}); // pop r11-r8
    emit({0x5A, 0x59, 0x58});                               // pop rdx,rcx,rax
    memcpy(p, expected, kHookLen); p += kHookLen;           // original instructions
    emit({0x49, 0xBB}); emit64(retAddr);                    // mov r11, retAddr
    emit({0x41, 0xFF, 0xE3});                               // jmp r11

    // patch: jmp [rip+0]; dq tramp — 14 bytes + NOP fill to kHookLen
    uint8_t patch[kHookLen] = {0xFF, 0x25, 0x00, 0x00, 0x00, 0x00};
    memcpy(patch + 6, &t, 8);
    memset(patch + 14, 0x90, kHookLen - 14);

    DWORD old;
    if (!VirtualProtect(reinterpret_cast<void*>(hookAddr), kHookLen,
                        PAGE_EXECUTE_READWRITE, &old)) return false;
    memcpy(reinterpret_cast<void*>(hookAddr), patch, kHookLen);
    VirtualProtect(reinterpret_cast<void*>(hookAddr), kHookLen, old, &old);
    FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(hookAddr), kHookLen);
    return true;
}

// second hook site: call at 0x856299 (merge point after the freecam update
// block). Pattern = `mov [r15+0x18], r14` followed by the call.
const uint8_t kHook2Sig[] = {0x4D, 0x89, 0x77, 0x18};
const size_t kHook2Len = 5;  // the call instruction we replace

uintptr_t FindHook2(uintptr_t moduleBase) {
    size_t size = 0;
    uintptr_t text = TextSection(moduleBase, size);
    if (!text) return 0;
    for (size_t i = 0; i + 9 <= size; ++i) {
        const uint8_t* p = reinterpret_cast<const uint8_t*>(text + i);
        if (memcmp(p, kHook2Sig, sizeof(kHook2Sig)) == 0 && p[4] == 0xE8)
            return text + i + 4;
    }
    return 0;
}

// allocate executable memory within ±2GB of site so a 5-byte E9 reaches it
uintptr_t AllocNear(uintptr_t site) {
    for (uintptr_t base = (site + 0x1000) & ~0xFFFFull;
         base < site + 0x7FFF0000; base += 0x10000) {
        void* p = VirtualAlloc(reinterpret_cast<void*>(base), 0x1000,
                               MEM_RESERVE | MEM_COMMIT, PAGE_EXECUTE_READWRITE);
        if (p) return reinterpret_cast<uintptr_t>(p);
    }
    for (uintptr_t base = (site - 0x1000) & ~0xFFFFull;
         base > site - 0x7FFF0000 && base != 0; base -= 0x10000) {
        void* p = VirtualAlloc(reinterpret_cast<void*>(base), 0x1000,
                               MEM_RESERVE | MEM_COMMIT, PAGE_EXECUTE_READWRITE);
        if (p) return reinterpret_cast<uintptr_t>(p);
    }
    return 0;
}

bool InstallHook2(uintptr_t site, uintptr_t callTarget) {
    uint8_t* p = reinterpret_cast<uint8_t*>(site);
    if (*p != 0xE8) return false;
    uintptr_t tramp = AllocNear(site);
    if (!tramp) return false;

    uintptr_t retAddr = site + kHook2Len;
    uintptr_t tick = reinterpret_cast<uintptr_t>(&CameraSmoothTick2);
    uint8_t* w = reinterpret_cast<uint8_t*>(tramp);
    auto emit = [&](std::initializer_list<uint8_t> b) {
        for (uint8_t x : b) *w++ = x;
    };
    auto emit64 = [&](uintptr_t v) { memcpy(w, &v, 8); w += 8; };

    // save volatile regs + xmm0-5, pass r15 as the tick's argument
    emit({0x50, 0x51, 0x52});
    emit({0x41, 0x50, 0x41, 0x51, 0x41, 0x52, 0x41, 0x53});
    emit({0x48, 0x81, 0xEC, 0x80, 0x00, 0x00, 0x00});
    for (int i = 0; i < 6; ++i)
        emit({0xF3, 0x0F, 0x7F, static_cast<uint8_t>(0x44 | (i << 3)), 0x24,
              static_cast<uint8_t>(0x20 + i * 0x10)});
    emit({0x4C, 0x89, 0xF9});                                // mov rcx, r15
    emit({0x48, 0xB8}); emit64(tick);                        // mov rax, tick
    emit({0xFF, 0xD0});                                      // call rax
    for (int i = 0; i < 6; ++i)
        emit({0xF3, 0x0F, 0x6F, static_cast<uint8_t>(0x44 | (i << 3)), 0x24,
              static_cast<uint8_t>(0x20 + i * 0x10)});
    emit({0x48, 0x81, 0xC4, 0x80, 0x00, 0x00, 0x00});
    emit({0x41, 0x5B, 0x41, 0x5A, 0x41, 0x59, 0x41, 0x58});
    emit({0x5A, 0x59, 0x58});
    emit({0x48, 0xB8}); emit64(callTarget);                  // mov rax, callTarget
    emit({0xFF, 0xD0});                                      // call rax (orig insn)
    emit({0x49, 0xBB}); emit64(retAddr);                     // mov r11, retAddr
    emit({0x41, 0xFF, 0xE3});                                // jmp r11

    DWORD old;
    if (!VirtualProtect(p, kHook2Len, PAGE_EXECUTE_READWRITE, &old)) return false;
    int32_t rel = static_cast<int32_t>(tramp - site - kHook2Len);
    p[0] = 0xE9;
    memcpy(p + 1, &rel, 4);
    VirtualProtect(p, kHook2Len, old, &old);
    FlushInstructionCache(GetCurrentProcess(), p, kHook2Len);
    return true;
}

} // namespace

#ifdef DIAG
// ---------------------------------------------------------------------------
// Diagnostic build: set DR0-3 write breakpoints on cam+0x80..0xBF for all
// threads. VEH records the writer instruction addresses, then removes them.
// ---------------------------------------------------------------------------
#include <tlhelp32.h>
#include <cstdio>

namespace {
volatile uintptr_t g_cam = 0;
volatile uintptr_t g_owner = 0;

struct Hit { uintptr_t ripRva; uintptr_t addrOff; int count; };
Hit g_hits[256];
volatile int g_nhits = 0;

LONG WINAPI DrVeh(EXCEPTION_POINTERS* ep) {
    auto rec = ep->ExceptionRecord;
    if (rec->ExceptionCode != EXCEPTION_SINGLE_STEP) return EXCEPTION_CONTINUE_SEARCH;
    auto ctx = ep->ContextRecord;
    if (!(ctx->Dr6 & 0xF)) return EXCEPTION_CONTINUE_SEARCH;
    uintptr_t rip = ctx->Rip;
    uintptr_t addr = rec->ExceptionInformation[0];
    if (rip > g_base && rip < g_base + 0x3000000) {
        uintptr_t rva = rip - g_base;
        uintptr_t off = addr >= g_cam ? addr - g_cam : 0;
        bool known = false;
        for (int i = 0; i < g_nhits; ++i) {
            if (g_hits[i].ripRva == rva && g_hits[i].addrOff == off) {
                g_hits[i].count++; known = true; break;
            }
        }
        if (!known && g_nhits < 256) {
            int i = g_nhits++;
            g_hits[i].ripRva = rva; g_hits[i].addrOff = off; g_hits[i].count = 1;
        }
    }
    ctx->Dr6 = 0;
    ctx->EFlags |= 0x10000;
    return EXCEPTION_CONTINUE_EXECUTION;
}

void SetDrOnThreads(uintptr_t watch0, bool enable) {
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) return;
    THREADENTRY32 te{sizeof(te)};
    DWORD pid = GetCurrentProcessId();
    if (Thread32First(snap, &te)) {
        do {
            if (te.th32OwnerProcessID != pid || te.th32ThreadID == GetCurrentThreadId())
                continue;
            HANDLE th = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_SET_CONTEXT,
                                   FALSE, te.th32ThreadID);
            if (!th) continue;
            SuspendThread(th);
            CONTEXT ctx{};
            ctx.ContextFlags = CONTEXT_DEBUG_REGISTERS;
            if (GetThreadContext(th, &ctx)) {
                if (enable) {
                    ctx.Dr0 = watch0;      ctx.Dr1 = watch0 + 0x10;
                    ctx.Dr2 = watch0 + 0x20; ctx.Dr3 = watch0 + 0x30;
                    // L0..L3 local enables + RW=write(01) + LEN=4B(11) each
                    ctx.Dr7 = 0x1 | (0x1 << 2) | (0x1 << 4) | (0x1 << 6)
                            | (0x1 << 16) | (0x3 << 18)
                            | (0x1 << 20) | (0x3 << 22)
                            | (0x1 << 24) | (0x3 << 26)
                            | (0x1 << 28) | (0x3 << 30);
                } else {
                    ctx.Dr7 &= ~0xFFFFFULL;
                }
                SetThreadContext(th, &ctx);
            }
            ResumeThread(th);
            CloseHandle(th);
        } while (Thread32Next(snap, &te));
    }
    CloseHandle(snap);
}

DWORD WINAPI DiagThread(LPVOID) {
    // wait for cam to exist (photo mode)
    for (;;) {
        if (g_slot) {
            uintptr_t p = *reinterpret_cast<uintptr_t*>(g_slot);
            if (p) {
                uintptr_t c = *reinterpret_cast<uintptr_t*>(p + 8);
                if (c && WorldLooksValid(*reinterpret_cast<Mat*>(c + 0x80))) {
                    g_owner = p; g_cam = c; break;
                }
            }
        }
        Sleep(200);
    }
    AddVectoredExceptionHandler(1, DrVeh);
    // arm only while photo/freecam flag is set -> catch the freecam writer
    while (!*reinterpret_cast<uint8_t*>(g_flagAddr)) Sleep(200);
    SetDrOnThreads(g_cam + 0x00, true);   // watch the param/source block
    Sleep(8000);
    SetDrOnThreads(0, false);

    wchar_t path[MAX_PATH];
    GetModuleFileNameW(nullptr, path, MAX_PATH);
    wcscpy_s(wcsrchr(path, L'\\') + 1,
             MAX_PATH - (wcsrchr(path, L'\\') + 1 - path), L"CameraSmooth-dbg.log");
    FILE* f = nullptr;
    _wfopen_s(&f, path, L"w");
    if (f) {
        fprintf(f, "cam=%p hits=%d\n", (void*)g_cam, g_nhits);
        for (int i = 0; i < g_nhits; ++i)
            fprintf(f, "rip=+%#08llx camoff=+%#03llx x%d\n",
                    (unsigned long long)g_hits[i].ripRva,
                    (unsigned long long)g_hits[i].addrOff,
                    g_hits[i].count);
        fclose(f);
    }
    return 0;
}
} // namespace
#endif

BOOL WINAPI DllMain(HINSTANCE inst, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        DisableThreadLibraryCalls(inst);
        g_base = reinterpret_cast<uintptr_t>(GetModuleHandleW(L"SnowRunner.exe"));
        if (!g_base) return TRUE;
        uintptr_t callSite = FindCallSite(g_base);
        if (!callSite) return TRUE;
        g_slot = ResolveSlotFromCallSite(callSite);
        g_flagAddr = g_base + kFlyFlagRva;
        QueryPerformanceFrequency(&g_freq);
        QueryPerformanceCounter(&g_prev);
#ifdef DIAG
        CreateThread(nullptr, 0, DiagThread, nullptr, 0, nullptr);
#else
        Trace("init");
        uintptr_t fn = FindEpilogue(g_base);
        TraceU("hook1 site ", fn);
        if (fn) { bool ok = InstallHook(fn); Trace(ok ? "hook1 ok" : "hook1 FAIL"); }
        uintptr_t site2 = FindHook2(g_base);
        TraceU("hook2 site ", site2);
        if (site2) {
            int32_t rel = *reinterpret_cast<int32_t*>(site2 + 1);
            bool ok = InstallHook2(site2, site2 + 5 + rel);
            Trace(ok ? "hook2 ok" : "hook2 FAIL");
        }
#endif
    }
    return TRUE;
}
