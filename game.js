document.addEventListener('DOMContentLoaded', () => {
    // === Canvas bootstrap ===
    const content = document.getElementById('content') || document.body;
    const c = document.createElement('canvas');
    const ctx = c.getContext('2d');
    content.innerHTML = '';
    document.body.style.margin = '0';
    document.body.style.overflow = 'hidden';
    c.style.display = 'block';
    content.appendChild(c);

    // DPR + resize
    const DPR = Math.min(2, (window.devicePixelRatio || 1));

    function resize() {
        const w = window.innerWidth || document.documentElement.clientWidth;
        const h = window.innerHeight || document.documentElement.clientHeight;
        c.style.width = w + 'px';
        c.style.height = h + 'px';
        c.width = (w * DPR) | 0;
        c.height = (h * DPR) | 0;
        ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
    }

    window.addEventListener('resize', resize);
    resize();

    // === World params ===
    const grav = 1600;       // px/s^2
    const rest = 0.55;       // restitution for disk collisions
    const fricGround = 0.015; // simple rolling friction (per frame when grounded)
    const fricAir = 0.997;   // gentle air drag for disk
    const wallRest = 0.5;

    // === Yarn geometry ===
    const radius = 46;              // disk radius (px)
    const rCore = radius * 0.45;    // inner core radius (px)
    const totalLength = 60 * 120;   // 60 m at 120 px/m -> px
    const SEG_LEN = 8;              // rope segment rest length (px)
    const ROPE_ITERS = 5;           // distance-constraint iterations
    const SUBSTEPS = 2;             // verlet substeps for stability

    // Derived counts
    const TOTAL_POINTS = Math.max(8, Math.floor(totalLength / SEG_LEN) + 1);

    // ==== Disk (rigid) state (integrated with Verlet) ====
    const world = {w: 0, h: 0};

    function syncWorld() {
        world.w = c.clientWidth;
        world.h = c.clientHeight;
    }

    syncWorld();

    const ball = {
        x: world.w * 0.5, y: world.h * 0.25, px: world.w * 0.5 - 120 * (1 / 60), // initial x-velocity ~120 px/s
        py: world.h * 0.25, angle: 0, pang: 0, // previous angle (for computing spin if needed)
        grounded: false,
    };

    // ==== Rope data (single particle chain) ====
    // points[0] is the free tail end; points[freeCount] is the head touching the disk; points[TOTAL_POINTS-1] deepest in the ball
    const points = new Array(TOTAL_POINTS).fill(0).map(() => ({x: 0, y: 0, px: 0, py: 0}));
    // Local (disk space) anchors for the wound portion (i >= freeCount). These coordinates are rigidly attached to the disk.
    const woundLocal = new Array(TOTAL_POINTS).fill(0).map(() => ({x: 0, y: 0}));
    // Index of the *first* wound particle in the chain (the one on the surface exit). All indices < freeCount are free.
    let freeCount = 2;  // start with a short dangling tail
    let exitAngle = Math.random() * Math.PI * 2; // where the rope leaves the disk

    // ===== Utility =====
    const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
    const lerp = (a, b, t) => a + (b - a) * t;

    // Pseudo-random but deterministic for a given seed
    function hash(n) {
        const s = Math.sin(n) * 43758.5453;
        return s - Math.floor(s);
    }

    function noise1(t) { // cheap smooth-ish noise
        const i = Math.floor(t);
        const f = t - i;
        const a = hash(i);
        const b = hash(i + 1);
        const u = f * f * (3 - 2 * f);
        return a * (1 - u) + b * u;
    }

    // Build a tangled path inside the disk and sample it at ~SEG_LEN spacing to get local anchors.
    // The first sampled point is on the outer rim at exitAngle; subsequent points wander through the interior.
    function buildWoundPath() {
        const maxR = radius - 3; // keep strands inside the disk visually
        const minR = rCore;

        // Path in (r, theta) as a function of t. We deliberately stir multiple frequencies to get a "jumbled" look.
        const phi1 = Math.random() * Math.PI * 2;
        const phi2 = Math.random() * Math.PI * 2;
        const phi3 = Math.random() * Math.PI * 2;

        function polarAt(t) {
            // t in [0, T]; we won't rely on its absolute scale, only arc-length sampling
            const baseTheta = 24 * Math.PI * t;                           // many wraps
            const twist = 1.3 * Math.sin(6 * t + phi1) + 0.45 * Math.sin(15 * t + phi2);
            const theta = baseTheta + twist + exitAngle;                  // rotate so path starts near exit

            // Radius oscillates between core and rim with additional wobble
            const osc = 0.5 + 0.5 * Math.sin(2 * Math.PI * t + phi3);
            const wob = 0.15 * (noise1(t * 3.1) - 0.5) + 0.08 * Math.sin(9.0 * t + phi2);
            const r = clamp(lerp(minR, maxR, clamp(osc + wob, 0, 1)), minR, maxR);
            return {r, theta};
        }

        // Convert polar to local XY
        const toXY = (p) => ({x: p.r * Math.cos(p.theta), y: p.r * Math.sin(p.theta)});

        // Arc-length sample
        const locals = [];
        // Seed first point explicitly on the rim at exitAngle (rope exit)
        locals.push({x: maxR * Math.cos(exitAngle), y: maxR * Math.sin(exitAngle)});
        let last = locals[0];

        // March t forward gathering points until we fill the whole chain
        let t = 0, dt = 0.003; // small step for good length accuracy
        while (locals.length < TOTAL_POINTS) {
            t += dt;
            const p = toXY(polarAt(t));
            const dx = p.x - last.x, dy = p.y - last.y;
            const d = Math.hypot(dx, dy);
            if (d >= SEG_LEN) {
                locals.push(p);
                last = p;
            }
            // Just in case, stop after very long param distance and fill randomly (shouldn't happen under normal params)
            if (t > 200) {
                // fallback: random point on circle (still within disk)
                const ang = Math.random() * Math.PI * 2;
                const rr = lerp(minR, maxR, Math.random());
                locals.push({x: rr * Math.cos(ang), y: rr * Math.sin(ang)});
            }
        }

        // We built from exit outward; we want chain order from exit (surface) -> deep interior,
        // so the last local becomes the chain end (points[TOTAL_POINTS-1]). That's already true.
        for (let i = 0; i < TOTAL_POINTS; i++) {
            woundLocal[i].x = locals[i].x;
            woundLocal[i].y = locals[i].y;
        }
    }

    // Initialize chain points and wound path
    function initRope() {
        buildWoundPath();

        // Place the disk and all wound points in world space
        const cx = ball.x, cy = ball.y;

        // Set up entire chain positions. Start with a short downward dangling tail from the exit point.
        // points[freeCount] must sit exactly on the exit point on the surface.
        function worldFromLocal(ix) {
            const lx = woundLocal[ix].x, ly = woundLocal[ix].y;
            const cosA = Math.cos(ball.angle), sinA = Math.sin(ball.angle);
            return {
                x: cx + (lx * cosA - ly * sinA), y: cy + (lx * sinA + ly * cosA)
            };
        }

        // Position wound portion
        for (let i = freeCount; i < TOTAL_POINTS; i++) {
            const wpos = worldFromLocal(i);
            points[i].x = points[i].px = wpos.x;
            points[i].y = points[i].py = wpos.y;
        }

        // Build a simple tail: points[freeCount-1] sits one segment away along the tangent (leaving clockwise)
        const ex = Math.cos(exitAngle), ey = Math.sin(exitAngle);
        const tx = -ey, ty = ex;
        const head = points[freeCount];

        // Tail seed
        const p1x = head.x + tx * SEG_LEN;
        const p1y = head.y + ty * SEG_LEN + 2; // tiny sag
        const p0x = p1x + tx * SEG_LEN;
        const p0y = p1y + ty * SEG_LEN + 2;

        points[freeCount - 1].x = points[freeCount - 1].px = p1x;
        points[freeCount - 1].y = points[freeCount - 1].py = p1y;
        points[0].x = points[0].px = p0x;
        points[0].y = points[0].py = p0y;
    }

    initRope();

    // === Input handling ===
    const mouse = {x: 0, y: 0, down: false};
    let lastMouse = {x: 0, y: 0, t: performance.now()};
    let hitCooldown = 0; // seconds between impulses

    function setMouse(e) {
        const rect = c.getBoundingClientRect();
        mouse.x = (e.clientX - rect.left);
        mouse.y = (e.clientY - rect.top);
    }

    c.addEventListener('mousemove', (e) => setMouse(e));
    c.addEventListener('mousedown', (e) => {
        mouse.down = true;
        setMouse(e);
    });
    c.addEventListener('mouseup', () => {
        mouse.down = false;
    });
    c.addEventListener('touchstart', (e) => {
        const t = e.touches[0];
        setMouse(t);
        mouse.down = true;
        e.preventDefault();
    });
    c.addEventListener('touchmove', (e) => {
        const t = e.touches[0];
        setMouse(t);
        e.preventDefault();
    });
    c.addEventListener('touchend', () => {
        mouse.down = false;
    });

    // === Physics core ===
    function integrateDisk(dt) {
        // Verlet integrate disk center
        const vx = (ball.x - ball.px);
        const vy = (ball.y - ball.py);

        ball.px = ball.x;
        ball.py = ball.y;
        ball.x += vx * fricAir;
        ball.y += vy * fricAir + grav * dt * dt; // gravity applied in position units

        // Collide with bounds
        const w = world.w, h = world.h;

        ball.grounded = false;

        // Floor
        if (ball.y + radius > h) {
            ball.y = h - radius;
            const vpy = ball.py - ball.y; // previous step vector along y (opposite of velocity)
            ball.py = ball.y + vpy * -rest; // reflect with restitution
            ball.grounded = true;
        }
        // Ceiling
        if (ball.y - radius < 0) {
            ball.y = radius;
            const vpy = ball.py - ball.y;
            ball.py = ball.y + vpy * -wallRest;
        }
        // Left
        if (ball.x - radius < 0) {
            ball.x = radius;
            const vpx = ball.px - ball.x;
            ball.px = ball.x + vpx * -wallRest;
        }
        // Right
        if (ball.x + radius > w) {
            ball.x = w - radius;
            const vpx = ball.px - ball.x;
            ball.px = ball.x + vpx * -wallRest;
        }

        // Rolling when grounded: advance angle according to horizontal travel
        if (ball.grounded) {
            const dx = (ball.x - ball.px);
            ball.angle += dx / radius; // approximate rolling; sign makes visual sense

            // simple kinetic friction -> damp horizontal motion via previous position
            const vx2 = ball.x - ball.px;
            ball.px = ball.x - vx2 * (1 - fricGround);
        }
    }

    function setWoundWorldPositions() {
        // Enforce wound (rigid) positions and keep their previous positions glued too (no velocity)
        const cosA = Math.cos(ball.angle), sinA = Math.sin(ball.angle);
        const cx = ball.x, cy = ball.y;
        for (let i = freeCount; i < TOTAL_POINTS; i++) {
            const lx = woundLocal[i].x, ly = woundLocal[i].y;
            const wx = cx + (lx * cosA - ly * sinA);
            const wy = cy + (lx * sinA + ly * cosA);
            points[i].px = points[i].x = wx;
            points[i].py = points[i].y = wy;
        }
    }

    function integrateTail(dt) {
        // Verlet integrate free particles only
        for (let i = 0; i < freeCount; i++) {
            const p = points[i];
            const vx = (p.x - p.px);
            const vy = (p.y - p.py);
            p.px = p.x;
            p.py = p.y;
            p.x += vx;
            p.y += vy + grav * dt * dt; // gravity

            // Collide with floor
            if (p.y > world.h) {
                p.y = world.h;
                p.py = p.y + (p.y - p.py) * -rest;
            }
            // Ceiling
            if (p.y < 0) {
                p.y = 0;
                p.py = p.y + (p.y - p.py) * -wallRest;
            }
            // Left/Right
            if (p.x < 0) {
                p.x = 0;
                p.px = p.x + (p.x - p.px) * -wallRest;
            }
            if (p.x > world.w) {
                p.x = world.w;
                p.px = p.x + (p.x - p.px) * -wallRest;
            }
        }
    }

    function satisfyConstraints() {
        // Distance constraints along the chain; wound nodes are treated as fixed.
        for (let iter = 0; iter < ROPE_ITERS; iter++) {
            for (let i = 1; i < TOTAL_POINTS; i++) {
                const a = points[i - 1];
                const b = points[i];
                let dx = b.x - a.x;
                let dy = b.y - a.y;
                let d = Math.hypot(dx, dy) || 1e-6;
                const diff = (d - SEG_LEN) / d;

                // Movement shares: if a or b is wound (i >= freeCount), we only move the free one.
                let moveA = 0.5, moveB = 0.5;
                if (i >= freeCount && i - 1 >= freeCount) {
                    moveA = 0;
                    moveB = 0;
                } else if (i >= freeCount) {
                    moveA = 1;
                    moveB = 0;
                } else if (i - 1 >= freeCount) {
                    moveA = 0;
                    moveB = 1;
                }

                // Apply corrections
                a.x += dx * diff * moveA;
                a.y += dy * diff * moveA;
                b.x -= dx * diff * moveB;
                b.y -= dy * diff * moveB;

                // Keep free particles above floor minimally during solving
                if (i - 1 < freeCount) {
                    if (a.y > world.h) a.y = world.h;
                }
                if (i < freeCount) {
                    if (b.y > world.h) b.y = world.h;
                }
            }
        }
    }

    // Auto-free logic: if the head link is overstretched, pop another particle off the disk
    function maybeFreeNext(bySegments = 1) {
        for (let k = 0; k < bySegments; k++) {
            if (freeCount >= TOTAL_POINTS - 1) return; // nothing left to free (keep one on disk for exit point)
            // Promote points[freeCount] to free: keep its current world position & previous position (so no pop)
            freeCount++;
        }
    }

    function tensionRelease() {
        // Measure stretch at the boundary link [freeCount-1] <-> [freeCount]
        if (freeCount <= 0) return;
        const a = points[freeCount - 1];
        const b = points[freeCount];
        const d = Math.hypot(b.x - a.x, b.y - a.y);
        if (d > SEG_LEN * 1.35) {
            const extra = Math.min(4, Math.floor((d / SEG_LEN - 1.35) * 3) + 1);
            maybeFreeNext(extra);
            // After freeing, immediately glue wound positions again to avoid snapping while the boundary moves
            setWoundWorldPositions();
        }
    }

    function handleMouseImpulses(dt, now) {
        hitCooldown = Math.max(0, hitCooldown - dt);
        const mdx = mouse.x - lastMouse.x;
        const mdy = mouse.y - lastMouse.y;
        const mdT = Math.max(1e-3, (now - lastMouse.t) / 1000);
        const mvx = mdx / mdT, mvy = mdy / mdT;

        lastMouse.x = mouse.x;
        lastMouse.y = mouse.y;
        lastMouse.t = now;

        const dx = mouse.x - ball.x;
        const dy = mouse.y - ball.y;
        const dist = Math.hypot(dx, dy);

        if (!mouse.down) return;
        if (hitCooldown > 0) return;
        if (dist > radius + 3) return;

        // Apply an impulse to the disk by modifying its previous position (Verlet velocity hack)
        const nx = dx / (dist || 1);
        const ny = dy / (dist || 1);

        // Approach speed into the center produces larger impulses
        const approach = -(nx * mvx + ny * mvy);
        const base = 520 + clamp(approach * 0.7, -250, 900);

        // Velocity after impulse v' = v + J/m. In Verlet: px' = x - v'
        const Jx = -nx * base;
        const Jy = -ny * base - 150; // slight upward bias for fun

        const vx = (ball.x - ball.px) + (Jx * (1 / 60));
        const vy = (ball.y - ball.py) + (Jy * (1 / 60));
        ball.px = ball.x - vx;
        ball.py = ball.y - vy;

        // Free some segments depending on impulse size
        const Jmag = Math.hypot(Jx, Jy);
        const segs = clamp(Math.floor(Jmag / 250), 1, 8);
        maybeFreeNext(segs);

        hitCooldown = 0.10;
    }

    // === Rendering ===
    function drawRope() {
        // Render the entire chain as a smoothed polyline
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        ctx.strokeStyle = 'rgba(230,160,190,0.9)';
        ctx.lineWidth = 2;

        // We draw segments between successive midpoints for smoothness
        const P = points;
        const n = TOTAL_POINTS;
        const mid = (a, b) => ({x: (a.x + b.x) * 0.5, y: (a.y + b.y) * 0.5});

        if (n < 2) return;

        // Start cap
        let m01 = mid(P[0], P[1]);
        ctx.beginPath();
        ctx.moveTo(P[0].x, P[0].y);
        ctx.quadraticCurveTo(P[0].x, P[0].y, m01.x, m01.y);
        ctx.stroke();

        // Middles
        for (let i = 1; i <= n - 2; i++) {
            const mPrev = mid(P[i - 1], P[i]);
            const mNext = mid(P[i], P[i + 1]);
            ctx.beginPath();
            ctx.moveTo(mPrev.x, mPrev.y);
            ctx.quadraticCurveTo(P[i].x, P[i].y, mNext.x, mNext.y);
            ctx.stroke();
        }

        // End cap
        const mnm1 = mid(P[n - 2], P[n - 1]);
        ctx.beginPath();
        ctx.moveTo(mnm1.x, mnm1.y);
        ctx.quadraticCurveTo(P[n - 1].x, P[n - 1].y, P[n - 1].x, P[n - 1].y);
        ctx.stroke();
    }

    function formatMeters(px) {
        return (px / 120).toFixed(1) + ' m';
    }

    // === Main loop ===
    let last = performance.now();

    function step(now) {
        syncWorld();
        const rawDt = (now - last) / 1000;
        const dt = clamp(rawDt, 0, 0.033);
        last = now;

        // Substep for stability
        const steps = Math.max(1, SUBSTEPS);
        const h = dt / steps;

        for (let s = 0; s < steps; s++) {
            // 1) Disk
            integrateDisk(h);
            // 2) Glue wound nodes to disk pose
            setWoundWorldPositions();
            // 3) Tail integrate
            integrateTail(h);
            // 4) Solve distances (tail vs head link vs wound)
            satisfyConstraints();
            // 5) Release on tension
            tensionRelease();
            // 6) Mouse impulses
            handleMouseImpulses(h, now);
        }

        // === Render ===
        // bg
        ctx.clearRect(0, 0, world.w, world.h);
        ctx.fillStyle = '#36a';
        ctx.fillRect(0, 0, world.w, world.h);

        // ground line
        ctx.strokeStyle = 'rgba(255,255,255,0.08)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, world.h - 1);
        ctx.lineTo(world.w, world.h - 1);
        ctx.stroke();

        // rope then ball
        drawRope();

        // UI: free length
        const freeLen = SEG_LEN * (freeCount - 1);
        ctx.fillStyle = 'rgba(255,255,255,0.9)';
        ctx.font = '14px system-ui, sans-serif';
        ctx.fillText('yarn: ' + formatMeters(freeLen), 12, 22);

        requestAnimationFrame(step);
    }

    requestAnimationFrame(step);
});
