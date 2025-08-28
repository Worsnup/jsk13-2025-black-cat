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
    const radius = 38;              // disk radius (px)
    const totalLength = 60 * 120;   // 60 m at 120 px/m -> px
    const TOTAL_POINTS = 500;       // total rope particles (including wound)
    const ROPE_ITERS = 5;           // distance-constraint iterations
    const SUBSTEPS = 2;             // verlet substeps for stability

    // Derived counts
    const SEG_LEN = (totalLength / (TOTAL_POINTS - 1)); // rest length between particles

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
    let freeCount = 4;  // start with a short dangling tail
    let exitAngle = Math.random() * Math.PI * 2; // where the rope leaves the disk

    // ===== Utility =====
    const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

    // Build a tangled path inside the disk and sample it at ~SEG_LEN spacing to get local anchors.
    // The first sampled point is on the outer rim at exitAngle; subsequent points wander through the interior.
    function buildWoundPath() {
        // New approach: layered great-circle bands with jitter, ensuring near-uniform
        // spacing and a filled silhouette that matches the old shaded ball look.
        const maxR = radius - 2;
        const minR = radius;

        const layers = [
            {tilt: -0.35, count: 2, wobR: 0.06, wobT: 0.20},
            {tilt: 0.55, count: 2, wobR: 0.05, wobT: 0.18},
            {tilt: 1.10, count: 2, wobR: 0.05, wobT: 0.15},
        ];

        const locals = [];

        function pushIfFar(p) {
            const last = locals[locals.length - 1];
            if (!last || Math.hypot(p.x - last.x, p.y - last.y) >= SEG_LEN) locals.push(p);
        }

        // Seed on rim at exitAngle for a clean tail join
        pushIfFar({x: maxR * Math.cos(exitAngle), y: maxR * Math.sin(exitAngle)});

        // Interleave tilted bands
        for (const L of layers) {
            for (let k = 0; k < L.count; k++) {
                const phase = (k + 0.5) / L.count * Math.PI * 0.7;
                for (let a = -Math.PI * 0.98; a <= Math.PI * 0.98; a += Math.PI) {
                    const ca = Math.cos(a), sa = Math.sin(a);
                    const ct = Math.cos(L.tilt + phase), st = Math.sin(L.tilt + phase);
                    let x = maxR * (ca * ct - sa * st);
                    let y = maxR * (ca * st + sa * ct);
                    // inward bias so we don't leave a donut hole
                    const t = (a + Math.PI) / (2 * Math.PI);
                    const inward = 0.25 + 0.75 * Math.sin(t * Math.PI);
                    const r = maxR - (maxR - minR) * (L.wobR * inward);
                    const len = Math.hypot(x, y) || 1;
                    x *= r / len;
                    y *= r / len;
                    // small jitter to break perfect bands
                    const jitter = (Math.sin((a + phase) * 23.71) * 0.5 + 0.5) * L.wobT;
                    const ang = Math.atan2(y, x) + jitter * 0.15;
                    const rr = Math.hypot(x, y);
                    x = rr * Math.cos(ang);
                    y = rr * Math.sin(ang);
                    pushIfFar({x, y});
                }
            }
        }

        // Ensure we have enough points; sprinkle interiors if needed
        while (locals.length < TOTAL_POINTS) {
            const ang = Math.random() * Math.PI * 2;
            const rr = minR + (maxR - minR) * Math.random();
            pushIfFar({x: rr * Math.cos(ang), y: rr * Math.sin(ang)});
        }

        for (let i = 0; i < TOTAL_POINTS; i++) {
            const p = locals[i] || locals[locals.length - 1];
            woundLocal[i].x = p.x;
            woundLocal[i].y = p.y;
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
        const segs = clamp(Math.floor(Jmag / 25), 1, 8);
        maybeFreeNext(segs);

        hitCooldown = 0.10;
    }

    // === Rendering ===
    function drawYarn() {
        // if (freeCount < 1) return;
        const P = points;
        const n = P.length; // Math.max(2, freeCount + 1);
        const mid = (a, b) => ({x: (a.x + b.x) * 0.5, y: (a.y + b.y) * 0.5});
        const PAL = [{a: 0.42, w: 3.0, col: '#f9a9c8'}, {a: 0.32, w: 3.0, col: '#8a2b4f'}, {
            a: 0.22, w: 3.0, col: '#6b1f3d'
        }];
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        for (const L of PAL) {
            ctx.strokeStyle = L.col;
            ctx.globalAlpha = L.a;
            ctx.lineWidth = L.w;
            let m01 = mid(P[0], P[1]);
            ctx.beginPath();
            ctx.moveTo(P[0].x, P[0].y);
            ctx.quadraticCurveTo(P[0].x, P[0].y, m01.x, m01.y);
            ctx.stroke();
            for (let i = 1; i <= n - 2; i++) {
                const mPrev = mid(P[i - 1], P[i]);
                const mNext = mid(P[i], P[i + 1]);
                ctx.beginPath();
                ctx.moveTo(mPrev.x, mPrev.y);
                ctx.quadraticCurveTo(P[i].x, P[i].y, mNext.x, mNext.y);
                ctx.stroke();
            }
        }
        ctx.globalAlpha = 1;
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

        drawYarn();

        // UI: free length
        const freeLen = SEG_LEN * (freeCount - 1);
        ctx.fillStyle = 'rgba(255,255,255,0.9)';
        ctx.font = '14px system-ui, sans-serif';
        ctx.fillText('yarn: ' + formatMeters(freeLen), 12, 22);

        requestAnimationFrame(step);
    }

    requestAnimationFrame(step);
});
