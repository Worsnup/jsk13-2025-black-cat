document.addEventListener('DOMContentLoaded', () => {
    // Create and attach canvas
    const content = document.getElementById('content');
    const c = document.createElement('canvas');
    const ctx = c.getContext('2d');
    content.innerHTML = '';
    document.body.style.margin = '0';
    document.body.style.overflow = 'hidden';
    c.style.display = 'block';
    content.appendChild(c);

    // Size & resize handling
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

    // ===== World params =====
    const grav = 1200;        // px/s^2
    const rest = 0.66;        // coefficient of restitution (floor/ceiling)
    const wallRest = 0.55;    // side restitution
    const muGround = 0.6;     // Coulomb friction (floor/ceiling) — dimensionless
    const muWall = 0.5;       // Coulomb friction (walls)
    const radius = 40;        // yarn radius in px

    // Rigid body params (solid sphere about center: I = 2/5 m r^2)
    const mass = 1;           // arbitrary units — only ratios matter here
    const inertia = (2 / 5) * mass * radius * radius;

    // ===== Spool / rope params =====
    const pxPerMeter = 120;                 // for UI formatting only
    const L_total = 60 * pxPerMeter;        // total yarn length in px (~60 m)
    let L_free = 0;                         // current unspooled length (px)
    const r_core = 0.6 * radius;            // minimum wound radius
    const alpha = (radius - r_core) / L_total; // maps L_total -> r_core

    // Friction / capstan-like parameters
    const v_max_feed = 2400;                 // px/s cap for feed rate
    // Hit-driven release tuning (convert dynamics -> length)
    const k_release_impulse = 0.015;        // px per unit impulse magnitude
    const k_release_angle = 40;           // px per radian of velocity angle change
    const k_release_momentum = 0.035;        // px per unit increase in |p| (= m*|v|)

    // Exit state
    let exitAngle = Math.random() * Math.PI * 2; // local angle on ball

    // IK rigid-rod rope (visual-only; one-way coupling)
    let SEG_LEN = 16;  // target segment length used in constraints
    const ROPE_ITERS = 6;    // a touch firmer for less numeric slop
    const ROPE_DAMP = 0.992; // higher damping to quell high-freq jiggle
    const ROPE_SUBSTEPS = 2; // integrate/solve in smaller steps for stability
    const ANCHOR_SMOOTH = 0.18; // per-frame smoothing toward moving anchor
    const nodes = [];
    let pending_nodes = 0;
    let ropeInited = false;
    // Smoothed anchor we actually pin to (reduces teleports at the head)
    let ropeAX = 0, ropeAY = 0;

    const w = c.clientWidth, h = c.clientHeight;

    function initRope(anchorX, anchorY, tx, ty) {
        nodes.length = 0;
        // Head pinned at anchor + one tail node
        const tailX = anchorX + tx * SEG_LEN;
        const tailY = anchorY + ty * SEG_LEN + 2; // tiny sag
        nodes.push({x: anchorX, y: anchorY, px: anchorX, py: anchorY});
        nodes.push({x: tailX, y: tailY, px: tailX, py: tailY});
        ropeInited = true;
    }

    function targetCountForLength(L) {
        // +1 because count is nodes, segments = nodes-1
        const segsNeeded = Math.max(1, Math.floor(L / SEG_LEN));
        return segsNeeded + 1;
    }

    function addTailSegment() {
        // Duplicate last node to extend, then immediately space it one seg-length away
        const n = nodes.length;
        const a = nodes[n - 2]; // second last
        const b = nodes[n - 1]; // last (tail)
        // Direction from a->b
        let dx = b.x - a.x, dy = b.y - a.y;
        const d = Math.hypot(dx, dy) || 1;
        dx /= d;
        dy /= d;
        // New node positioned continuing the ray by SEG_LEN
        const nx = b.x + dx * SEG_LEN;
        const ny = b.y + dy * SEG_LEN;
        nodes.push({x: nx, y: ny, px: nx, py: ny});
    }

    function ensureRopeForLength(L) {
        const target = targetCountForLength(L);
        // Only grow; never shrink (no re-spool)
        pending_nodes = target - nodes.length;
    }

    function integrateRope(dt) {
        for (let i = 1; i < nodes.length; i++) { // skip pinned head (i=0)
            const n = nodes[i];
            const vx = (n.x - n.px) * ROPE_DAMP;
            const vy = (n.y - n.py) * ROPE_DAMP + grav * dt * dt * 0.35; // slightly gentler accel -> less ringing
            n.px = n.x;
            n.py = n.y;
            n.x += vx;
            n.y += vy;
            if (n.y > h) {
                n.y = h;
                // kill most of the bounce; keep tiny slip to avoid stickiness
                n.py = n.y + (n.y - n.py) * 0.2;
            }
        }
    }

    function satisfyRope(anchorX, anchorY) {
        // FABRIK-style IK with rigid segments.
        // Head is hard-pinned to the anchor; tail is "free" and guided by gravity
        // via the integrateRope() prediction step. This is visual-only and does
        // not feed impulses back into the ball.
        if (nodes.length < 2) {
            // Still pin the head if rope hasn't grown yet
            nodes[0].x = anchorX;
            nodes[0].y = anchorY;
            nodes[0].px = anchorX;
            nodes[0].py = anchorY;
            return;
        }
        // Pin head (base)
        nodes[0].x = anchorX;
        nodes[0].y = anchorY;
        nodes[0].px = anchorX;
        nodes[0].py = anchorY;
        // Use the post-integration tail position as the target
        const ti = nodes.length - 1;
        const tgtX = nodes[ti].x, tgtY = nodes[ti].y;
        for (let iter = 0; iter < ROPE_ITERS; iter++) {
            // ----- Forward reach: place tail at its target and pull chain toward head
            nodes[ti].x = tgtX;
            nodes[ti].y = tgtY;
            for (let i = ti - 1; i >= 0; i--) {
                const nx = nodes[i + 1].x - nodes[i].x;
                const ny = nodes[i + 1].y - nodes[i].y;
                const d = Math.hypot(nx, ny) || 1e-6;
                const r = SEG_LEN / d;
                nodes[i].x = nodes[i + 1].x - nx * r;
                nodes[i].y = nodes[i + 1].y - ny * r;
            }
            // ----- Backward reach: re-pin head and push chain toward tail
            nodes[0].x = anchorX;
            nodes[0].y = anchorY;
            for (let i = 1; i <= ti; i++) {
                const nx = nodes[i].x - nodes[i - 1].x;
                const ny = nodes[i].y - nodes[i - 1].y;
                const d = Math.hypot(nx, ny) || 1e-6;
                const r = SEG_LEN / d;
                nodes[i].x = nodes[i - 1].x + nx * r;
                nodes[i].y = nodes[i - 1].y + ny * r;
            }
        }
    }

    // Light curvature smoothing to damp zig-zag between constraints
    function smoothRope(alpha = 0.1) {
        for (let i = 1; i < nodes.length - 1; i++) {
            const a = nodes[i - 1], b = nodes[i], c = nodes[i + 1];
            const mx = (a.x + c.x) * 0.5;
            const my = (a.y + c.y) * 0.5;
            b.x += (mx - b.x) * alpha;
            b.y += (my - b.y) * alpha;
        }
    }

    // Ball state
    const ball = {
        x: c.clientWidth * 0.5, y: c.clientHeight * 0.25, vx: 140 * (Math.random() * 2 - 1), vy: 0, angle: 0,   // rotation around Z (radians)
        spin: 0     // angular velocity (rad/s)
    };

    // Input state
    let mouse = {x: 0, y: 0, down: false};
    let lastMouse = {x: 0, y: 0, t: performance.now()};
    let hitCooldown = 0; // seconds

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
    // Prevent text selection on drag
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

    // Utility
    const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
    const cross2 = (ax, ay, bx, by) => ax * by - ay * bx; // 2D scalar cross

    // Drawing the yarn ball
    function drawYarn(x, y, r) {
        // Contact shadow (soft, stronger near contact)
        const groundY = c.clientHeight;
        const distToGround = Math.max(0, groundY - (y + r));
        const contactT = 1 - Math.max(0, Math.min(1, distToGround / (r * 1.2)));
        const shadowAlpha = 0.15 + 0.25 * contactT;
        const shadowRY = r * (0.28 + 0.22 * contactT);
        const shadowRX = r * (0.9 - 0.2 * contactT);
        ctx.save();
        ctx.globalAlpha = shadowAlpha;
        ctx.fillStyle = '#000';
        ctx.beginPath();
        ctx.ellipse(x + r * 0.18, Math.min(groundY - 1, y + r + shadowRY * 0.2), shadowRX, shadowRY, 0, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();

        // Base diffuse shading (fabric-like, less plasticky)
        const lgx = x - r * 0.45, lgy = y - r * 0.5; // light direction (up-left)
        const base = ctx.createRadialGradient(lgx, lgy, r * 0.2, x, y, r * 1.05);
        base.addColorStop(0.0, '#ff8cbc');  // light pink
        base.addColorStop(0.45, '#f3669c');
        base.addColorStop(1.0, '#b03a64');  // deep shadowed pink
        ctx.fillStyle = base;
        ctx.beginPath();
        ctx.arc(x, y, r, 0, Math.PI * 2);
        ctx.fill();

        // Clip to sphere and draw texture/strands inside
        ctx.save();
        ctx.beginPath();
        ctx.arc(x, y, r, 0, Math.PI * 2);
        ctx.clip();

        // Slight overall darkening toward rim (fabric Fresnel)
        const rim = ctx.createRadialGradient(x, y, r * 0.2, x, y, r);
        rim.addColorStop(0, 'rgba(0,0,0,0.0)');
        rim.addColorStop(0.7, 'rgba(0,0,0,0.05)');
        rim.addColorStop(1, 'rgba(0,0,0,0.18)');
        ctx.fillStyle = rim;
        ctx.fillRect(x - r, y - r, r * 2, r * 2);

        // Rotate strand texture with spin so the yarn pattern moves with the ball
        ctx.translate(x, y);
        ctx.rotate(ball.angle);

        // Strand parameters
        const layers = [{count: 18, ro: 1.02, w: 1.4, a: 0.40, hue: '#f9a9c8', tilt: -0.35}, // lighter strands
            {count: 16, ro: 0.96, w: 1.2, a: 0.30, hue: '#8a2b4f', tilt: 0.55},  // darker in-between
            {count: 12, ro: 0.88, w: 1.0, a: 0.22, hue: '#6b1f3d', tilt: 1.15}   // deeper grooves
        ];

        // Draw wrapped great-circle arcs with slight jitter
        for (const L of layers) {
            ctx.strokeStyle = L.hue;
            ctx.globalAlpha = L.a;
            ctx.lineCap = 'round';
            for (let i = 0; i < L.count; i++) {
                const t = (i + 0.5) / L.count;
                const phi = (t - 0.5) * Math.PI; // distribute across sphere
                const jitter = (Math.sin(i * 12.9898) * 43758.5453) % 1; // deterministic
                const offset = (jitter - 0.5) * 0.25;                    // small offset
                const tilt = L.tilt + offset;

                // Great-circle param sweep
                ctx.beginPath();
                ctx.lineWidth = L.w * (1.0 - 0.25 * Math.abs(Math.sin(phi))); // thinner near rim
                // Draw as series of small chords approximating a curved wrap
                let first = true;
                for (let a = -Math.PI * 0.95; a <= Math.PI * 0.95; a += Math.PI / 60) {
                    const ca = Math.cos(a), sa = Math.sin(a);
                    // rotate by tilt around Z to get different families of wraps
                    const px = (r * L.ro) * (ca * Math.cos(tilt) - sa * Math.sin(tilt));
                    const py = (r * L.ro) * (ca * Math.sin(tilt) + sa * Math.cos(tilt)) * Math.cos(phi);
                    // simple foreshortening: fade as it goes "behind"
                    const behind = py > 0 ? 1 : 0.65;
                    if (first) {
                        ctx.moveTo(px, py);
                        first = false;
                    } else {
                        ctx.lineTo(px, py);
                    }
                    ctx.globalAlpha = L.a * behind;
                }
                ctx.stroke();
            }
        }

        // Subtle cross-hatch noise for fiber fuzz
        ctx.globalAlpha = 0.06;
        ctx.fillStyle = '#ffffff';
        for (let i = 0; i < 90; i++) {
            const nx = (Math.random() * 2 - 1) * r;
            const ny = (Math.random() * 2 - 1) * r;
            if (nx * nx + ny * ny > r * r) continue;
            const len = 2 + Math.random() * 6;
            const ang = Math.random() * Math.PI;
            ctx.save();
            ctx.translate(nx, ny);
            ctx.rotate(ang);
            ctx.fillRect(-len * 0.5, -0.3, len, 0.6);
            ctx.restore();
        }

        // Ambient occlusion at bottom inside the sphere
        ctx.globalAlpha = 1;
        const ao = ctx.createRadialGradient(0, r * 0.55, r * 0.1, 0, r * 0.55, r * 0.95);
        ao.addColorStop(0, 'rgba(0,0,0,0.25)');
        ao.addColorStop(1, 'rgba(0,0,0,0.0)');
        ctx.fillStyle = ao;
        ctx.fillRect(-r, 0, r * 2, r); // lower hemisphere

        // Small soft specular highlight (fabric sheen, not plastic)
        ctx.save();
        ctx.rotate(-ball.angle); // keep highlight fixed in world/light space
        ctx.translate(-x, -y);
        const hx = x - r * 0.35, hy = y - r * 0.42;
        const spec = ctx.createRadialGradient(hx, hy, 0, hx, hy, r * 0.5);
        spec.addColorStop(0.0, 'rgba(255,255,255,0.35)');
        spec.addColorStop(0.4, 'rgba(255,255,255,0.12)');
        spec.addColorStop(1.0, 'rgba(255,255,255,0.0)');
        ctx.fillStyle = spec;
        ctx.beginPath();
        ctx.arc(x, y, r, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();

        ctx.restore(); // end clip + texture

        // Thin outer rim to suggest thickness
        ctx.strokeStyle = 'rgba(0,0,0,0.22)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(x, y, r - 0.5, 0, Math.PI * 2);
        ctx.stroke();
    }

    // ===== Impulse-based collisions preserving linear & angular momentum =====
    function contactVelocity(vx, vy, omega, rx, ry) {
        // velocity of contact point: v + w x r
        const cx = vx + (-omega * ry);
        const cy = vy + (omega * rx);
        return {cx, cy};
    }

    function applyImpulseAtPoint(Jx, Jy, rx, ry) {
        // Linear
        ball.vx += Jx / mass;
        ball.vy += Jy / mass;
        // Angular (about COM): Δω = (r × J)/I
        const torqueZ = cross2(rx, ry, Jx, Jy);
        ball.spin += torqueZ / inertia;
    }

    function resolvePlaneContact(nx, ny, px, py, e, mu) {
        // nx,ny: unit plane normal pointing INTO the allowed region
        // p = ball center; r = vector from COM to contact point = -n * radius
        const rx = -nx * radius;
        const ry = -ny * radius;

        // Relative velocity at contact
        const cv = contactVelocity(ball.vx, ball.vy, ball.spin, rx, ry);
        const vn = cv.cx * nx + cv.cy * ny; // normal component
        const tx = -ny, ty = nx;            // tangent unit (perp to n)

        // If separating already, skip
        if (vn >= 0) return;

        // Effective mass terms (denominators)
        const rn = cross2(rx, ry, nx, ny); // (r × n)_z
        const rt = cross2(rx, ry, tx, ty); // (r × t)_z
        const kN = 1 / (1 / mass + (rn * rn) / inertia);
        const kT = 1 / (1 / mass + (rt * rt) / inertia);

        // Normal impulse enforces restitution on normal relative velocity
        const Jn = -(1 + e) * vn * kN; // scalar along n
        applyImpulseAtPoint(Jn * nx, Jn * ny, rx, ry);

        // Recompute contact velocity after normal impulse for correct friction
        const cv2 = contactVelocity(ball.vx, ball.vy, ball.spin, rx, ry);
        const vt2 = cv2.cx * tx + cv2.cy * ty;

        // Desired tangential impulse to stick (vt -> 0)
        let Jt = -vt2 * kT; // along tangent
        const maxF = mu * Math.abs(Jn);
        // Clamp to Coulomb cone (sliding if too large)
        const JtClamped = clamp(Jt, -maxF, maxF);
        applyImpulseAtPoint(JtClamped * tx, JtClamped * ty, rx, ry);
    }

    // Position correction to avoid sinking (non-energetic)
    function positionalCorrection(nx, ny, penetration) {
        if (penetration <= 0) return;
        const slop = 0.2; // small tolerance
        const percent = 0.8; // correction strength
        const corr = Math.max(0, penetration - slop) * percent;
        ball.x += nx * corr;
        ball.y += ny * corr;
    }

    function r_spool_of(L) {
        return clamp(radius - alpha * (L_total - L), r_core, radius);
    }

    function drawRope(ballPX, ballPY) {
        // Build a *render-only* polyline that begins exactly on the ball,
        // so visuals stay connected even if IK lags.
        const n = nodes.length;
        if (n < 1) return;
        const P = [{x: ballPX, y: ballPY}, ...nodes]; // virtual head at ball contact
        const vn = P.length;

        // Helper to get tapered width along the rope [0..1]
        const colorAt = (i) => (i < 3 ? 'rgba(255,200,220,0.9)' : 'rgba(230,160,190,0.85)');

        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        ctx.miterLimit = 2;

        if (vn === 2) {
            // Simple 2-point rope
            ctx.strokeStyle = colorAt(1);
            ctx.beginPath();
            ctx.moveTo(P[0].x, P[0].y);
            ctx.lineTo(P[1].x, P[1].y);
            ctx.stroke();
            return;
        }

        // Smooth rope via piecewise quadratic Bézier between successive midpoints.
        // This allows per-segment linewidth/color while keeping a smooth curve.
        const mid = (a, b) => ({x: (a.x + b.x) * 0.5, y: (a.y + b.y) * 0.5});

        // Start cap: from node0 to midpoint(0,1) with control at node0
        let m01 = mid(P[0], P[1]);
        ctx.strokeStyle = colorAt(1);
        ctx.beginPath();
        ctx.moveTo(P[0].x, P[0].y);
        ctx.quadraticCurveTo(P[0].x, P[0].y, m01.x, m01.y);
        ctx.stroke();

        // Middle smoothed segments
        for (let i = 1; i <= vn - 2; i++) {
            const mPrev = mid(P[i - 1], P[i]);
            const mNext = mid(P[i], P[i + 1]);
            ctx.strokeStyle = colorAt(i);
            ctx.beginPath();
            ctx.moveTo(mPrev.x, mPrev.y);
            ctx.quadraticCurveTo(P[i].x, P[i].y, mNext.x, mNext.y);
            ctx.stroke();
        }

        // End cap: from midpoint(n-2,n-1) to node(n-1) with control at node(n-1)
        const mnm1 = mid(P[vn - 2], P[vn - 1]);
        ctx.strokeStyle = colorAt(vn - 1);
        ctx.beginPath();
        ctx.moveTo(mnm1.x, mnm1.y);
        ctx.quadraticCurveTo(P[vn - 1].x, P[vn - 1].y, P[vn - 1].x, P[vn - 1].y);
        ctx.stroke();
    }

    function formatMeters(px) {
        const m = px / pxPerMeter;
        return m.toFixed(1) + ' m';
    }

    // ===== Physics integration =====
    let last = performance.now();

    function step(now) {
        if (pending_nodes > 0) {
            addTailSegment()
            pending_nodes--;
        }

        const dt = clamp((now - last) / 1000, 0, 0.033); // clamp to ~30 FPS step for stability
        last = now;

        // Update cooldown
        hitCooldown = Math.max(0, hitCooldown - dt);

        // Apply gravity
        ball.vy += grav * dt;

        // Integrate
        ball.x += ball.vx * dt;
        ball.y += ball.vy * dt;
        // Update angular motion
        ball.angle += ball.spin * dt;
        // Tiny air damping (optional; set to 1 for perfect energy conservation)
        const air = 0.996;
        ball.vx *= air;
        ball.vy *= air;
        ball.spin *= air;

        // Collisions with world bounds using impulse-based resolution
        // Floor (y = h)
        if (ball.y + radius > h) {
            const penetration = ball.y + radius - h;
            positionalCorrection(0, -1, penetration);
            resolvePlaneContact(0, -1, ball.x, ball.y, rest, muGround);
        }
        // Ceiling (y = 0)
        if (ball.y - radius < 0) {
            const penetration = radius - ball.y;
            positionalCorrection(0, 1, penetration);
            resolvePlaneContact(0, 1, ball.x, ball.y, wallRest, muGround);
        }
        // Left wall (x = 0)
        if (ball.x - radius < 0) {
            const penetration = radius - ball.x;
            positionalCorrection(1, 0, penetration);
            resolvePlaneContact(1, 0, ball.x, ball.y, wallRest, muWall);
        }
        // Right wall (x = w)
        if (ball.x + radius > w) {
            const penetration = ball.x + radius - w;
            positionalCorrection(-1, 0, penetration);
            resolvePlaneContact(-1, 0, ball.x, ball.y, wallRest, muWall);
        }

        // If off-screen far below (after energy degraded), reset near top
        if (ball.y - radius > h + 400) {
            ball.x = w * 0.5;
            ball.y = h * 0.25;
            ball.vx = 120 * (Math.random() * 2 - 1);
            ball.vy = -600;
            ball.spin = 0;
        }

        // Mouse interaction: apply a physically consistent impulse at surface
        const mt = now;
        const mdx = mouse.x - lastMouse.x;
        const mdy = mouse.y - lastMouse.y;
        const mdT = Math.max(0.001, (mt - lastMouse.t) / 1000);
        const mvx = mdx / mdT, mvy = mdy / mdT;
        lastMouse.x = mouse.x;
        lastMouse.y = mouse.y;
        lastMouse.t = mt;

        const dx = mouse.x - ball.x;
        const dy = mouse.y - ball.y;
        const dist = Math.hypot(dx, dy);
        if (dist < radius + 2 && hitCooldown <= 0) {
            // Unit vector from ball to mouse
            let nx = dx / (dist || 1);
            let ny = dy / (dist || 1);

            // --- Pre-hit state snapshots for release computation
            const vBefore = Math.hypot(ball.vx, ball.vy);
            const angBefore = Math.atan2(ball.vy, ball.vx);

            // Approach speed along n (mouse moving into the ball gives bigger hit)
            const approach = -(nx * mvx + ny * mvy); // >0 if cursor moving into the center
            const base = 420 + clamp(approach * 0.6, -200, 800);

            // External impulse vector applied at the surface point nearest the mouse
            const Jx = -nx * base;
            const Jy = -ny * base - 120; // slight upward bias is just more fun

            // Apply at contact point r = -n * radius to generate the correct spin
            const rx = -nx * radius;
            const ry = -ny * radius;
            applyImpulseAtPoint(Jx, Jy, rx, ry);

            // Spool coupling: project hit impulse along exit tangent and kick rope tail
            const worldExitAngle = exitAngle + ball.angle;
            const ex = Math.cos(worldExitAngle), ey = Math.sin(worldExitAngle);
            const tx = -ey, ty = ex; // tangent at exit
            const J_along_exit = Jx * tx + Jy * ty;
            // Impart velocity to tail via Verlet: prev = curr - v*dt
            const kick = J_along_exit * 0.02; // tuned
            if (ropeInited) {
                const tail = nodes.length - 1;
                nodes[tail].x = nodes[tail].x - tx * kick;
                nodes[tail].y = nodes[tail].y - ty * kick;
            }

            // --- Post-hit state for release computation
            const vAfter = Math.hypot(ball.vx, ball.vy);
            const angAfter = Math.atan2(ball.vy, ball.vx);
            // Smallest signed angle difference
            let dTheta = angAfter - angBefore;
            dTheta = ((dTheta + Math.PI) % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2) - Math.PI;
            const dP = mass * Math.max(0, vAfter - vBefore); // only count momentum increases
            const Jmag = Math.hypot(Jx, Jy);

            // Convert dynamics (impulse/angle/momentum) to release length
            let dL = k_release_impulse * Jmag + k_release_angle * Math.abs(dTheta) + k_release_momentum * dP;
            // Hard cap per hit using existing velocity cap scale
            dL = clamp(dL, 0, v_max_feed * 0.25);

            if (dL > 0) {
                L_free = clamp(L_free + dL, 0, L_total);
                ensureRopeForLength(L_free);
            }
            // Optional: nudge exitAngle toward the hit angle (local coords)
            const hitLocal = Math.atan2(ny, nx) - ball.angle;
            let dAng = ((hitLocal - exitAngle + Math.PI) % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2) - Math.PI;
            exitAngle += dAng * 0.05;

            hitCooldown = 0.09; // short cooldown to avoid continuous pushing
        }

        // Spool mechanics: compute anchor and tangent (release occurs only on hit above)
        // Compute anchor and tangent based on current exitAngle & wound radius
        const r_spool = r_spool_of(L_free);
        const worldExitAngle2 = exitAngle + ball.angle;
        const nx_e = Math.cos(worldExitAngle2), ny_e = Math.sin(worldExitAngle2);
        const tx_e = -ny_e, ty_e = nx_e;
        const anchorX = ball.x + nx_e * r_spool;
        const anchorY = ball.y + ny_e * r_spool;
        // Smooth the anchor motion to reduce head jitter
        if (!ropeInited) {
            // Initialize rope if needed
            initRope(anchorX, anchorY, tx_e, ty_e);
            ropeAX = anchorX;
            ropeAY = anchorY;
        }
        const lerpA = 1 - Math.pow(1 - ANCHOR_SMOOTH, Math.max(1, (now - (last - 1000 * dt)) / 16.666));
        ropeAX += (anchorX - ropeAX) * lerpA;
        ropeAY += (anchorY - ropeAY) * lerpA;

        // Exit point simply rides with the ball's spin (no back-reaction from rope).
        exitAngle += ball.spin * dt;

        // Rope dynamics: substep integrate under gravity, solve IK, then smooth curvature
        const steps = Math.max(1, ROPE_SUBSTEPS);
        const dts = dt / steps;
        for (let s = 0; s < steps; s++) {
            integrateRope(dts);
            satisfyRope(ropeAX, ropeAY);
            smoothRope(0.10);
        }

        // Render
        ctx.clearRect(0, 0, c.clientWidth, c.clientHeight);
        // Background
        ctx.fillStyle = '#36a';
        ctx.fillRect(0, 0, c.clientWidth, c.clientHeight);

        // Ground line for reference
        ctx.strokeStyle = 'rgba(255,255,255,0.08)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, c.clientHeight - 1);
        ctx.lineTo(c.clientWidth, c.clientHeight - 1);
        ctx.stroke();

        // Draw rope and a tiny stick/slip indicator at the anchor
        drawRope(ball.x + nx_e * radius, ball.y + ny_e * radius);
        ctx.save();

        // UI meter
        ctx.fillStyle = 'rgba(255,255,255,0.85)';
        ctx.font = '14px system-ui, sans-serif';
        ctx.fillText('yarn: ' + formatMeters(L_free), 12, 20);

        // Draw ball on top
        drawYarn(ball.x, ball.y, radius);

        requestAnimationFrame(step);
    }

    requestAnimationFrame(step);
});
