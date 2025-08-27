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
    const mu_spool_static = 0.5;
    const mu_spool_dynamic = 0.35;
    const N0 = 5 * mass * grav;             // baseline normal force proxy
    const k_spin = 0.8 * mass * grav * 0.001; // spin contribution to normal (tuned small)
    const k_tight = 10;                     // how strongly rope tries to match surface speed
    const v_max_feed = 800;                 // px/s cap for feed rate

    // Exit state
    let exitAngle = Math.random() * Math.PI * 2; // local angle on ball

    // IK rigid-rod rope (visual-only; one-way coupling)
    const SEG_LEN_MIN = 6;   // minimum segment length
    const SEG_LEN_MAX = 14;  // keep segments within this visual range
    let SEG_LEN = 10;  // target segment length used in constraints
    const ROPE_ITERS = 5;   // constraint iterations (firmness)
    const ROPE_DAMP = 0.985; // per-step velocity damping for less rubbery feel
    const nodes = [];
    let ropeInited = false;

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
        // keep SEG_LEN in [min,max] as rope grows (optional smoothing)
        SEG_LEN = clamp(SEG_LEN, SEG_LEN_MIN, SEG_LEN_MAX);

        const target = targetCountForLength(L);
        // Only grow; never shrink (no re-spool)
        while (nodes.length < target) addTailSegment();
    }

    function integrateRope(dt) {
        for (let i = 1; i < nodes.length; i++) { // skip pinned head (i=0)
            const n = nodes[i];
            const vx = (n.x - n.px) * ROPE_DAMP;
            const vy = (n.y - n.py) * ROPE_DAMP + grav * dt * dt * 0.4;
            n.px = n.x;
            n.py = n.y;
            n.x += vx;
            n.y += vy;
        }
    }

    function doPass(fromIndex, toIndex, step, moveA, moveB) {
        for (let i = fromIndex; i !== toIndex; i += step) {
            const a = nodes[i - 1], b = nodes[i];
            let dx = b.x - a.x, dy = b.y - a.y;
            let d = Math.hypot(dx, dy) || 1e-6;
            const diff = (d - SEG_LEN) / d;
            if (i - 1 !== 0) {
                a.x += dx * diff * moveA;
                a.y += dy * diff * moveA;
            }
            b.x -= dx * diff * moveB;
            b.y -= dy * diff * moveB;
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

    function ropeTangentialSpeed(dt, tx_e, ty_e) {
        if (nodes.length < 2) return 0;
        const n1 = nodes[1];
        const vx1 = (n1.x - n1.px) / Math.max(1e-6, dt);
        const vy1 = (n1.y - n1.py) / Math.max(1e-6, dt);
        return vx1 * tx_e + vy1 * ty_e;
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

    function drawRope() {
        if (nodes.length < 2) return;
        ctx.lineCap = 'round';
        for (let i = 1; i < nodes.length; i++) {
            const t = i / (nodes.length - 1);
            ctx.strokeStyle = i < 3 ? 'rgba(255,200,220,0.9)' : 'rgba(230,160,190,0.85)';
            ctx.lineWidth = 3.1 * (1 - 0.65 * t); // thicker + gentle taper (from prior tweak)
            ctx.beginPath();
            ctx.moveTo(nodes[i - 1].x, nodes[i - 1].y);
            ctx.lineTo(nodes[i].x, nodes[i].y);
            ctx.stroke();
        }
    }


    function formatMeters(px) {
        const m = px / pxPerMeter;
        return m.toFixed(1) + ' m';
    }

    // ===== Physics integration =====
    let last = performance.now();

    function step(now) {
        const dt = clamp((now - last) / 1000, 0, 0.033); // clamp to ~30 FPS step for stability
        last = now;

        // Update cooldown
        hitCooldown = Math.max(0, hitCooldown - dt);

        const w = c.clientWidth, h = c.clientHeight;

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
            // Optional: nudge exitAngle toward the hit angle (local coords)
            const hitLocal = Math.atan2(ny, nx) - ball.angle;
            let dAng = ((hitLocal - exitAngle + Math.PI) % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2) - Math.PI;
            exitAngle += dAng * 0.05;

            hitCooldown = 0.09; // short cooldown to avoid continuous pushing
        }

        // Spool mechanics: stick/slip and length update
        // Compute anchor and tangent based on current exitAngle & wound radius
        const r_spool = r_spool_of(L_free);
        const worldExitAngle2 = exitAngle + ball.angle;
        const nx_e = Math.cos(worldExitAngle2), ny_e = Math.sin(worldExitAngle2);
        const tx_e = -ny_e, ty_e = nx_e;
        const anchorX = ball.x + nx_e * r_spool;
        const anchorY = ball.y + ny_e * r_spool;
        // Initialize rope if needed
        if (!ropeInited) initRope(anchorX, anchorY, tx_e, ty_e);

        // Ball surface velocity at anchor (including translation)
        const cv = contactVelocity(ball.vx, ball.vy, ball.spin, nx_e * r_spool, ny_e * r_spool);
        const v_ball_t = cv.cx * tx_e + cv.cy * ty_e;
        // One-way coupling: rope follows yarn, never affects it.
        // Feed rope out when the surface moves along +tangent; never re-spool.
        const feed = clamp(v_ball_t, 0, v_max_feed);
        L_free = clamp(L_free + feed * dt, 0, L_total);
        ensureRopeForLength(L_free);
        // Exit point simply rides with the ball's spin (no back-reaction from rope).
        exitAngle += ball.spin * dt;
        // For UI color: "sticking" means not feeding this frame
        const sticking = feed < 1e-3;

        // Rope dynamics: integrate under gravity, then solve IK rigid rods
        integrateRope(dt);
        satisfyRope(anchorX, anchorY);

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
        drawRope();
        ctx.save();
        ctx.strokeStyle = sticking ? 'rgba(120,255,160,0.8)' : 'rgba(255,140,120,0.9)';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(anchorX, anchorY);
        ctx.lineTo(anchorX + tx_e * 18, anchorY + ty_e * 18);
        ctx.stroke();
        ctx.restore();

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
