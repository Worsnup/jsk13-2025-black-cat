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
    const rest = 0.72;        // coefficient of restitution (floor/ceiling)
    const wallRest = 0.6;     // side restitution
    const muGround = 0.6;     // Coulomb friction (floor/ceiling) — dimensionless
    const muWall = 0.5;       // Coulomb friction (walls)
    const radius = 40;        // yarn radius in px

    // Rigid body params (solid sphere about center: I = 2/5 m r^2)
    const mass = 1;           // arbitrary units — only ratios matter here
    const inertia = (2 / 5) * mass * radius * radius;

    // Ball state
    const ball = {
        x: c.clientWidth * 0.5,
        y: c.clientHeight * 0.25,
        vx: 140 * (Math.random() * 2 - 1),
        vy: 0,
        angle: 0,   // rotation around Z (radians)
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
        const layers = [
            {count: 18, ro: 1.02, w: 1.4, a: 0.40, hue: '#f9a9c8', tilt: -0.35}, // lighter strands
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

            hitCooldown = 0.09; // short cooldown to avoid continuous pushing
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

        drawYarn(ball.x, ball.y, radius);

        requestAnimationFrame(step);
    }

    requestAnimationFrame(step);
});
