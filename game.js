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

  // World params
  const grav = 1200;      // px/s^2
  const rest = 0.72;      // bounciness
  const wallRest = 0.6;   // side bounciness
  const radius = 40;      // yarn radius in px

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
  let mouse = { x: 0, y: 0, down: false };
  let lastMouse = { x: 0, y: 0, t: performance.now() };
  let hitCooldown = 0; // seconds

  function setMouse(e) {
    const rect = c.getBoundingClientRect();
    mouse.x = (e.clientX - rect.left);
    mouse.y = (e.clientY - rect.top);
  }
  c.addEventListener('mousemove', (e) => setMouse(e));
  c.addEventListener('mousedown', (e) => { mouse.down = true; setMouse(e); });
  c.addEventListener('mouseup', () => { mouse.down = false; });
  // Prevent text selection on drag
  c.addEventListener('touchstart', (e) => { const t=e.touches[0]; setMouse(t); mouse.down=true; e.preventDefault(); });
  c.addEventListener('touchmove', (e) => { const t=e.touches[0]; setMouse(t); e.preventDefault(); });
  c.addEventListener('touchend', () => { mouse.down=false; });

  // Utility
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

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
      { count: 18, ro: 1.02, w: 1.4, a: 0.40, hue: '#f9a9c8', tilt: -0.35 }, // lighter strands
      { count: 16, ro: 0.96, w: 1.2, a: 0.30, hue: '#8a2b4f', tilt: 0.55 },  // darker in-between
      { count: 12, ro: 0.88, w: 1.0, a: 0.22, hue: '#6b1f3d', tilt: 1.15 }   // deeper grooves
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

  // Physics integration
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
    ball.spin *= 0.996; // air damping

    // Collisions: floor and walls
    if (ball.y + radius > h) {
      ball.y = h - radius;
      ball.vy *= -rest;
      // small friction on bounce
      ball.vx *= 0.98;
      // couple linear velocity into spin (rolling tendency)
      ball.spin += (ball.vx / radius) * 0.45;
    }
    if (ball.y - radius < 0) {
      ball.y = radius;
      ball.vy *= -wallRest;
    }
    if (ball.x - radius < 0) {
      ball.x = radius;
      ball.vx *= -wallRest;
      ball.spin *= 0.9;
    } else if (ball.x + radius > w) {
      ball.x = w - radius;
      ball.vx *= -wallRest;
      ball.spin *= 0.9;
    }

    // If off-screen far below (after energy degraded), reset near top
    if (ball.y - radius > h + 400) {
      ball.x = w * 0.5; ball.y = h * 0.25; ball.vx = 120*(Math.random()*2-1); ball.vy = -600;
    }

    // Mouse interaction: bounce when cursor intersects
    // Compute mouse velocity magnitude (for flicks)
    const mt = now;
    const mdx = mouse.x - lastMouse.x;
    const mdy = mouse.y - lastMouse.y;
    const mdT = Math.max(0.001, (mt - lastMouse.t) / 1000);
    const mvx = mdx / mdT, mvy = mdy / mdT;
    lastMouse.x = mouse.x; lastMouse.y = mouse.y; lastMouse.t = mt;

    const dx = mouse.x - ball.x;
    const dy = mouse.y - ball.y;
    const dist = Math.hypot(dx, dy);
    if (dist < radius + 2 && hitCooldown <= 0) {
      // Direction away from cursor to push ball
      let nx = dx / (dist || 1);
      let ny = dy / (dist || 1);
      // Base impulse up and away, stronger if mouse moving into the ball
      const approach = -(nx*mvx + ny*mvy); // positive if moving towards center
      const base = 420 + clamp(approach * 0.6, -200, 800);
      ball.vx += -nx * base;
      ball.vy += -ny * base - 120; // a bit extra upward bias
      // add some spin from tangential mouse swipes
      const tangential = (-ny * mvx + nx * mvy);
      ball.spin += (tangential / radius) * 0.0025;
      hitCooldown = 0.09; // short cooldown to avoid continuous pushing
    }

    // Render
    ctx.clearRect(0, 0, c.clientWidth, c.clientHeight);
    // Background
    ctx.fillStyle = '#111';
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