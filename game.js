document.addEventListener('DOMContentLoaded', () => {
  // Create and attach canvas
  const content = document.getElementById('content');
  const c = document.createElement('canvas');
  const ctx = c.getContext('2d');
  content.innerHTML = '';
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
    vy: 0
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
    // Shadow
    ctx.save();
    ctx.globalAlpha = 0.25;
    ctx.fillStyle = '#000';
    ctx.beginPath();
    ctx.ellipse(x + r*0.2, y + r*0.9, r*0.9, r*0.35, 0, 0, Math.PI*2);
    ctx.fill();
    ctx.restore();

    // Base ball
    const grd = ctx.createRadialGradient(x - r*0.3, y - r*0.3, r*0.2, x, y, r);
    grd.addColorStop(0, '#f6a'); // light pinkish
    grd.addColorStop(1, '#c26'); // darker yarn
    ctx.fillStyle = grd;
    ctx.beginPath();
    ctx.arc(x, y, r, 0, Math.PI*2);
    ctx.fill();

    // Yarn strands (simple arcs)
    ctx.strokeStyle = 'rgba(255,255,255,0.3)';
    ctx.lineWidth = 2;
    for (let i = -r; i <= r; i += r/5) {
      ctx.beginPath();
      ctx.arc(x - r*0.3, y + i*0.15, r*1.05, -Math.PI*0.1, Math.PI*0.9);
      ctx.stroke();
    }
    ctx.strokeStyle = 'rgba(80,0,40,0.35)';
    ctx.lineWidth = 1.5;
    for (let i = -r; i <= r; i += r/4.5) {
      ctx.beginPath();
      ctx.arc(x + r*0.2, y + i*0.18, r*0.9, Math.PI*0.1, Math.PI*1.1);
      ctx.stroke();
    }

    // Center highlight
    ctx.beginPath();
    ctx.strokeStyle = 'rgba(255,255,255,0.25)';
    ctx.lineWidth = 2;
    ctx.arc(x, y, r*0.65, -Math.PI*0.2, Math.PI*0.6);
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

    // Collisions: floor and walls
    if (ball.y + radius > h) {
      ball.y = h - radius;
      ball.vy *= -rest;
      // small friction on bounce
      ball.vx *= 0.98;
    }
    if (ball.y - radius < 0) {
      ball.y = radius;
      ball.vy *= -wallRest;
    }
    if (ball.x - radius < 0) {
      ball.x = radius;
      ball.vx *= -wallRest;
    } else if (ball.x + radius > w) {
      ball.x = w - radius;
      ball.vx *= -wallRest;
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